# Inference server

`vla_inference_server.py` serves a LeRobot checkpoint (pi0.5, SmolVLA, ...)
over HTTP for the `Stack Cubes with the VLA Policy` objective. The workspace
`docker-compose.yaml` completes MoveIt Pro's `inference_server` service with
this directory's image. Model and device selection live in
`../config/vla_serving.yaml`.

## Running it

For a checkpoint produced by Trainer, add the Hugging Face token once in the
**Train** tab and choose **Configure for ExecutePolicy**. MoveIt Pro provisions
that token to the inference server without adding it to this workspace or the
container environment.

The stock checkpoint resolves the gated `google/paligemma` tokenizer on first
load and has no Trainer run to configure. For that checkpoint, export a token
from an account that has accepted the
[PaliGemma license](https://huggingface.co/google/paligemma-3b-pt-224):

```bash
export HF_TOKEN=hf_your_token_here
moveit_pro build
moveit_pro run -c vla_sim --with-inference-server
```

The first run builds the image and downloads the checkpoint into `../hf_cache/`;
later runs reuse both. Then run **Stack Cubes with the VLA Policy** in the web
UI, and **Reset MuJoCo Sim** between attempts.

Model loading takes a minute or more. To keep the model warm across restarts of
the stack, run the server on its own in one terminal and the stack, without
`--with-inference-server`, in another:

```bash
# Terminal 1: the server, which prints its loading and ready status.
moveit_pro run --only-inference-server
# Terminal 2: restart this as often as you like; the loaded model survives.
moveit_pro run -c vla_sim
```

Pick one mode per session. Passing `--with-inference-server` while a
side-started server is running adopts that container, so stopping the stack
stops the server too.

Serving a different checkpoint also takes two edits in
`../objectives/stack_cubes_with_the_vla_policy.xml`, because the request has to
match what the checkpoint was trained on: set `image_names` to its camera names,
which the server rejects the request for if they differ, and set `dt` to 1/`fps`.

## Environment

Set these in the workspace `.env`; all are optional.

| Variable | Effect |
| --- | --- |
| `HF_TOKEN` | Explicit fallback for gated or private checkpoints configured outside Trainer. |
| `HF_HUB_OFFLINE` | `1` serves only what is already in the cache, with no network access. |
| `VLA_HF_CACHE` | Host path for the Hugging Face cache. Defaults to `../hf_cache`. |
| `VLA_MODELS_DIR` | Host folder mounted at `/models`, for checkpoints stored outside the workspace. Defaults to `../models`. |
| `VLA_CONFIG_DIR` | Host directory mounted at `/vla_config`, the one Trainer writes `vla_serving.yaml` into. Defaults to `src/<MOVEIT_CONFIG_PACKAGE>/config`; set it for a package nested elsewhere, such as `src/moveit_pro_kinova_configs/kinova_sim/config`. |
| `VLA_TORCH_INDEX` | Package index the image installs torch from, for example `https://download.pytorch.org/whl/cpu` on a machine with no NVIDIA GPU. Defaults to PyPI. |

## The HTTP contract

`GET /health` reports `loading` / `ready` / `error` and needs no token.
Authenticated `GET /status` also reports the selected checkpoint and immutable
revision so Trainer can confirm the exact model is loaded. It also reports
`configRevision`, the sha256 of the serving file this process loaded, which
Trainer compares with the file it wrote, and `trainerHandoffVersion`, the
`moveit_pro_trainer_handoff_version` that file declares. `POST /infer` and
`GET /status` require `MOVEIT_INFERENCE_KEY` as a bearer token. `moveit_pro run
--with-inference-server` and `--only-inference-server` derive it from the
deployment's frontend key and pass it to the local inference server, and
`moveit_pro inference-key` prints the same value for a remote host. The server
itself speaks plain HTTP. In the local deployment it publishes on `127.0.0.1`
only, which keeps the token off the network; off-box, a TLS proxy in front of it
is what keeps the token off the wire. TLS covers the robot-to-proxy hop only:
the proxy forwards to the server over plain HTTP on the stack's private
compose network, which nothing else may join. Replacing `vla_serving.yaml` or the
Trainer-provisioned credential reloads the process so the old model's GPU
memory is released first.
A reload waits for a running policy: it holds off until `/infer` has been quiet
for one chunk of playback (at least `RELOAD_IDLE_SECONDS`), and `/status` reports
`reloadPending: true` meanwhile.
Only a call hung past `INFER_ABANDONED_SECONDS` is reloaded over. A pinned
`checkpoint_revision` downloads that whole repository revision into the cache.
Two settings decide what code and weights the container runs, so point both only
at sources you trust: `checkpoint` chooses the robot's actions, and
`VLA_TORCH_INDEX` supplies the torch build.

## Serving from another machine

`remote/` holds a standalone Compose file and nginx configuration for running
this server on a GPU host instead of the robot. Do not merge them with the
local MoveIt Pro deployment; they are a separate stack on a separate machine.
[Connect a VLA Policy](https://docs.picknik.ai/how_to/vla/connect_a_vla_policy/)
carries the full procedure. What this repository adds:

- nginx terminates TLS and proxies `POST /infer` and `GET /status`. `/health`
  stays internal. Set `INFERENCE_TLS_DIR` to a directory holding
  `fullchain.pem` and `privkey.pem`, whose Subject Alternative Name covers the
  exact hostname or address the robot connects to. The proxy runs non-root, so
  both files must be readable by the configured UID, or nginx fails its own
  config test at startup.
- Keep `MOVEIT_INFERENCE_KEY` in a protected env file on the GPU host, never in
  a workspace `.env` under version control. Print it on the robot with
  `moveit_pro inference-key`, which derives it from that deployment's frontend
  key, and provision it on the GPU host. It carries no authority over the
  Runtime's other endpoints.
- The TLS proxy image is pinned by digest, so for a security update you bump the
  digest in `remote/docker-compose.yaml` and recreate the `tls` service.
- Every Objective calling this policy must set `policy_call_timeout` above the
  adapter's `http_timeout` (9.0s by default, `MOVEIT_INFERENCE_HTTP_TIMEOUT`).
  `policy_call_timeout` defaults to 3.0, under that budget. The adapter is
  single-threaded, so a call that outlives its caller queues the next run's
  first request behind a stale one. `stack_cubes_with_the_vla_policy.xml`
  sets 10.0.
- On the robot, `INFER_URL` points at `https://<gpu-host>:8443/infer` and `MOVEIT_INFERENCE_CA_FILE` gives the path of the certificate to trust when it is not from a public CA. Give an absolute path. Compose resolves a relative one against /opt/moveit_pro rather than your shell's directory, and a bare filename becomes a named volume; either way the adapter then refuses to start. With an HTTPS `INFER_URL`, `moveit_pro run --with-inference-server` and `--only-inference-server` exit with an error, since the local inference server has no TLS listener. Unset `INFER_URL` to use the local inference server again.

## Running the image outside compose

Compose builds the image and supplies the environment it needs. By hand, from
this directory:

```bash
docker build -f Dockerfile.vla_inference_server -t vla_inference_server .
docker run --rm --user "$(id -u):$(id -g)" \
  --gpus all \
  -e HOME=/tmp -e USER=vla \
  -v "$PWD/../hf_cache:/hf" -e HF_HOME=/hf -e HF_TOKEN="$HF_TOKEN" \
  -v "$PWD/../config:/vla_config:ro" \
  -e MOVEIT_INFERENCE_KEY="$(moveit_pro inference-key)" \
  -p 127.0.0.1:8973:8973 vla_inference_server
```

`--user` keeps bind-mounted files from being written as uid 1000, which means
the image's own passwd entry no longer applies, so `HOME` and `USER` have to be
set for torch's import-time cache setup. `--gpus all` exposes the GPU to the
container; without it `device: auto` silently serves on cpu. Omit the flag on a
machine without an NVIDIA GPU, where it fails outright. The Hugging Face cache
mount makes checkpoint downloads persist, and the config mount is where the
server reads which checkpoint to load. The image's entrypoint already runs the
server, so anything after the image name is appended as arguments to it, and
`--checkpoint <dir-or-hf-id>` overrides the config.

## Tests

`test_vla_inference_server.py` needs `lerobot` and `torch`, so it runs in the
container, which mounts this directory at `/app`:

```bash
docker exec "$(docker ps -qf name=inference_server)" python -m unittest -v test_vla_inference_server
```
