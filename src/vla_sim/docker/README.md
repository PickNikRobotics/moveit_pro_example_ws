# Inference server

`vla_inference_server.py` serves a LeRobot checkpoint (pi0.5, SmolVLA, ...)
over HTTP for the `Stack Cubes with the VLA Policy` objective. The workspace
`docker-compose.yaml` completes MoveIt Pro's `inference_server` service with
this directory's image. Model and device selection live in
`../config/vla_serving.yaml`.

## Running it

The default checkpoint resolves the gated `google/paligemma` tokenizer on first
load, so sign in with an account that has accepted the
[PaliGemma license](https://huggingface.co/google/paligemma-3b-pt-224):

```bash
hf auth login
export HF_TOKEN="$(hf auth token)"
moveit_pro build
moveit_pro run -c vla_sim --with-inference-server
```

MoveIt Pro automatically selects CPU, NVIDIA, Jetson, or AMD from the host;
there is no accelerator option to set. Existing `hf` CLI credentials are used
by the command above without putting the token in shell history.

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
| `HF_TOKEN` | Token for gated or private Hugging Face downloads. |
| `HF_HUB_OFFLINE` | `1` serves only what is already in the cache, with no network access. |
| `VLA_HF_CACHE` | Host path for the Hugging Face cache. Defaults to `../hf_cache`. |
| `VLA_MODELS_DIR` | Host folder mounted at `/models`, for checkpoints stored outside the workspace. Defaults to `../models`. |
| `VLA_TORCH_INDEX` | Package index the image installs torch from, for example `https://download.pytorch.org/whl/cpu` on a machine with no NVIDIA GPU. Defaults to PyPI. |

On a MoveIt Pro ROCm target, the image ignores `VLA_TORCH_INDEX` and installs
LeRobot on AMD's immutable ROCm 7.2.2 PyTorch release image, then verifies the
exact Torch, torchvision, and Triton versions. PyTorch exposes both NVIDIA and
AMD devices as `cuda`; check `/health`'s `accelerator` field to distinguish the
binary backend.

## The HTTP contract

`GET /health` reports `loading` / `ready` / `error` and needs no token.
`POST /infer` requires the deployment's `MOVEIT_FRONTEND_KEY` as a bearer
token. The server speaks plain HTTP and publishes on `127.0.0.1` only, which is
what keeps that token off the network. Two settings decide what code and weights
the container runs, so point both only at sources you trust: `checkpoint`
chooses the robot's actions, and `VLA_TORCH_INDEX` supplies the torch build.
The ROCm target instead pins the complete AMD base image by digest.

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
  -e MOVEIT_FRONTEND_KEY=moveit-secret-key \
  -p 127.0.0.1:8973:8973 vla_inference_server
```

For AMD, build with `--build-arg MOVEIT_TARGET=-rocm7.2.2` and replace
`--gpus all` with `--device /dev/kfd --device /dev/dri`; add the numeric group
that owns `/dev/kfd` with `--group-add "$(stat -c %g /dev/kfd)"` when the host
user does not already have a matching device ACL.

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

The server and benchmark tests run in the container, which mounts this
directory at `/app`:

```bash
docker exec "$(docker ps -qf name=inference_server)" \
  python -m unittest -v test_vla_inference_server test_benchmark_vla_inference
```

After `/health` reports `ready`, collect a repeatable synthetic-input latency
report with the checkpoint's exact camera names and state width:

```bash
docker exec "$(docker ps -qf name=inference_server)" \
  python benchmark_vla_inference.py \
    --camera scene --camera wrist --camera overview --state-dim 8 \
    --warmups 1 --samples 5 \
    --json /tmp/vla-benchmark.json --html /tmp/vla-benchmark.html
```

The harness validates every returned action and records backend provenance,
warmup budget, request latency, output shape, and action digests. Synthetic
images measure serving throughput, not task success. Pass `--payload` with a
recorded request JSON for representative-input comparisons. Copy the two
reports out with `docker cp` before removing the container.
