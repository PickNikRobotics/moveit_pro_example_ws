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
later runs reuse both. Then run **Stack Cubes with the VLA Policy** in the MoveIt Pro Desktop App, and **Reset MuJoCo Sim** between attempts.

Only a *missing* image is built that way, and `moveit_pro build` skips this
service because its compose profile is off by default, so nothing rebuilds the
image when this directory changes. The scripts here are mounted rather than read
from the image, so one that needs a package the existing image predates fails at
import. After editing the Dockerfile or its pinned versions, drop the image and
let the next run build it:

```bash
moveit_pro down
docker rmi moveit_pro-inference_server:latest
```

The command above uses the default CPU/CUDA image tag. On AMD, remove
`moveit_pro-inference_server-rocm:latest` instead. A named instance uses its
Compose project name in place of `moveit_pro`; `docker images` confirms the tag. `moveit_pro down` first
because Docker refuses to remove an image a container still references, and a
stopped container counts.

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

## AMD GPUs

With a launcher that supports AMD VLA inference, no vendor-specific workspace settings are needed. The launcher derives `MOVEIT_INFERENCE_IMAGE_SUFFIX` from its selected target: empty for CPU, CUDA, and Jetson, `-rocm` for AMD on x86_64. That one value selects both the Dockerfile stage and the image tag, so a cached CPU/CUDA image cannot satisfy an AMD launch. An arm64 AMD host serves from the CPU image. The ROCm stage pins its own AMD PyTorch distribution, independent of the Runtime image's ROCm version, and uses direct `/dev/kfd` and `/dev/dri` access; the AMD Container Toolkit is optional. The server reports its Torch and HIP versions at startup. PyTorch's `cuda` device name also means GPU execution on ROCm.

The ROCm image targets Linux `amd64` and refuses an automatic CPU fallback when the GPU is
unavailable. `device: cpu` remains an explicit opt-out. CUDA/CPU images retain their existing Torch
versions and `VLA_TORCH_INDEX` override; ROCm preserves the libraries from its pinned base image.

## Qualcomm Hexagon NPU (pi0.5)

Select `backend: qnn` and set `qnn_bundle` to a local compiled-context JSON
manifest in `../config/vla_serving.yaml`. For example:

```yaml
backend: qnn
qnn_bundle: /models/pi05-qnn/bundle.json
checkpoint: PickNikRobotics/pi05_kinova_gen3_cube_stack_sim
checkpoint_revision: "823bd5e3a82a961df9a5dfd0a1374358cd2d368e"
policy_class: pi05
fps: 10.0
state_dim: 8
```

The backend runs native `QnnGraph_execute` on IQ9 HTP devices, not an ONNX
Runtime session or a PyTorch NPU device. `device` and `int8` apply only to
PyTorch; QNN placement and precision come from the compiled bundle. GR00T
N1.7 remains a PyTorch policy; GR00T NPU support is not implemented.

### Compiled bundle

The schema-1 manifest identifies `model_id` and the immutable source
`model_revision`, `camera_names` in training order, `request_state_dim`,
`model_action_dim`, `state_columns`, `normalization`, `action_mapping`,
`chunk_size`, `max_action_dim`, `denoising_steps`, `expert_output`, `dt`,
`noise_seed`, `stats`, `tokenizer`, `graphs`, `device_placement`, and `precision`.
Paths to graph directories, statistics and tokenizer are absolute or relative
to the manifest, so the bundle can be mounted read-only under `/models`.
No checkpoint weights or tokenizer are fetched from the Hub on the QNN path.
`checkpoint` must equal `model_id`; a nonempty `checkpoint_revision` must equal
`model_revision`. `fps` and `state_dim` must match, or be 0 to use the bundle's
trained values. `/status` reports the bundle's actual immutable source revision.

Supported graph ABIs are `aihub.pi05.four_graphs.euler.v1` (vision encoder,
token embedding, backbone, action expert) and
`kinova.pi05.six_graphs.euler.v1` (the `backbone` entry is replaced by
`prefix_0_6`, `prefix_6_12` and `prefix_12_18`). They require three 224x224 RGB
cameras, 200 language tokens, 50-step
chunks padded to 32 action dimensions and ten already-integrated expert
updates. Host preprocessing uses the saved QUANTILES state/action statistics
and absolute trained joint order; relative-action bundles are unsupported.
The statistics JSON contains finite `observation.state.q01`,
`observation.state.q99`, `action.q01` and `action.q99` arrays. The tokenizer
must be a complete local Hugging Face tokenizer directory. Vision/token run
on HTP1; backbone/expert run on HTP0.

Each graph directory contains `model.bin` (native compiled context), `abi.json`
(from `qnn-context-binary-utility`) and `manifest.json`. The latter declares
`context_sha256` and a one-to-one `output_aliases` map from compiler output
names to source graph names. The backend snapshots each context once, checks
its digest, and loads those same bytes with QNN's default (nonpersistent)
context config, then releases them after deserialization, so a second host
weight copy does not stay resident. Fixed input/output names, shapes and dtypes are checked before serving;
quantized, dynamic or sparse graph IO is refused. Compilation precision is fixed by the bundle:
vision FP16/W8A16, token FP16, backbone W4A16/W8A16 or FP16 split, and expert
W8A16/W16A16/native-RMS FP16. Serving support does not qualify a compiled
checkpoint's accuracy or authorize real-robot deployment; validate that
checkpoint independently before using its actions.

### Native runtime requirements

The standard inference image intentionally does **not** install QAIRT. Use a
board environment with QAIRT 2.49 (QNN core API 2.38), compatible compiled
contexts and the HTP/FastRPC devices accessible to the inference process.
Install or mount the native libraries and DSP skeletons, retain their library
search paths and permissions, and set:

- `QNN_HTP_LIBRARY`: full path to `libQnnHtp.so` (default
  `/usr/lib/libQnnHtp.so`).
- `LD_LIBRARY_PATH` and `ADSP_LIBRARY_PATH`: the board's QAIRT host libraries
  and compatible Hexagon skeleton directories.
- `QNN_HTP_BURST`: the native burst/TURBO performance configuration is applied
  by default; set `0` to leave the HTP power configuration untouched.
- `OMP_NUM_THREADS=1`: bound host image preprocessing threads.
- Docker `--ulimit memlock=-1:-1`: allow the native FastRPC weight mappings.
  The default Docker limit is 8 MiB; the six-context IQ9 bundle failed native
  load with QNN status 1002 (`QNN_COMMON_ERROR_MEM_ALLOC`) at that limit, and
  loaded with unlimited memlock. This is a container setting, not a host
  permission or system-library change.

Hardware/library provisioning is a host deployment requirement, not a new
launcher image target: the shipped `inference_server` service does not mount
QAIRT, the HTP devices or a compiled bundle, so `backend: qnn` through the
launcher parks in `error`. Launcher deployment is tracked as a follow-up.
Do not expose a plain HTTP listener over the network;
use the existing remote TLS proxy and inference-only bearer key. Missing QAIRT,
unsupported ABI, mismatched source metadata or failed native warmup parks the
existing server in `error`, with no CPU/GPU neural fallback. Restore
`backend: pytorch` to return to the existing path; replacing the serving YAML
reloads the process when the policy is idle and any QNN load has finished,
freeing the native sessions first. A failed native release retains ownership
for retry instead of dropping the runner or claiming a ready NPU.

QNN keeps the same `/infer` RTC fields: `prev_chunk_left_over` contains the
unexecuted normalized `action_chunk_raw` tail, `inference_delay` is the frozen
prefix length, and `guidance_horizon` is soft width beyond that prefix (0 uses
the configured default). Malformed carryover is refused before graph execution.
After each native integrated expert update, host residual arithmetic uses stock
LeRobot RTC guidance with the configured schedule and weight cap; it never takes
a second Euler update. Without carryover the native output and seeded noise
sequence are unchanged. Every response includes physical `action_chunk`,
normalized trained-width `action_chunk_raw`, and the bundle's trained `dt`.
Warmup preserves the noise stream for the first real request.

## Environment

Set these in the workspace `.env`; all are optional.

| Variable | Effect |
| --- | --- |
| `HF_TOKEN` | Explicit fallback for gated or private checkpoints configured outside Trainer. |
| `HF_HUB_OFFLINE` | `1` serves only what is already in the cache, with no network access. |
| `VLA_HF_CACHE` | Host path for the Hugging Face cache. Defaults to `../hf_cache`. |
| `VLA_MODELS_DIR` | Host folder mounted at `/models`, for checkpoints stored outside the workspace. Defaults to `../models`. |
| `VLA_CONFIG_DIR` | Host directory mounted at `/vla_config`, the one Trainer writes `vla_serving.yaml` into. Defaults to `src/<MOVEIT_CONFIG_PACKAGE>/config`; set it for a package nested elsewhere, such as `src/moveit_pro_kinova_configs/kinova_sim/config`. |
| `VLA_TORCH_INDEX` | Package index the image installs torch from, for example `https://download.pytorch.org/whl/cpu` on a machine with no NVIDIA GPU. Defaults to PyPI for CPU/CUDA images; ignored by the pinned ROCm image. |

## The HTTP contract

`GET /health` reports `loading` / `ready` / `error` and needs no token.
With QNN, `int8` is always `false` and `qnnPrecision` reports the bundle's
per-graph `precision` map, on both `/health` and `/status`.
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

The inference-server tests need `lerobot` and `torch`, so they run in the
inference image. From the workspace root, use a fresh CPU-only container without
starting or changing a running server:

```bash
docker run --rm --network none --entrypoint python \
  -e PYTHONDONTWRITEBYTECODE=1 -e OMP_NUM_THREADS=1 \
  -v "$PWD/src/vla_sim/docker:/app:ro" -w /app \
  moveit_pro-inference_server:latest \
  -m unittest -v test_vla_inference_server test_vla_qnn_backend
```

QNN tests use fake native sessions and need neither QAIRT, NPU hardware, a Hub
connection nor checkpoint weights. They cover bundle selection/validation,
off-path output and RNG parity, stock LeRobot RTC arithmetic, malformed
carryover and the authenticated `/status`/`/infer` round-trip.
