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

## Qualcomm Hexagon NPU (pi0.5 and GR00T N1.7)

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

The backend runs native `QnnGraph_execute` on IQ9 (Hexagon v73) and IQ10
(Hexagon v81) HTP devices, not an ONNX Runtime session or a PyTorch NPU device.
`device` and `int8` apply only to PyTorch; QNN placement and precision come from
the compiled bundle, whose `graph_abi` also selects the policy family, so
`policy_class` may stay empty. Contexts are compiled per Hexagon version: an
IQ9 bundle does not load on an IQ10 or the reverse.

### Compiled bundle

The schema-1 manifest identifies `model_id` and the immutable source
`model_revision`, `camera_names` in training order, `request_state_dim`,
`model_action_dim`, `state_columns`, `normalization`, `action_mapping`,
`chunk_size`, `max_action_dim`, `denoising_steps`, `expert_output`, `dt`,
`noise_seed`, `stats`, `tokenizer`, `graphs`, `device_placement`, and `precision`.
Paths to graph directories, statistics and tokenizer are absolute or relative
to the manifest, so the bundle can be mounted read-only under `/models`.
No checkpoint weights, tokenizer or `moveit_pro_policy.json` are fetched from the
Hub on the QNN path, so `state_names` and `action_names` always come from
`vla_serving.yaml`.
`checkpoint` must equal `model_id`; a nonempty `checkpoint_revision` must equal
`model_revision`. `fps` and `state_dim` must match, or be 0 to use the bundle's
trained values. `/status` reports the bundle's actual immutable source revision.

Supported graph ABIs are `aihub.pi05.four_graphs.euler.v1` (vision encoder,
token embedding, backbone, action expert) and
`kinova.pi05.six_graphs.euler.v1` (the `backbone` entry is replaced by
`prefix_0_6`, `prefix_6_12` and `prefix_12_18`). They require three 224x224 RGB
cameras, 200 language tokens and chunks padded to 32 action dimensions.
`chunk_size` and `denoising_steps` come from the compiled expert's validated
sampling descriptor; all token/expert chunk, attention and RoPE ports must
agree with that extent. Host preprocessing uses the saved QUANTILES state/action statistics
and absolute trained joint order; relative-action bundles are unsupported.
The statistics JSON contains finite `observation.state.q01`,
`observation.state.q99`, `action.q01` and `action.q99` arrays. The tokenizer
must be a complete local Hugging Face tokenizer directory. `device_placement`
names HTP device 0 or 1 for each graph: the IQ9 has devices 0 and 1 (its
bundles put vision and token on 1, the rest on 0); the IQ10 has one device, 0.

Each graph directory contains `model.bin` (native compiled context), `abi.json`
(from `qnn-context-binary-utility`) and `manifest.json`, which declares
`context_sha256` and a one-to-one `output_aliases` map from compiler output
names to source graph names. The digest is checked against the exact bytes
loaded, and the expert's sampling schedule and every graph's input/output names,
shapes and dtypes before serving. Quantized or sparse graph IO and invalid
extents are refused. Only static IO is supported, and `abi.json` cannot prove a
port static: the compiler/stager must verify that. The expert manifest also
carries `sampling` (below). Compilation precision is fixed by the bundle: vision
FP16/W8A16, token FP16, backbone W4A16/W8A16 or FP16 split, and expert
W8A16/W16A16/native-RMS FP16. Serving support does not qualify a compiled
checkpoint's accuracy or authorize real-robot deployment.

### Supplying a different pi0.5 export

These ABIs support a **fixed uniform decreasing Euler grid**, with
`round((1-time_step)*denoising_steps)` clipped to the precomputed conditioning
table and an expert output of `x_t + integration_dt * velocity`. The expert
already integrates each update; neither off-path serving nor RTC integrates it
a second time. A different step count requires re-exporting/recompiling that
conditioning table **and** the baked negative interval. A different chunk size
also requires matching token suffix/RoPE/attention and expert IO. The three
camera slots, prefix architecture and padded action width remain unchanged.

At successful export/compilation staging time, call `emit_sampling` from
`vla_qnn_export.py` with the **exact ONNX source used for that context** and the
resulting native binary. An equivalent CLI, in the export environment (which
needs NumPy and ONNX, e.g. `onnx==1.19.1`), is:

```bash
python vla_qnn_export.py --source action_expert.onnx \
  --context model.bin --output sampling.json
```

The helper reads the supported graph operations, conditioning-table extent and
Float32 integration constant without running the graph or loading its external
weights. Put the emitted object in the expert's `manifest.json` as `sampling`
and set the bundle's chunk/step fields to its values; the serving image does not
need ONNX. Keep `guidance_horizon` at or below the bundle's `chunk_size` in both
`vla_serving.yaml` (the server refuses to start otherwise) and client requests
(rejected with HTTP 400). The stager is trusted to pair source and output:
matching IO does not prove an unrelated binary came from that ONNX file, so
never use the helper to relabel artifacts. The retained native-RMS expert with digest
`9a7c1e47c3a6fa14d34751fe6f418b9d08ca831436ec0cf1a9859aa35b76d125` predates this
descriptor and remains compatible at 50 rows/10 updates only; unknown binaries
require exporter metadata, even at 50/10. Alternate chunk/step support is tested
only with fake sessions and tiny ONNX schedule graphs; qualify a real alternate
export separately before deployment.

### GR00T N1.7 bundle

`graph_abi: groot.n17.six_graphs.euler.v1` chains `vision` (once per camera),
`tokens`, `backbone_0_8`, `backbone_8_16`, `observation_features` and
`action_step`. The manifest carries the shared identity, camera, timing,
`graphs`, `device_placement` and `precision` (`fp16`, `w8a16` or `w4a16` per
graph, with `action_step` required to be `fp16`) fields, plus `prompt`,
`max_state_dim`, `max_action_dim`, `chunk_size`, `denoising_steps`,
`image_transform` (the checkpoint's default N1.7 transform),
`normalization: MIN_MAX`, `action_mapping: {mode: trained_min_max}`, `stats` and
`static_inputs`.

The export fixes the prompt, so `/infer` refuses any other task with HTTP 400:
set the Objective's `prompt` to the bundle's. `static_inputs` is an `.npz` of the
prompt's `token_ids`, `image_mask`, `valid_mask`, `attention_bias`, `rope_cos` and
`rope_sin`; its token count and image-token count must agree with the graphs.
The statistics JSON holds `observation.state.min`/`max` (trained width),
`action.min`/`max` per chunk step (`[chunk_size, model_action_dim]`, absolute
columns repeated) and `action.relative`, one flag per action column. Host
preprocessing is LeRobot's: its N1.7 image transform, Qwen3-VL patching and
clipped min-max state normalization. The host integrates each `action_step`
velocity over the bucketed flow schedule and decodes relative columns against
the request state.

The `action_step` manifest carries `sampling`, emitted at staging time from the
GR00T model config the step was exported from and the LeRobot checkpoint's
config, which supplies the step count and ramp overrides and the chunk caps.
Both are required:

```bash
python vla_qnn_export.py --groot-model-config model_config.json \
  --groot-policy-config config.json --context model.bin --output sampling.json
```

RTC follows the PyTorch GR00T path: for a checkpoint with relative action
columns, `action_chunk_raw` is the absolute chunk and the server re-anchors the
carryover to the new state; frozen rows stay fixed and the rest ramp in at
`groot_rtc_ramp_rate` (0 uses the exported rate).

The action step must be compiled at FP16, and the server refuses a bundle whose
`action_step` precision is anything else: a W8A16 action step drifts far enough
from the checkpoint to trip ExecutePolicy's joint limits. A single FP16
action-step context (about 2.2 GB) loads on the IQ10 but not on the IQ9, which
refuses it with QNN status 1002 (`QNN_COMMON_ERROR_MEM_ALLOC`) on either HTP
device, so an IQ9 GR00T bundle needs the FP16 action step split across two
contexts (a follow-up). `model_action_dim` must equal `request_state_dim`.

The action step is exported at the model's full `action_horizon`, so the exporter
refuses a checkpoint whose LeRobot `chunk_size` or `n_action_steps` is below
`action_horizon`. The sidecar records `action_horizon` and those `policy_caps`,
and the server refuses a sidecar whose recorded values break the same rule. The
server also refuses a sidecar exported without the LeRobot config, whose caps
it cannot verify; re-stage such a sidecar with `--groot-policy-config`.

Because the host decodes relative action columns against the state column at
the same position, the server refuses a `vla_serving.yaml` whose `action_names`
name a relative column differently from `state_names` at that position; list
both in the same joint order.

### Serve on the IQ10 NPU

On a Qualcomm Dragonwing IQ10 running Ubuntu 24.04, `moveit_pro run
--with-inference-server` serves the GR00T N1.7 Kinova cube-stacking fine-tune on
the Hexagon v81 NPU, with no image or compose edits, after this one-time board
setup:

1. Add `VLA_QNN_HOST: iq10` to `~/.config/moveit_pro/moveit_pro_config.9.yaml`
   (keep `MOVEIT_TARGET: ''`: the IQ10 has no `/dev/fastrpc-cdsp*`, so the
   CLI's generic `-qnn` Runtime target does not apply to it).
2. Put the compiled bundle in
   `src/vla_sim/models/groot_n17_kinova_cube_stack_20k_iq10_qnn/` (the
   `qnn_bundle` that `config/vla_serving_iq10.yaml` names).
3. If the active robot configuration package (`MOVEIT_CONFIG_PACKAGE`) is not
   `vla_sim`, also set `VLA_CONFIG_DIR` to the directory holding
   `vla_serving_iq10.yaml`, for example
   `/home/<user>/moveit_pro/moveit_pro_example_ws/src/vla_sim/config`. The
   worker mounts `src/<MOVEIT_CONFIG_PACKAGE>/config` otherwise, and refuses to
   start without the file.

`VLA_QNN_HOST` changes three things in the workspace `docker-compose.yaml`:

- The inference image is built from the `runtime-qnn` stage and tagged
  `moveit_pro-inference_server-qnn`: Ubuntu 24.04 with CPU torch and the
  QAIRT 2.50 runtime libraries from the hash-pinned `onnxruntime-qnn` 2.6.0
  wheel, whose V81 skeleton FastRPC finds through `ADSP_LIBRARY_PATH`.
- `inference_qnn_worker` joins the `inference` profile and starts as a
  dependency of `inference_server`. It runs this server with `--unix-socket` and
  `--config /vla_config/vla_serving_<VLA_QNN_HOST>.yaml`; a missing file is an
  error naming its path, not a fallback to defaults. `--no-drivers` starts
  `inference_server` with `up --no-deps`, which skips the worker, so run
  without it.
- `inference_server` relays its loopback port to that socket
  (`vla_inference_forwarder.py`) instead of loading a model.

The split exists because the IQ10 BSP's FastRPC serves only root clients:
`fastrpc-rm` creates its per-session POSIX queues `0644 root`, and the client
must `pidfd_getfd` buffers out of the root daemon. The worker therefore runs as
root with the host IPC and PID namespaces (the queue names embed the client's
host PID) and `CAP_SYS_PTRACE`, which the launcher refuses for the
network-listening `inference_server`. The worker has `network_mode: none`, drops
every other capability, sets `no-new-privileges`, runs with a read-only root
filesystem and a process limit, and mounts the board's `libcdsprpc.so`,
`libkiumd.so`, `liblrmc.so` and `/vendor/dsp` (the DSP shell) read-only. No
`--privileged` and no device nodes are needed. `inference_server` keeps every
hardening the launcher checks and stays the only listener; the worker's socket
lives in the `vla_qnn_ipc` volume that only the two services mount. The relay
checks the bearer token before it reads a POST body, and the worker checks it
again.

Root with the host PID namespace and `CAP_SYS_PTRACE` amounts to host root (the
worker can read `/proc/1/root`). That is the price of this BSP's FastRPC
contract, and it is why the worker takes no input but the bearer-checked HTTP
API relayed from the loopback-only port. Enable `VLA_QNN_HOST` only on a board
whose local users you trust, until the BSP offers a supported non-root FastRPC
route.

`moveit_pro run` stops only the services it names, so after Ctrl-C the worker
outlives its relay: it exits by itself 60 s after the relay's last poll,
releasing the NPU. `moveit_pro down` removes it at once. A worker parked after
a native QNN failure restarts itself when the next relay connects, so rerunning
`moveit_pro run --with-inference-server` recovers it either way.

MoveIt Pro Trainer's **Configure for ExecutePolicy** writes `vla_serving.yaml`,
which the IQ10 worker does not read: it serves compiled bundles, not PyTorch
checkpoints. Compile a Trainer checkpoint into a bundle and point
`vla_serving_iq10.yaml` at it instead.

`/health` reports `loading` until the worker answers and then relays the
worker's own state, for example `{"status": "ready", "device": "npu:qnn_htp",
"qnnPrecision": {...}}`. Debug the native side with `docker compose logs
inference_qnn_worker` (`docker logs moveit_pro-inference_qnn_worker-1`). If
`journalctl -u fastrpc-rm` shows `Glink RM ... -11` or `CB exhausted`, the DSP
is wedged and even host root fails: reboot the board.

### Native runtime requirements

The standard inference image intentionally does **not** install QAIRT. Use a
board environment with QAIRT 2.49 (QNN core API 2.38, IQ9) or 2.50 (core API
2.39, IQ10), contexts compiled for that board's Hexagon version and the
HTP/FastRPC devices accessible to the inference process. The IQ10's NSP reports
four cores; the backend exposes only the first, because its contexts are
compiled single-core. On the IQ10, export the QAIRT package's FastRPC search
paths (`ADSP_LIBRARY_PATH` and `CDSP*_LIBRARY_PATH`, as its `set_env.sh` does)
or the DSP cannot open the V81 skeleton and device creation fails with status
14001; run the server from the board's Ubuntu 24.04 userland, because FastRPC
from a Debian-based image stalls every native call.
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

Hardware/library provisioning is a host deployment requirement. On the IQ10,
`VLA_QNN_HOST` provides all of it (see "Serve on the IQ10 NPU" above). On any
other host the shipped `inference_server` service does not mount QAIRT, the HTP
devices or a compiled bundle, so `backend: qnn` through the launcher parks in
`error`. Do not expose a plain HTTP listener over the network; use the existing
remote TLS proxy and inference-only bearer key.

There is no CPU/GPU neural fallback. A QNN load, warmup, native execution,
cleanup or process-replacement failure parks the server in `error` until an
**explicit inference-server restart**; a config rewrite alone does not recover
it. Correct the inputs (or set `backend: pytorch`), then restart the
process/container. A healthy server reloads on serving-YAML replacement once
idle (waiting up to 120 seconds for a QNN load/warmup) and frees native sessions
first; if a hung call still holds them after 1 second, process exit releases
them. Native cleanup is attempted once, never retried. PyTorch-backend retries
are unchanged.

QNN keeps the same `/infer` RTC fields: `prev_chunk_left_over` is the unexecuted
normalized `action_chunk_raw` tail, `inference_delay` the frozen prefix length
and `guidance_horizon` the soft width beyond it (0 uses the configured default).
Responses carry physical `action_chunk`, normalized trained-width
`action_chunk_raw` (absolute for a GR00T checkpoint with relative columns, see
above) and the bundle's trained `dt`. For pi0.5, after each native,
already-integrated expert update, stock LeRobot RTC guidance adds only its
residual; without carryover the native output and seeded noise sequence are
unchanged, and warmup preserves that noise stream. Malformed carryover, an image
that scales to a zero extent, and a host RTC correction that overflows Float32
are refused with HTTP 400 without poisoning the runner. No clipping or magnitude
limit is applied, and native dynamic range under large carryover is unqualified:
nonfinite native outputs or execution failures remain fatal and require the
explicit restart.

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
| `VLA_QNN_HOST` | Set on a Qualcomm board whose FastRPC serves only root clients (`iq10`): builds the `-qnn` image, starts `inference_qnn_worker`, and serves `vla_serving_<value>.yaml`. Set it in the board's `moveit_pro_config.9.yaml`, never in a committed `.env`. See "Serve on the IQ10 NPU". |
| `VLA_QNN_CDSPRPC_LIBRARY`, `VLA_QNN_KIUMD_LIBRARY`, `VLA_QNN_LRMC_LIBRARY`, `VLA_QNN_DSP_DIR` | Host paths of the FastRPC client libraries and DSP partition the worker mounts. Default to the IQ10 Ubuntu layout (`/usr/lib/libcdsprpc.so`, `/lib/libkiumd.so`, `/lib/liblrmc.so`, `/vendor/dsp`). |

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

Use `moveit_pro-inference_server-qnn:latest` on a QNN host. The relay's own
tests need only the standard library and also run in colcon
(`test_vla_inference_forwarder`); add them to the command above with
`test_vla_inference_forwarder`.

QNN tests use fake native sessions and need neither QAIRT, NPU hardware, a Hub
connection nor checkpoint weights. `test_vla_qnn_export` additionally needs
ONNX, and `test_vla_qnn_availability` needs pytest. Provision both into a
temporary test-only directory, not the serving image: the first command below
constrains them to the image's installed versions, so a conflict fails
provisioning instead of shadowing packages such as numpy; the second runs all
four suites offline with the source mounted read-only:

```bash
test_deps="$(mktemp -d)"
docker run --rm --entrypoint sh --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -v "$test_deps:/test-deps" \
  moveit_pro-inference_server:latest -c \
  'python -m pip freeze > /tmp/image-constraints.txt &&
   python -m pip install --target /test-deps \
     -c /tmp/image-constraints.txt "pytest==9.0.3" "onnx==1.19.1"'

docker run --rm --network none --entrypoint python \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e USER=vla \
  -e PYTHONDONTWRITEBYTECODE=1 -e OMP_NUM_THREADS=1 \
  -e PYTHONPATH=/test-deps -v "$test_deps:/test-deps:ro" \
  -v "$PWD/src/vla_sim/docker:/app:ro" -w /app \
  moveit_pro-inference_server:latest \
  -m pytest -v -p no:cacheprovider test_vla_inference_server.py \
  test_vla_qnn_backend.py test_vla_qnn_export.py test_vla_qnn_availability.py
```
