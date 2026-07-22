# Policy serving

Runs a trained policy against the `Execute Color-Stack Policy` objective via MoveIt Pro's `ExecutePolicy` behavior and the `/get_action_chunk` service.

## 1. Serve (model container)

```bash
docker compose run --rm serve_policy \
  --checkpoint <path-to-merged-checkpoint> --policy-class pi05
```

If `SERVE_POLICY_PORT` is set, also point the bridge's `INFER_URL` at
the same port (it defaults to `http://172.17.0.1:8973/infer`).

## 2. Run (MoveIt Pro UI)

The adapter starts automatically with Pro. To run manually without the whole stack:

```bash
ros2 run kinova_vla_test_sim get_action_chunk_adapter.py
```

## Tests

`../test/test_get_action_chunk_adapter.py` runs under `colcon test`. `test_serve_policy.py` needs `lerobot`/`torch`, so run it in model container:

```bash
env -u LD_LIBRARY_PATH <python-with-lerobot> -m pip install pytest
env -u LD_LIBRARY_PATH <python-with-lerobot> -m pytest test_serve_policy.py
```
