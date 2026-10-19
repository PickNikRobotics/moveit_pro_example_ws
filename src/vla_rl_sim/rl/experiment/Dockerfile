# SmolVLA base-policy experiment (vla_rl_sim prototype) on gfx1151: the Nyx image's ROCm 7.2.2 torch 2.10,
# plus the matching ROCm torchvision, LeRobot 0.6.0 with SmolVLA, and mjbatch 0.1.4 (MuJoCo 3.14).
FROM sim-renderer-strix-nyx:gs0.1.3-rocm7.2.2
RUN pip install --break-system-packages --no-cache-dir --no-deps \
      "https://repo.radeon.com/rocm/manylinux/rocm-rel-7.2.2/torchvision-0.25.0%2Brocm7.2.2.git82df5f59-cp312-cp312-linux_x86_64.whl" \
 && pip install --break-system-packages --no-cache-dir "lerobot[smolvla]==0.6.0" "mjbatch==0.1.4" \
 && python3 -c "import torch, torchvision, lerobot, mjbatch, mujoco; assert torch.version.hip; print(torch.__version__, torchvision.__version__, mujoco.__version__)"
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MUJOCO_EGL_DEVICE_ID=0 TORCH_ROCM_AOTRITON_ENABLE_EXPERIMENTAL=1 HF_HOME=/cache/hf
