#!/bin/sh
# Install the optional Nyx color-camera renderer when the image build sets INSTALL_NYX=true.
set -eu
[ "${INSTALL_NYX:-false}" = true ] || exit 0
if [ -f /opt/nyx_shim/lib/liboidn_device_shim.so ]; then
  exec /opt/nyx_shim/install_runtime.sh
fi
pip install --break-system-packages --only-binary=:all: --index-url https://download.pytorch.org/whl/cu130 torch==2.10.0+cu130
pip install --break-system-packages --only-binary=:all: gs_nyx==0.1.3
pip check
