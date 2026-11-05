# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Image-build and runtime dependencies the package declares. No ROS."""

import os
import subprocess
from pathlib import Path

import pytest
import yaml
from catkin_pkg.package import parse_package

PACKAGE = Path(__file__).resolve().parents[1]
INSTALL_NYX = PACKAGE / "docker/install_nyx.sh"


def run_install_nyx(tmp_path: Path, install_nyx: str | None) -> list[str]:
    """Run the installer with a recording pip and return the pip calls."""
    log = tmp_path / "pip.log"
    pip = tmp_path / "pip"
    pip.write_text(f'#!/bin/sh\necho "$*" >> {log}\n')
    pip.chmod(0o755)
    env = {**os.environ, "PATH": f"{tmp_path}:{os.environ['PATH']}"}
    env.pop("INSTALL_NYX", None)
    if install_nyx is not None:
        env["INSTALL_NYX"] = install_nyx
    subprocess.run(["sh", str(INSTALL_NYX)], env=env, check=True)
    return log.read_text().splitlines() if log.exists() else []


@pytest.mark.parametrize("install_nyx", [None, "", "false", "1"])
def test_nyx_install_is_off_unless_opted_in(tmp_path: Path, install_nyx) -> None:
    """Ordinary builds, NVIDIA or not, download nothing for Nyx."""
    assert run_install_nyx(tmp_path, install_nyx) == []


@pytest.mark.skipif(
    Path("/opt/nyx_shim/lib/liboidn_device_shim.so").exists(),
    reason="the base image's Nyx shim installer takes over",
)
def test_nyx_install_when_opted_in(tmp_path: Path) -> None:
    """INSTALL_NYX=true installs Torch, then Nyx, then checks the environment."""
    calls = run_install_nyx(tmp_path, "true")
    assert [call.split()[0] for call in calls] == ["install", "install", "check"]
    assert "torch==2.10.0+cu130" in calls[0].split()
    assert "gs_nyx==0.1.3" in calls[1].split()


def test_every_configured_planner_is_declared() -> None:
    """planner_server cannot configure if a planner plugin's package is missing."""
    params = yaml.safe_load((PACKAGE / "params/nav2_params.yaml").read_text())
    planner = params["planner_server"]["ros__parameters"]
    needed = {planner[p]["plugin"].split("::")[0] for p in planner["planner_plugins"]}
    declared = {d.name for d in parse_package(str(PACKAGE)).exec_depends}
    assert needed <= declared, needed - declared
