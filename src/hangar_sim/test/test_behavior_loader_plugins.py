#!/usr/bin/env python3

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

"""hangar_sim must load only the Behavior loaders its own config chain declares.

MoveIt Pro's config loader merges every `behavior_plugin.yaml` under the
workspace's `src/` into every config, not only the config of the package that
ships it. CI builds `--packages-up-to hangar_sim`, so a loader added that way
is never built, and the Objective server aborts at start-up.

Loads hangar_sim through `load_system_config`, the loader `moveit_pro run`
uses, once with this workspace and once with an empty one. No simulator.
"""

from pathlib import Path

from moveit_studio_utils_py.system_config import load_system_config

# test/ -> hangar_sim/ -> src/ -> the workspace root.
WORKSPACE = Path(__file__).resolve().parents[3]


def behavior_loader_plugins(user_ws: Path) -> set[str]:
    objectives = load_system_config("hangar_sim", user_ws)[0].objectives
    return {
        plugin
        for plugins in objectives.behavior_loader_plugins.values()
        for plugin in plugins
    }


def test_workspace_adds_no_behavior_loader_plugins(tmp_path):
    assert behavior_loader_plugins(WORKSPACE) == behavior_loader_plugins(tmp_path)
