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

"""Tests that runtime.launch.xml parses and types the adapter's http_timeout as float."""

from pathlib import Path

from launch.frontend import Parser
from launch_ros.actions import Node

LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "runtime.launch.xml"


def test_http_timeout_param_has_float_type() -> None:
    """Parse the file the way `ros2 launch` does, which rejects an unknown type identifier,
    then check that an integer-valued MOVEIT_INFERENCE_HTTP_TIMEOUT reaches the node as the
    double it declares."""
    root, parser = Parser.load(str(LAUNCH_FILE))
    parser.parse_description(root)
    (adapter,) = [e for e in root.children if e.type_name == "node"]
    timeout = [
        p
        for p in adapter.children
        if p.type_name == "param" and p.get_attr("name") == "http_timeout"
    ]
    (entry,) = Node.parse_nested_parameters(timeout, parser)
    (value,) = entry.values()
    assert value.value_type is float
