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

"""Every runtime file reference must resolve from the INSTALLED tree, not just the source tree.

A reference that reaches outside an installed directory works in the source tree and fails at
hardware init with "XML Error: Error opening file ...", which is a long way from the cause. This
resolves each reference against the installed share directory the way MuJoCo and the resource
retriever do: includes and `file=` relative to the file that names them, and `package://` through
the package share.
"""

import re
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

SHARE = Path(get_package_share_directory("market_sim"))
MJCF = SHARE / "mjcf"
# MuJoCo resolves an <include> against the including file, but <mesh>/<texture>/<hfield>
# file= against the TOP LEVEL model's directory, with <compiler meshdir> prepended when set.
INCLUDE = re.compile(r'<include file="([^"]+)"')
ASSET = re.compile(r'(?<!include )file="([^"]+)"')
PACKAGE_URI = re.compile(r'filename="(package://[^"]+)"')
MODEL_ROOT = MJCF
MESHDIR = "assets"


def referenced_files():
    """Yield (naming file, raw reference, kind) for every file reference in the package."""
    for root in (MJCF, SHARE / "description"):
        for path in sorted(root.rglob("*")):
            if path.suffix not in (".xml", ".xacro", ".urdf") or not path.is_file():
                continue
            text = path.read_text(errors="ignore")
            for raw in INCLUDE.findall(text):
                yield path, raw, "include"
            for raw in ASSET.findall(text):
                yield path, raw, "asset"
            for raw in PACKAGE_URI.findall(text):
                yield path, raw, "package"


def resolve(naming_file, raw, kind):
    """Where this reference should land in the install tree, or None if it is not ours."""
    if raw.startswith("package://"):
        package, _, rest = raw[len("package://") :].partition("/")
        if package != "market_sim":
            return None  # another package's share
        return SHARE / rest
    if raw.startswith("$(") or raw.startswith("/"):
        return None  # a xacro substitution or an absolute system path
    if kind == "include":
        return (naming_file.parent / raw).resolve()
    # An asset: top-level model directory, with meshdir for meshes.
    direct = (MODEL_ROOT / raw).resolve()
    if direct.exists():
        return direct
    return (MODEL_ROOT / MESHDIR / raw).resolve()


def test_every_runtime_reference_resolves_in_the_install_tree():
    missing = []
    checked = 0
    for naming_file, raw, kind in referenced_files():
        target = resolve(naming_file, raw, kind)
        if target is None:
            continue
        checked += 1
        if not target.exists():
            missing.append(f"{naming_file.relative_to(SHARE)} -> {raw}")
    assert checked, "no references found; the install tree is probably empty"
    assert (
        not missing
    ), "references that do not resolve in the install tree:\n" + "\n".join(missing)


def test_no_reference_escapes_the_installed_share_directory():
    """A `../` that climbs out of share/market_sim cannot be installed and must not appear."""
    escaping = []
    for naming_file, raw, kind in referenced_files():
        target = resolve(naming_file, raw, kind)
        if target is None:
            continue
        if not str(target).startswith(str(SHARE)):
            escaping.append(f"{naming_file.relative_to(SHARE)} -> {raw} ({target})")
    assert (
        not escaping
    ), "references outside the installed share directory:\n" + "\n".join(escaping)
