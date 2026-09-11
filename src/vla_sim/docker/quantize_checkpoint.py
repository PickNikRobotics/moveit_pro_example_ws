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

"""Write a pi0.5 checkpoint back out at the size the server runs it at.

`int8: true` in vla_serving.yaml reaches the same weights by quantizing on every
load, from a full-width checkpoint that has to be downloaded and held first.
This writes that result once, as a checkpoint of its own that the server then
loads as it finds it.

Runs on the cpu, in the inference server's image but not in the serving
container, which mounts /models read-only. README.md carries the invocation.
"""

import argparse
import sys
from pathlib import Path

from lerobot.configs.policies import PreTrainedConfig

from vla_inference_server import (
    MODEL_WEIGHTS_FILE,
    load_full_policy,
    log,
    resolve_policy_type,
    save_quantized_checkpoint,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        required=True,
        help="the pi0.5 checkpoint to quantize: a local directory or an HF repo id",
    )
    parser.add_argument(
        "--out",
        required=True,
        help="directory to write the quantized checkpoint into",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    out = Path(args.out)
    if out.exists():
        log(f"ERROR: {out} already exists; move it aside first")
        return 1

    policy_type = resolve_policy_type(args.checkpoint, "")
    if policy_type != "pi05":
        log(
            f"ERROR: quantization covers pi0.5's language backbone and vision "
            f"tower by name, and '{args.checkpoint}' is a {policy_type} checkpoint"
        )
        return 1

    # The cpu holds the full-width weights for exactly as long as it takes to
    # halve them; nothing here needs a gpu, and the gpu it would need is the one
    # this is trying to fit into.
    log(f"loading '{args.checkpoint}' on the cpu and quantizing it ...")
    policy_config = PreTrainedConfig.from_pretrained(args.checkpoint)
    policy_config.device = "cpu"
    try:
        policy = load_full_policy(
            args.checkpoint, policy_type, policy_config, int8=True
        )
    except ValueError as error:
        log(f"ERROR: {error}")
        return 1

    log(f"writing '{out}' ...")
    copied = save_quantized_checkpoint(policy, args.checkpoint, str(out))
    written = (out / MODEL_WEIGHTS_FILE).stat().st_size / 1024**3
    log(
        f"wrote {MODEL_WEIGHTS_FILE} ({written:.2f} GiB) and {len(copied)} other file(s)"
    )
    log(f"serve it by setting the vla_serving.yaml checkpoint to '{out}'")
    return 0


if __name__ == "__main__":
    sys.exit(main())
