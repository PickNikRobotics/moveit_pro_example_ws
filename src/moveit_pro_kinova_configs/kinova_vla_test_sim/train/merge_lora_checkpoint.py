#!/usr/bin/env python
"""Merge a lerobot pi0.5 LoRA adapter into a dense checkpoint.

`lerobot-train` saves adapter-only checkpoints, which the eval/serve loaders can't
read directly. This loads the base policy, applies the adapter, merges the LoRA
deltas into the base weights, and saves a dense checkpoint with the trained
processors copied alongside.

Usage:
  merge_lora_checkpoint.py IN_ADAPTER_DIR OUT_DENSE_DIR
"""

import json
import shutil
import sys
from pathlib import Path

import torch
from lerobot.policies.pi05.modeling_pi05 import PI05Policy
from lerobot.processor import ProcessorStepRegistry
from lerobot.processor.relative_action_processor import RelativeActionsProcessorStep

try:
    ProcessorStepRegistry.get("relative_actions_processor")
except Exception:
    ProcessorStepRegistry.register("relative_actions_processor")(
        RelativeActionsProcessorStep
    )

from peft import PeftConfig, PeftModel  # noqa: E402


def main() -> None:
    in_dir, out_dir = sys.argv[1], sys.argv[2]
    in_p, out_p = Path(in_dir), Path(out_dir)

    peft_cfg = PeftConfig.from_pretrained(in_dir)
    base_ref = peft_cfg.base_model_name_or_path or "lerobot/pi05_base"
    print(f"[merge] base={base_ref}  adapter={in_dir}")

    base = PI05Policy.from_pretrained(base_ref)
    model = PeftModel.from_pretrained(base, in_dir)
    merged = model.merge_and_unload().to(torch.bfloat16)  # PEFT merges in float32

    out_p.mkdir(parents=True, exist_ok=True)
    merged.save_pretrained(out_p)

    # save_pretrained writes the base policy's config; overwrite it with the trained
    # config so the merged checkpoint keeps the right action shape/dtype and no adapter flag.
    trained_cfg = json.loads((in_p / "config.json").read_text())
    trained_cfg["use_peft"] = False
    trained_cfg["dtype"] = "bfloat16"
    (out_p / "config.json").write_text(json.dumps(trained_cfg, indent=2))

    copied = []
    for f in in_p.iterdir():
        if (
            f.name.startswith("policy_pre")
            or f.name.startswith("policy_post")
            or f.name == "train_config.json"
        ):
            shutil.copy2(f, out_p / f.name)
            copied.append(f.name)

    print(f"[merge] copied processors: {sorted(copied)}")
    print(f"[merge] dense checkpoint -> {out_p}")
    print(f"[merge] contents: {sorted(p.name for p in out_p.iterdir())}")


if __name__ == "__main__":
    main()
