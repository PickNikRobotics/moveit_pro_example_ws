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
"""SmolVLA wrapper for batched sim rollouts: LeRobot's own policy and processors (so a saved
checkpoint loads unchanged in vla_sim's inference server), caller-supplied flow noise for DSRL, and
the pooled prefix embedding the sampler already computes, exposed as the RL observation feature.
The sampler is split into encode (prefix + KV cache) and decode (denoise from given noise) so the
RL actor can pick the noise after seeing the observation."""

import os
from pathlib import Path

# gfx1151 (Strix Halo): AOTriton flash attention is opt-in on RDNA (3.2x on SmolVLA's ViT), and
# MIOpen fails on the patch-embedding conv, so convolutions use PyTorch's native kernels.
os.environ.setdefault("TORCH_ROCM_AOTRITON_ENABLE_EXPERIMENTAL", "1")

import numpy as np
import torch

torch.backends.cudnn.enabled = False
from lerobot.configs.policies import PreTrainedConfig
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy, make_att_2d_masks
from lerobot.utils.constants import OBS_LANGUAGE_ATTENTION_MASK, OBS_LANGUAGE_TOKENS

STATE_DIM = ACTION_DIM = 8


def image_key(name: str) -> str:
    return f"observation.images.{name}"


class SmolVLA:
    def __init__(
        self,
        checkpoint: str | Path,
        image_names: list[str],
        image_size: int,
        device: str = "cuda",
        stats: dict | None = None,
    ):
        """checkpoint: smolvla_base (new features, needs stats) or a checkpoint this module saved
        (features and processors come with it)."""
        checkpoint = str(checkpoint)
        cfg = PreTrainedConfig.from_pretrained(checkpoint)
        fresh = stats is not None
        if fresh:
            cfg.input_features = {
                "observation.state": PolicyFeature(FeatureType.STATE, (STATE_DIM,))
            }
            for n in image_names:
                cfg.input_features[image_key(n)] = PolicyFeature(
                    FeatureType.VISUAL, (3, image_size, image_size)
                )
            cfg.output_features = {
                "action": PolicyFeature(FeatureType.ACTION, (ACTION_DIM,))
            }
        cfg.load_vlm_weights = (
            False  # the SmolVLA checkpoint already carries the VLM weights it uses
        )
        cfg.device = device
        self.cfg = cfg
        self.policy = SmolVLAPolicy.from_pretrained(checkpoint, config=cfg).to(device)
        if fresh:
            self.pre, self.post = make_pre_post_processors(cfg, dataset_stats=stats)
        else:
            self.pre, self.post = make_pre_post_processors(
                cfg, pretrained_path=checkpoint
            )
        self.image_names, self.device = image_names, device
        self.chunk, self.max_action_dim = cfg.chunk_size, cfg.max_action_dim

    @property
    def feature_dim(self) -> int:
        return self.policy.model.vlm_with_expert.config.text_config.hidden_size

    def batch(
        self,
        images: np.ndarray,
        state: np.ndarray,
        prompt: str,
        actions=None,
        action_is_pad=None,
    ) -> dict:
        """images (B, cams, H, W, 3) uint8 in image_names order; state (B, 8)."""
        b = {
            "observation.state": torch.as_tensor(state, dtype=torch.float32),
            "task": [prompt] * len(state),
        }
        img = torch.as_tensor(images).permute(0, 1, 4, 2, 3).float().div_(255.0)
        for k, n in enumerate(self.image_names):
            b[image_key(n)] = img[:, k]
        if actions is not None:
            b["action"] = torch.as_tensor(actions, dtype=torch.float32)
            b["action_is_pad"] = torch.as_tensor(action_is_pad)
        return self.pre(b)

    @torch.no_grad()
    def encode(self, images, state, prompt) -> dict:
        """Prefix pass (vision, language, state) and KV cache; the pooled contextualized prefix is
        the observation feature the DSRL actor and critic see. Mirrors VLAFlowMatching.sample_actions.
        """
        pol, m = self.policy, self.policy.model
        pol.eval()
        proc = self.batch(images, state, prompt)
        imgs, img_masks = pol.prepare_images(proc)
        st = pol.prepare_state(proc)
        with torch.autocast("cuda", dtype=torch.bfloat16):
            embs, pad, att = m.embed_prefix(
                imgs,
                img_masks,
                proc[OBS_LANGUAGE_TOKENS],
                proc[OBS_LANGUAGE_ATTENTION_MASK],
                state=st,
            )
            outs, kv = m.vlm_with_expert.forward(
                attention_mask=make_att_2d_masks(pad, att),
                position_ids=torch.cumsum(pad, dim=1) - 1,
                past_key_values=None,
                inputs_embeds=[embs, None],
                use_cache=m.config.use_cache,
                fill_kv_cache=True,
            )
        w = pad.unsqueeze(-1).float()
        pooled = (outs[0].float() * w).sum(1) / w.sum(1).clamp_min(1)
        return {"pad": pad, "kv": kv, "feat": pooled, "b": len(state)}

    @torch.no_grad()
    def decode(self, ctx: dict, noise: torch.Tensor | None = None) -> np.ndarray:
        """Flow-matching denoise from the given initial noise (B, chunk, max_action_dim); returns
        (B, chunk, 8) unnormalized absolute joint targets."""
        m = self.policy.model
        b = ctx["b"]
        x = (
            m.sample_noise(self.noise_shape(b), self.device)
            if noise is None
            else noise.to(self.device, torch.float32)
        )
        dt = -1.0 / m.config.num_steps
        with torch.autocast("cuda", dtype=torch.bfloat16):
            for k in range(m.config.num_steps):
                t = torch.full(
                    (b,), 1.0 + k * dt, dtype=torch.float32, device=self.device
                )
                x = x + dt * m.denoise_step(
                    prefix_pad_masks=ctx["pad"],
                    past_key_values=ctx["kv"],
                    x_t=x,
                    timestep=t,
                )
        act = self.post(x[:, :, :ACTION_DIM].float())
        return act.float().cpu().numpy()

    def sample(self, images, state, prompt, noise=None):
        ctx = self.encode(images, state, prompt)
        return self.decode(ctx, noise), ctx["feat"]

    def noise_shape(self, b: int) -> tuple[int, int, int]:
        return (b, self.chunk, self.max_action_dim)

    def save(self, out: Path) -> None:
        out.mkdir(parents=True, exist_ok=True)
        self.policy.save_pretrained(out)
        self.pre.save_pretrained(out)
        self.post.save_pretrained(out)


def stats_from(states: np.ndarray, actions: np.ndarray) -> dict:
    def ms(x):
        x = x.reshape(-1, x.shape[-1])
        return {"mean": torch.tensor(x.mean(0)), "std": torch.tensor(x.std(0) + 1e-6)}

    return {"observation.state": ms(states), "action": ms(actions)}
