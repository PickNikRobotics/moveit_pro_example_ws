# M19: why the October SmolVLA base failed (vla_sim cube stacking)

Environment and commands: [README.md](README.md). Data and checkpoints: `~/.cache/mjbatch_rl/exp19/`.

**Data:** `scripted.py --n 1024 --seed 100`, 869 successful demos (mean success step 96.6 of 120).

**Training:** `sft.py`, batch 32, lr 1e-4 cosine over 20k steps, stopped at 11,175. Resumed from step 10,000 for 10k more at lr 4e-5 (`sft_869_r10k`).

**Evaluation:** 192 trials, seed 12345, MuJoCo 3.14. "c20" means 20 of 50 chunk steps executed (the Objective's setting); "c10" means 10.

| Checkpoint | Training samples | Open-loop arm error | 16-trial diagnostic (reach < 2 cm / lift > 10 cm) | c20 | c10 | c20, 180-step episodes |
|---|---|---|---|---|---|---|
| October `sft_217` | 48k (217 demos) | not captured | 7 within 2.5 cm / 2 | 0/192 | 1/192 | — |
| step 5,000 | 160k | 0.046 rad | 1 / 3 | 2/192 (1.0%) | 2/192 | — |
| step 10,000 | 320k | 0.022 rad | 12 / 10 | 26/192 (13.5% ± 4.8) | 22/192 (11.5%) | 34/192 (17.7% ± 5.4) |
| step 15,000 (resumed at 10k, lr 4e-5) | 480k | 0.020 rad | 15 within 2.1 cm / 12; 7 of 16 stacked | 65/192 (33.9% ± 6.7) | — | 79/192 (41.1% ± 7.0) |
| step 20,000 (resumed at 15k, lr 2e-5) | 640k | 0.018 rad | 16 within 2.5 cm / 12; 5 of 16 stacked | 86/192 (44.8% ± 7.0) | — | 90/192 (46.9% ± 7.1) |

At step 20,000 on the Runtime's MuJoCo 3.6.0 (`plain36`), 120-step episodes: 83/192 (43.2% ± 7.0). The two engines agree within noise.

**Findings:**
- The October base was undertrained. Success rose from about 1% to 13.5% between 5k and 10k steps, as the open-loop error halved.
- Executing 10 steps per chunk instead of 20 does not help.
- A 120-step episode cuts off some slow successes: 8 of 192 more trials succeed with 180 steps. The oracle itself averages 97 steps.

**Base for RL:** `sft_869_r15k/step_5000` (20,000 total steps).

**Run hygiene:** a second GPU process (an evaluation during training) caused a GPU memory access fault on gfx1151. Run one GPU job at a time.
