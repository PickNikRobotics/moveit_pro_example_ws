#!/usr/bin/env python3
"""Generate a tileable dirt-ground texture for grinding_sim's outdoor floor.

Deterministic (fixed seed) and dependency-light, so it can be regenerated and reviewed.

Uses tileable value noise rather than summed sines. Sines were the first attempt and they
banded badly: integer frequencies in x and y put every octave's extrema on the same rows
and columns, which renders as regular stripes across the ground at grazing angles. Value
noise on a wrapping integer lattice has no preferred direction and still tiles exactly,
because the lattice itself wraps.
"""
import numpy as np
from PIL import Image

SIZE = 1024
BASE = np.array([0.33, 0.25, 0.18])   # damp earth
DRY  = np.array([0.55, 0.45, 0.34])   # sun-dried patches
rng = np.random.default_rng(11)


def tileable_value_noise(size, cells):
    """Value noise on a x wrapping lattice, smoothstep-interpolated."""
    lattice = rng.random((cells, cells))
    t = np.arange(size) / size * cells
    i0 = np.floor(t).astype(int) % cells
    i1 = (i0 + 1) % cells
    f = t - np.floor(t)
    f = f * f * (3.0 - 2.0 * f)                      # smoothstep
    a = lattice[np.ix_(i0, i0)]; b = lattice[np.ix_(i1, i0)]
    c = lattice[np.ix_(i0, i1)]; d = lattice[np.ix_(i1, i1)]
    fx = f[:, None]; fy = f[None, :]
    return (a * (1 - fx) * (1 - fy) + b * fx * (1 - fy)
            + c * (1 - fx) * fy + d * fx * fy)


n = np.zeros((SIZE, SIZE)); amp = 1.0; total = 0.0
for cells in (4, 8, 16, 32, 64, 128):
    n += amp * tileable_value_noise(SIZE, cells)
    total += amp; amp *= 0.55
n /= total
n = (n - n.min()) / (np.ptp(n) + 1e-9)

img = BASE[None, None, :] + (DRY - BASE)[None, None, :] * n[..., None] ** 1.3

# Fine grit so it reads as soil rather than a smooth gradient.
grit = rng.random((SIZE, SIZE))
img *= (0.90 + 0.20 * grit)[..., None]

# Scattered darker pebbles and clods.
for _ in range(1100):
    cx, cy = rng.integers(0, SIZE, 2); r = int(rng.integers(2, 8))
    yy, xx = np.ogrid[-r:r+1, -r:r+1]
    m = (xx * xx + yy * yy <= r * r)[..., None]
    ys = (np.arange(cy - r, cy + r + 1) % SIZE)[:, None]
    xs = (np.arange(cx - r, cx + r + 1) % SIZE)[None, :]
    img[ys, xs] = np.where(m, img[ys, xs] * rng.uniform(0.58, 0.86), img[ys, xs])

Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save('dirt_ground.png')
a = np.clip(img, 0, 1)
# Row/column means should not oscillate if the texture is directionless.
print(f'wrote dirt_ground.png {SIZE}x{SIZE}  row-mean std={a.mean(axis=(1,2)).std():.4f} '
      f'col-mean std={a.mean(axis=(0,2)).std():.4f} (low = no banding)')
