#!/usr/bin/env python3
"""Prove generate_ground_colormap.py's output has no periodic repeat: FFT autocorrelation
(largest secondary peak vs. zero-lag) + normalized cross-correlation template matching of
sub-patches against the rest of the image. Downsamples to MAX_SIDE first - a repeat visible at
normal viewing distance shows up fine at this resolution, and it keeps the FFT tractable.

Usage: python3 verify_ground_colormap.py <image.png> [--tile-pitch-m N --px-per-m N]
--tile-pitch-m/--px-per-m are optional: when given, also reports the autocorrelation value at
that exact repeat distance, for comparing a naive tiled texture's own repeat pitch against the
"no real peak" case.
"""
import argparse

import numpy as np
from PIL import Image
from scipy.signal import fftconvolve

MAX_SIDE = 2048


def load_gray(path):
    im = Image.open(path).convert("L")
    native_size = im.size
    if im.size[0] > MAX_SIDE:
        im = im.resize((MAX_SIDE, MAX_SIDE), Image.LANCZOS)
    return np.asarray(im).astype(np.float64), native_size


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("--tile-pitch-m", type=float, default=None)
    parser.add_argument("--px-per-m", type=float, default=None)
    args = parser.parse_args()

    raw, native_size = load_gray(args.image)
    arr = (raw - raw.mean()) / raw.std()
    h, w = arr.shape

    f = np.fft.fft2(arr)
    ac = np.fft.ifft2(f * np.conj(f)).real
    ac = np.fft.fftshift(ac)
    zero_lag = ac[h // 2, w // 2]

    # exclude a window around zero-lag covering the grain's own local correlation length (natural
    # 1/f falloff of any photo, not a "repeat") - a real repeat shows as a peak far outside this.
    excl = max(40, h // 20)
    mask = np.ones_like(ac, dtype=bool)
    mask[h // 2 - excl : h // 2 + excl, w // 2 - excl : w // 2 + excl] = False
    secondary = ac.copy()
    secondary[~mask] = -np.inf
    peak_idx = np.unravel_index(np.argmax(secondary), secondary.shape)
    peak_val = secondary[peak_idx]
    dy, dx = peak_idx[0] - h // 2, peak_idx[1] - w // 2
    peak_pct = peak_val / zero_lag * 100
    print(
        f"{args.image}: {native_size[0]}x{native_size[1]}px source, downsampled to {w}x{h} for FFT"
    )
    print(
        f"autocorrelation: zero-lag={zero_lag:.1f}, largest off-center peak={peak_val:.1f} "
        f"({peak_pct:.2f}% of zero-lag) at lag (dx={dx}, dy={dy})"
    )

    if args.tile_pitch_m is not None and args.px_per_m is not None:
        px_per_m_ds = args.px_per_m * (w / native_size[0])
        pitch_px = round(args.tile_pitch_m * px_per_m_ds)
        if pitch_px < w:
            pitch_val = ac[h // 2, min(w - 1, w // 2 + pitch_px)] / zero_lag * 100
            print(
                f"autocorrelation at the declared tile pitch ({args.tile_pitch_m}m = "
                f"{pitch_px}px, dy=0): {pitch_val:.2f}% of zero-lag"
            )

    # --- template matching: random sub-patches against the whole image ---
    rng = np.random.default_rng(0)
    patch_size = 128
    N = patch_size * patch_size
    ii = np.zeros((h + 1, w + 1))
    ii[1:, 1:] = np.cumsum(np.cumsum(raw, axis=0), axis=1)
    ii2 = np.zeros((h + 1, w + 1))
    ii2[1:, 1:] = np.cumsum(np.cumsum(raw * raw, axis=0), axis=1)
    ph = pw = patch_size
    win_sum = ii[ph:, pw:] - ii[:-ph, pw:] - ii[ph:, :-pw] + ii[:-ph, :-pw]
    win_sq_sum = ii2[ph:, pw:] - ii2[:-ph, pw:] - ii2[ph:, :-pw] + ii2[:-ph, :-pw]
    win_mean = win_sum / N
    win_std = np.sqrt(np.clip(win_sq_sum / N - win_mean**2, 1e-6, None))

    best_matches = []
    for _ in range(6):
        y0 = rng.integers(0, h - patch_size)
        x0 = rng.integers(0, w - patch_size)
        patch = raw[y0 : y0 + patch_size, x0 : x0 + patch_size]
        t = patch - patch.mean()
        t_std = patch.std()
        number = fftconvolve(raw, t[::-1, ::-1], mode="valid")
        ncc = number / (win_std * t_std * N)
        corr_masked = ncc.copy()
        sy0, sx0 = max(0, y0 - patch_size), max(0, x0 - patch_size)
        sy1, sx1 = min(ncc.shape[0], y0 + patch_size), min(
            ncc.shape[1], x0 + patch_size
        )
        corr_masked[sy0:sy1, sx0:sx1] = -np.inf
        best = float(np.max(corr_masked))
        by, bx = np.unravel_index(np.argmax(corr_masked), corr_masked.shape)
        print(
            f"template match at ({x0},{y0}): best off-self NCC={best:.3f} at ({bx},{by})"
        )
        best_matches.append(best)

    print(
        f"summary: secondary autocorr peak={peak_pct:.2f}% of zero-lag, "
        f"max template-match NCC={max(best_matches):.3f}"
    )


if __name__ == "__main__":
    main()
