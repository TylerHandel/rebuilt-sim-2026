"""Pixel difference of two screenshots: imgdiff.py a.png b.png [diff.png]
Prints the share of pixels that differ by more than 2 and by more than 24 (of 255, any channel), and
the largest difference; writes the differing pixels in red over a dimmed copy of a."""
import sys

import numpy as np
from PIL import Image

a = np.asarray(Image.open(sys.argv[1]).convert("RGB")).astype(np.int16)
b = np.asarray(Image.open(sys.argv[2]).convert("RGB")).astype(np.int16)
d = np.abs(a - b).max(axis=2)
n = d.size
print(f"pixels differing >2: {100 * (d > 2).sum() / n:.3f}%, >24: {100 * (d > 24).sum() / n:.3f}%, max {d.max()}")
if len(sys.argv) > 3:
    out = (a * 0.35).astype(np.uint8)
    out[d > 2] = [255, 0, 0]
    Image.fromarray(out).save(sys.argv[3])
