#!/usr/bin/env python3
from PIL import Image
import os

# allow huge images
Image.MAX_IMAGE_PIXELS = None

# 1) load
im = Image.open("grass_dry.png")

# 2) quantize to a small palette (e.g. 128 colors)
im = im.convert("P", palette=Image.ADAPTIVE, colors=256)

# 3) write with maximum zlib compression
im.save(
    "grass_dry_256c.png",
    optimize=True,
    compress_level=9  # 0–9, 9 is slowest but smallest
)

# report size
size = os.path.getsize("grass_dry_256c.png") / 1024**2
print(f"→ grass_dry_256c.png = {size:.1f} MB")
