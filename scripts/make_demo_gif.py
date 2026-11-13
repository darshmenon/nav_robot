#!/usr/bin/env python3
"""Render docs/patrol.gif from a recorded run (list of {x, y, yaw, scan}) drawn over the map and zones.
Usage: make_demo_gif.py run.json docs/patrol.gif"""
import json, math, os, sys
from PIL import Image, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HALF, SIZE = 6.0, 600
PX = SIZE / (2 * HALF)
KEEPOUT, SLOW = (2.5, 5.0, -4.5, -1.0), (-1.0, 1.5, 1.0, 4.0)


def w2c(x, y):
    return ((x + HALF) * PX, (HALF - y) * PX)


def base():
    im = Image.open(f"{ROOT}/maps/office.pgm").convert("RGB").resize((SIZE, SIZE), Image.NEAREST)
    d = ImageDraw.Draw(im, "RGBA")
    for z, col in ((KEEPOUT, (220, 40, 40, 90)), (SLOW, (240, 200, 40, 90))):
        d.rectangle([*w2c(z[0], z[3]), *w2c(z[1], z[2])], fill=col)
    return im


def main(src, out):
    raw = json.load(open(src))
    moved = [i for i in range(1, len(raw)) if math.hypot(raw[i]["x"] - raw[i - 1]["x"], raw[i]["y"] - raw[i - 1]["y"]) > 0.005]
    raw = raw[max(moved[0] - 2, 0):moved[-1] + 8]          # drop idle time before and after the run
    frames_in = raw[::max(1, len(raw) // 150)]              # cap at about 150 frames
    bg, path, frames = base(), [], []
    for f in frames_in:
        im = bg.copy()
        d = ImageDraw.Draw(im)
        c, s = math.cos(f["yaw"]), math.sin(f["yaw"])
        for sx, sy in f["scan"]:
            px, py = w2c(f["x"] + sx * c - sy * s, f["y"] + sx * s + sy * c)
            d.rectangle([px - 1, py - 1, px + 1, py + 1], fill=(0, 170, 0))
        path.append(w2c(f["x"], f["y"]))
        if len(path) > 1:
            d.line(path, fill=(30, 90, 220), width=2)
        cx, cy = path[-1]
        d.ellipse([cx - 8, cy - 8, cx + 8, cy + 8], fill=(60, 150, 255), outline=(255, 255, 255))
        d.line([cx, cy, cx + 14 * c, cy - 14 * s], fill=(255, 255, 255), width=2)
        frames.append(im.quantize(64))
    frames[0].save(out, save_all=True, append_images=frames[1:], duration=100, loop=0, optimize=True)
    print(f"{out}: {len(frames)} frames, {os.path.getsize(out) // 1024} KB")


if __name__ == "__main__":
    main(*sys.argv[1:3])
