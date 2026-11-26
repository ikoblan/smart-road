#!/usr/bin/env python3
"""Regenerate original BMP assets. Optional development tool: requires Pillow.
The application embeds the prebuilt BMPs, so Python is not needed to play.
Keep these dimensions synchronized with src/config.rs and src/ui.rs.
"""
from pathlib import Path
import math
import random
from PIL import Image, ImageDraw

OUT = Path(__file__).resolve().parents[1] / "assets"
OUT.mkdir(exist_ok=True)
SIZE, CENTRE, SCALE = 840, 420, 7
ROAD_HALF = 18


def pt(x: float, y: float) -> tuple[int, int]:
    return round(CENTRE + x * SCALE), round(CENTRE + y * SCALE)


def rotate(x: float, y: float, q: int) -> tuple[float, float]:
    for _ in range(q):
        x, y = -y, x
    return x, y


def build_road() -> None:
    rng = random.Random(2026)
    image = Image.new("RGB", (SIZE, SIZE), (22, 42, 43))
    d = ImageDraw.Draw(image)
    # Quiet block landscaping, with deterministic, subtle texture.
    for _ in range(10000):
        x, y = rng.randrange(SIZE), rng.randrange(SIZE)
        d.point((x, y), fill=rng.choice([(24, 46, 46), (20, 39, 40), (27, 47, 46)]))
    lo, hi = pt(-18, -18)[0], pt(18, 18)[0]
    # Pavement strips, then two intersecting carriageways.
    d.rectangle((lo - 12, 0, hi + 12, SIZE), fill=(62, 79, 82))
    d.rectangle((0, lo - 12, SIZE, hi + 12), fill=(62, 79, 82))
    d.rectangle((lo, 0, hi, SIZE), fill=(37, 46, 58))
    d.rectangle((0, lo, SIZE, hi), fill=(37, 46, 58))
    for _ in range(8500):
        x, y = rng.randrange(SIZE), rng.randrange(SIZE)
        if lo < x < hi or lo < y < hi:
            d.point((x, y), fill=rng.choice([(39, 48, 59), (35, 44, 55)]))
    # Lane separators stay out of the junction itself.
    for q in range(4):
        def line(points, fill=(166, 181, 184), width=2):
            d.line([pt(*rotate(x, y, q)) for x, y in points], fill=fill, width=width)
        for x in [-18, 18]:
            line([(x, -60), (x, -18)], (164, 178, 180), 2)
        for x in [-12, -6, 6, 12]:
            for y in range(-60, -18, 6):
                line([(x, y), (x, min(y + 2.6, -18))], (126, 143, 150), 2)
        for x in [-0.25, 0.25]:
            line([(x, -60), (x, -18.1)], (192, 163, 99), 2)
        # Dashed virtual admission boundary, not a traffic-light stop line.
        for x in [-17, -15, -13, -11, -9, -7, -5, -3, -1]:
            line([(x, -24.5), (x + 0.8, -24.5)], (91, 157, 162), 2)
        # Incoming route arrows: down, then driver's right (screen left), etc.
        for x, turn in [(-15, 0), (-9, 1), (-3, 2)]:
            y = -32
            if turn == 1:
                line([(x, y - 2.0), (x, y + 2.0)], width=3)
                line([(x - 0.8, y + 1.1), (x, y + 2.0), (x + 0.8, y + 1.1)], width=3)
            else:
                direction = -1 if turn == 0 else 1
                end = x + direction * 1.5
                line([(x, y - 2.0), (x, y + 0.8), (end, y + 0.8)], width=3)
                line([(end - direction * 0.7, y), (end, y + 0.8), (end - direction * 0.7, y + 1.6)], width=3)
    # City blocks with shallow shadows, roof seams, and small planted islands.
    for bx, by in [(28, 28), (620, 28), (28, 620), (620, 620)]:
        d.rounded_rectangle((bx + 7, by + 9, bx + 176, by + 156), radius=12, fill=(14, 28, 34))
        d.rounded_rectangle((bx, by, bx + 168, by + 148), radius=9, fill=(40, 64, 73), outline=(61, 88, 95), width=2)
        d.rectangle((bx + 14, by + 15, bx + 154, by + 130), fill=(48, 72, 80))
        for x in range(bx + 23, bx + 142, 24):
            d.rectangle((x, by + 24, x + 12, by + 67), fill=(35, 62, 73), outline=(67, 101, 110))
        d.rectangle((bx + 19, by + 91, bx + 87, by + 112), fill=(60, 84, 89), outline=(76, 104, 110))
        d.rectangle((bx + 105, by + 89, bx + 142, by + 117), fill=(39, 58, 68))
    for x, y in [(239, 59), (239, 144), (239, 230), (58, 239), (146, 239),
                 (599, 59), (599, 145), (599, 230), (688, 239), (776, 239),
                 (239, 599), (239, 688), (239, 776), (59, 599), (146, 599),
                 (599, 599), (599, 688), (599, 776), (688, 599), (776, 599)]:
        d.ellipse((x - 17, y - 14, x + 20, y + 23), fill=(14, 32, 34))
        d.ellipse((x - 18, y - 18, x + 17, y + 17), fill=(38, 80, 68))
        d.ellipse((x - 13, y - 15, x + 9, y + 7), fill=(52, 99, 78))
    image.save(OUT / "road.bmp")


def build_car(name: str, body: tuple[int, int, int]) -> None:
    image = Image.new("RGB", (80, 40), (255, 0, 255))
    d = ImageDraw.Draw(image)
    dark = tuple(round(c * 0.61) for c in body)
    light = tuple(min(255, round(c * 0.70 + 255 * 0.30)) for c in body)
    for x in [15, 55]:
        d.rounded_rectangle((x, 1, x + 12, 9), radius=2, fill=(12, 20, 28))
        d.rounded_rectangle((x, 30, x + 12, 38), radius=2, fill=(12, 20, 28))
    d.rounded_rectangle((3, 5, 77, 35), radius=8, fill=dark, outline=(14, 25, 33), width=2)
    d.rounded_rectangle((6, 7, 74, 31), radius=7, fill=body)
    d.line((12, 8, 67, 8), fill=light, width=2)
    # Cabin, rear glass, windshield, and panoramic roof.
    d.polygon([(24, 8), (51, 8), (59, 13), (59, 27), (51, 32), (24, 32), (20, 28), (20, 12)], fill=(22, 40, 54))
    d.polygon([(48, 10), (54, 13), (57, 16), (57, 25), (48, 29)], fill=(122, 182, 201))
    d.polygon([(22, 13), (28, 10), (28, 29), (22, 27)], fill=(72, 126, 151))
    d.rounded_rectangle((31, 9, 45, 31), radius=2, fill=body)
    d.rectangle((33, 11, 43, 14), fill=light)
    d.rectangle((71, 9, 75, 15), fill=(238, 245, 224))
    d.rectangle((71, 25, 75, 31), fill=(238, 245, 224))
    d.rectangle((4, 10, 7, 15), fill=(241, 89, 82))
    d.rectangle((4, 25, 7, 30), fill=(241, 89, 82))
    d.line((65, 11, 65, 29), fill=light)
    image.save(OUT / f"car_{name}.bmp")


if __name__ == "__main__":
    build_road()
    for name, color in [("cyan", (68, 184, 200)), ("amber", (224, 169, 72)),
                        ("coral", (211, 104, 100)), ("lime", (135, 184, 116))]:
        build_car(name, color)
    print(f"Generated five original BMP assets in {OUT}")
