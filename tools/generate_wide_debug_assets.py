#!/usr/bin/env python3

from pathlib import Path

from PIL import Image, ImageDraw


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = WORKSPACE_ROOT / "src" / "forklift_nav_bringup" / "maps"
MAP_PATH = MAP_DIR / "warehouse_map_wide.png"
KEEPOUT_MASK_PATH = MAP_DIR / "warehouse_keepout_mask_wide.png"
SPEED_MASK_PATH = MAP_DIR / "warehouse_speed_mask_wide.png"

WIDTH = 286
HEIGHT = 423
RESOLUTION = 0.05
ORIGIN_X = -7.0
ORIGIN_Y = -10.5
BOUNDARY_PX = 3

OBSTACLES = [
    (-4.5, 5.5, 1.6, 4.0),
    (4.5, 5.5, 1.6, 4.0),
    (-4.5, 0.0, 1.6, 4.0),
    (4.5, 0.0, 1.6, 4.0),
    (2.8, -5.0, 0.8, 0.8),
]


def world_to_pixel(x: float, y: float) -> tuple[int, int]:
    px = int(round((x - ORIGIN_X) / RESOLUTION))
    py = HEIGHT - int(round((y - ORIGIN_Y) / RESOLUTION))
    return px, py


def draw_world_rect(draw: ImageDraw.ImageDraw, cx: float, cy: float, sx: float, sy: float, value) -> None:
    x0 = cx - sx / 2.0
    x1 = cx + sx / 2.0
    y0 = cy - sy / 2.0
    y1 = cy + sy / 2.0
    p0 = world_to_pixel(x0, y1)
    p1 = world_to_pixel(x1, y0)
    left = min(p0[0], p1[0])
    right = max(p0[0], p1[0])
    top = min(p0[1], p1[1])
    bottom = max(p0[1], p1[1])
    draw.rectangle([left, top, right, bottom], fill=value)


def main() -> None:
    base = Image.new("RGB", (WIDTH, HEIGHT), (255, 255, 255))
    keepout = Image.new("L", (WIDTH, HEIGHT), 0)
    speed = Image.new("L", (WIDTH, HEIGHT), 0)

    base_draw = ImageDraw.Draw(base)
    keepout_draw = ImageDraw.Draw(keepout)

    base_draw.rectangle([0, 0, WIDTH - 1, HEIGHT - 1], outline=(0, 0, 0), width=BOUNDARY_PX)

    for obstacle in OBSTACLES:
        draw_world_rect(base_draw, *obstacle, value=(0, 0, 0))
        draw_world_rect(keepout_draw, *obstacle, value=100)

    base.save(MAP_PATH)
    keepout.save(KEEPOUT_MASK_PATH)
    speed.save(SPEED_MASK_PATH)
    print(f"Wrote {MAP_PATH}")
    print(f"Wrote {KEEPOUT_MASK_PATH}")
    print(f"Wrote {SPEED_MASK_PATH}")


if __name__ == "__main__":
    main()
