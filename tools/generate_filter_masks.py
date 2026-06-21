from pathlib import Path

from PIL import Image, ImageDraw


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = WORKSPACE_ROOT / "src" / "forklift_nav_bringup" / "maps"
MAP_PATH = MAP_DIR / "warehouse_map.png"
KEEPOUT_MASK_PATH = MAP_DIR / "warehouse_keepout_mask.png"
SPEED_MASK_PATH = MAP_DIR / "warehouse_speed_mask.png"

RESOLUTION = 0.05
ORIGIN_X = -7.0
ORIGIN_Y = -10.5


def world_to_pixel(x: float, y: float, height: int) -> tuple[int, int]:
    px = int(round((x - ORIGIN_X) / RESOLUTION))
    py = height - int(round((y - ORIGIN_Y) / RESOLUTION))
    return px, py


def draw_world_rect(draw: ImageDraw.ImageDraw, x0: float, y0: float, x1: float, y1: float, value: int, height: int) -> None:
    p0 = world_to_pixel(x0, y0, height)
    p1 = world_to_pixel(x1, y1, height)
    left = min(p0[0], p1[0])
    right = max(p0[0], p1[0])
    top = min(p0[1], p1[1])
    bottom = max(p0[1], p1[1])
    draw.rectangle([left, top, right, bottom], fill=value)


def main() -> None:
    base = Image.open(MAP_PATH).convert("L")
    keepout = Image.new("L", base.size, 0)
    speed = Image.new("L", base.size, 0)

    keepout_draw = ImageDraw.Draw(keepout)
    speed_draw = ImageDraw.Draw(speed)

    # Keepout buffers around shelves / rack islands.
    keepout_rects = [
        (-3.8, 5.2, -1.9, 7.0),
        (-3.8, 1.4, -1.6, 4.0),
        (-3.4, -8.9, -1.7, -5.6),
        (1.1, 6.0, 5.1, 9.0),
        (1.1, 3.4, 5.0, 6.0),
        (1.1, 1.0, 5.0, 3.4),
    ]
    for rect in keepout_rects:
        draw_world_rect(keepout_draw, *rect, value=100, height=keepout.height)

    # Speed-limited regions. Values are percentages of max speed.
    speed_rects = [
        (-4.2, -9.2, -1.0, -5.0, 50),
        (-4.4, 0.6, -0.8, 7.2, 65),
        (-0.6, 5.0, 5.6, 9.6, 70),
        (0.8, -3.6, 5.8, 4.6, 60),
    ]
    for x0, y0, x1, y1, value in speed_rects:
        draw_world_rect(speed_draw, x0, y0, x1, y1, value=value, height=speed.height)

    keepout.save(KEEPOUT_MASK_PATH)
    speed.save(SPEED_MASK_PATH)
    print(f"Wrote {KEEPOUT_MASK_PATH}")
    print(f"Wrote {SPEED_MASK_PATH}")


if __name__ == "__main__":
    main()
