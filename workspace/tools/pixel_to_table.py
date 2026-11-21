"""Turn a pixel of a frozen wrist capture into a position on the table.

Casts the ray through pixel (u, v) from where the camera was when `image_id` was captured and
intersects it with a horizontal plane. `height` is that plane's height above the table in
metres: 0 for a spot on the table, an object's height for a spot on its top face. A point on
the side or top of an object, read with height 0, lands behind the object: for a tall object
aim at where it meets the table, or pass its height.

Returns x, y in metres in the base frame (x right, y forward, origin under the base) and saves
`click.jpg`, the capture with a crosshair on the pixel: open it to check you pointed at what
you meant. Accuracy depends on the camera calibration (see robot_config/README.md).
"""

from util import capture

MOTION = False


async def pixel_to_table(ctx, image_id: str, u: float, v: float, height: float = 0.0) -> dict:
    img, meta = capture.load(ctx, image_id)
    p = capture.table_point(ctx, meta, u, v, height)
    click = ctx.save_image("click.jpg", capture.crosshair(img, [(u, v, f"({u:.0f},{v:.0f})")]), f"crosshair on pixel ({u:.0f}, {v:.0f}) of {image_id}")
    ctx.emit("markers", markers=[{"id": "pixel", "label": f"({u:.0f},{v:.0f})", "pos": [float(c) for c in p], "color": "#ff6ad5", "radius": 0.01}])
    return {
        "x": round(float(p[0]), 3),
        "y": round(float(p[1]), 3),
        "z": round(float(p[2]), 3),
        "image_id": image_id,
        "pixel": [u, v],
        "click_image": click,
        "camera_calibrated": bool(ctx.skills.cam.calibrated),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(pixel_to_table)
