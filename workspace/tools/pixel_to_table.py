"""Turn a pixel of a frozen capture into a position in the robot's frame.

`height` is how far above the table the thing at that pixel is, in metres. Use 0 for a spot on
the table, such as the edge where an object meets it. For a spot on top of an object, give the
object's height: a point on a top face read with height 0 comes out behind the object, further
from the camera, by about the object's height for the top camera.

`top` captures are the accurate ones: the mat's four corner tags fix the mapping. `wrist`
captures use a rough camera model and can be centimetres off on the real arm.

Returns x, y, z in metres in the base frame (x right, y forward, origin under the base) and
saves `click.jpg`, the capture with a crosshair on the pixel: open it to check you pointed at
what you meant.
"""

from util import capture


def pixel_to_table(ctx, image_id: str, u: float, v: float, height: float = 0.0) -> dict:
    img, meta = capture.load(ctx, image_id)
    p = capture.table_point(ctx, meta, u, v, height)
    click = ctx.save_image("click.jpg", capture.crosshair(img, [(u, v, f"({u:.0f},{v:.0f})")]), f"crosshair on pixel ({u:.0f}, {v:.0f}) of {image_id}")
    ctx.emit("markers", markers=[{"id": "pixel", "label": f"({u:.0f},{v:.0f})", "pos": [float(c) for c in p], "color": "#ff6ad5", "radius": 0.01}])
    return {"x": round(float(p[0]), 3), "y": round(float(p[1]), 3), "z": round(float(p[2]), 3), "image_id": image_id, "camera": meta["camera"], "pixel": [u, v], "click_image": click}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(pixel_to_table)
