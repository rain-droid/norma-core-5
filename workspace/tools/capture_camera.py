"""Take a still from a camera and freeze it with the pose it was taken from.

Cameras: `wrist` rides on the gripper and sees what the gripper points at (your own jaws are
the black shapes at the bottom edge); `top` is fixed above the table and gives an overview.
Does not move the arm. Returns an `image_id`: pass it, with a pixel, to pixel_to_table.
Pixels are those of the unmodified image (`image`); `grid_image` is the same frame with
labelled gridlines every 100 px to read coordinates from. Open the image file to see it.
"""

from util import capture
from util.motion import tcp_report

MOTION = False


async def capture_camera(ctx, camera: str = "wrist", grid: bool = True) -> dict:
    img = await ctx.frame(camera, settle=0.15)
    meta = capture.freeze(ctx, camera, img, grid)
    return {
        "image_id": meta["image_id"],
        "camera": camera,
        "width": meta["width"],
        "height": meta["height"],
        "image": meta["image"],
        "grid_image": meta.get("grid_image"),
        "gives_table_positions": meta["T_base_tool"] is not None,
        "tcp": tcp_report(ctx) if ctx.robot.ready else None,
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(capture_camera)
