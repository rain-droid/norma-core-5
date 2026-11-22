"""Take a still from a camera and freeze it with what is needed to measure in it.

Cameras: `top` hangs above the table and sees the whole mat: use it to find things and to
measure where they are. The robot is at the right edge of its image and reaches towards the
left, so the robot's forward is image-left and its right is image-up. `wrist` rides on the gripper and sees what the gripper points at (your own jaws
are the black shapes at the bottom edge): use it to look closely and to check a grasp.
`across` is the colour view of the Orbbec depth camera across the table: use it to inspect
stacking and clearance from the side. It is a colour image, not a depth measurement; use
depth_scan for heights. Use only top-camera pixels for calibrated table x/y.

Does not move the arm. Returns an `image_id`: pass it, with a pixel, to pixel_to_table. Pixels
are those of the unmodified image (`image`); `grid_image` is the same frame with labelled
gridlines every 100 px to read coordinates from. Open the image file to see it.
"""

from util import capture
from util.motion import tcp_report


def capture_camera(ctx, camera: str = "top", grid: bool = True) -> dict:
    img = ctx.frame(camera, settle=0.15)
    meta = capture.freeze(ctx, camera, img, grid)
    state = ctx.arm.state()
    return {
        "image_id": meta["image_id"],
        "camera": camera,
        "width": meta["width"],
        "height": meta["height"],
        "image": meta["image"],
        "grid_image": meta.get("grid_image"),
        "tcp": tcp_report(state["ee"], state["gripper"]) if state["ee"] else None,
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(capture_camera)
