"""Decide what to draw: turn a picture into pen strokes, or take strokes you wrote yourself.

Exactly one source:
 * `image_path`: a picture file (a line drawing: dark lines on a light ground).
 * `image_id` (+ `quad_px`): a camera capture, to copy a drawing that lies on the table.
   `quad_px` is the four corners of that drawing's sheet in the capture, in pixels: top-left,
   top-right, bottom-right, bottom-left as the drawing reads. The sheet is straightened first.
 * `strokes`: a list of polylines, each a list of [x, y] points, x to the right and y upward,
   in any units. This is how to draw something from imagination or to write text: compose the
   lines yourself.

Lines of the picture are thinned to their centre lines, so a thick line or a filled shape
becomes single strokes. `detail` is how closely curves are followed, as a fraction of the
drawing's width (smaller keeps more points). `invert: true` is for light lines on dark.

Does not move the arm. Returns the number of strokes and `preview_image`: black strokes
numbered in drawing order, pen-up travel in pale red. Open it before drawing: it is exactly
what the pen will do. The drawing is kept until the next draw_plan; draw_run draws it.
"""

import cv2

from util import capture, strokes as S
from util.arm import RobotError


def draw_plan(ctx, image_path: str | None = None, image_id: str | None = None, quad_px: list | None = None, strokes: list | None = None,
              invert: bool = False, detail: float = 0.004) -> dict:  # fmt: skip
    if sum(x is not None for x in (image_path, image_id, strokes)) != 1:
        raise RobotError("give exactly one of image_path, image_id, strokes")
    if strokes is not None:
        drawing = S.normalise(strokes)
    else:
        if image_path is not None:
            img = cv2.imread(str(ctx.root / image_path if not str(image_path).startswith("/") else image_path))
            if img is None:
                raise RobotError(f"cannot read a picture from '{image_path}'")
        else:
            img, _ = capture.load(ctx, image_id)
        if quad_px is not None and len(quad_px) != 4:
            raise RobotError("quad_px is four [u, v] pixels: top-left, top-right, bottom-right, bottom-left")
        drawing = S.vectorize(img, quad_px, invert, detail)
    st = S.load()
    st.update({"drawing": drawing, "strokes_done": 0})
    S.save(st)
    return {
        "strokes": len(drawing["strokes"]),
        "height_over_width": drawing["height"],
        "line_length_in_widths": round(S.length(drawing), 2),
        "preview_image": ctx.save_image("preview.jpg", S.preview(drawing), "what the pen will draw: strokes numbered in order, pen-up travel in pale red"),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(draw_plan)
