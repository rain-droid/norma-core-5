"""Build the hackathon deck: docs/presentation/code_as_policy.pptx. The speaker notes are the 3-minute video script.
Run from the repository root: uv run --with python-pptx --with matplotlib python docs/presentation/charts.py && uv run --with python-pptx python docs/presentation/build_deck.py
"""
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_CONNECTOR, MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Inches, Pt

NAVY, BLUE, GREY, LIGHT, WHITE = RGBColor(0x1F, 0x4E, 0x79), RGBColor(0x2E, 0x75, 0xB6), RGBColor(0x59, 0x59, 0x59), RGBColor(0xEA, 0xF1, 0xF8), RGBColor(0xFF, 0xFF, 0xFF)
FONT, IMG, OUT = "Arial", "docs/images/two_arm/", "docs/presentation/"
prs = Presentation()
prs.slide_width, prs.slide_height = Inches(13.333), Inches(7.5)
BLANK = prs.slide_layouts[6]


def text(slide, x, y, w, h, lines, size=20, color=GREY, bold=False, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, bullets=False):
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.word_wrap, tf.vertical_anchor = True, anchor
    for i, line in enumerate([lines] if isinstance(lines, str) else lines):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align
        p.space_after = Pt(10 if bullets else 2)
        run = p.add_run()
        run.text = ("•  " + line) if bullets else line
        run.font.name, run.font.size, run.font.bold, run.font.color.rgb = FONT, Pt(size), bold, color
    return box


def slide(title, notes, source=None):
    s = prs.slides.add_slide(BLANK)
    text(s, 0.6, 0.35, 12.1, 1.25, title, size=27, color=NAVY, bold=True, anchor=MSO_ANCHOR.MIDDLE)
    if source:
        text(s, 0.6, 6.9, 12.1, 0.4, source, size=12, color=GREY)
    s.notes_slide.notes_text_frame.text = notes
    return s


def box(s, x, y, w, h, label, fill=LIGHT, color=NAVY, size=16, bold=True):
    shp = s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(h))
    shp.fill.solid(); shp.fill.fore_color.rgb = fill; shp.line.color.rgb = BLUE
    tf = shp.text_frame; tf.word_wrap = True; tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    for i, line in enumerate(label.split("\n")):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph(); p.alignment = PP_ALIGN.CENTER
        r = p.add_run(); r.text = line; r.font.name, r.font.size, r.font.bold, r.font.color.rgb = FONT, Pt(size if i == 0 else size - 3), bold and i == 0, color
    return shp


def arrow(s, x1, y1, x2, y2, label=None, color=BLUE):
    c = s.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(x1), Inches(y1), Inches(x2), Inches(y2))
    c.line.color.rgb, c.line.width = color, Pt(2.5)
    c.line._get_or_add_ln().append(c.line._get_or_add_ln().makeelement("{http://schemas.openxmlformats.org/drawingml/2006/main}tailEnd", {"type": "triangle"}))
    if label:
        text(s, min(x1, x2) - 0.6, max(y1, y2) + 0.75, abs(x2 - x1) + 1.2, 0.4, label, size=14, color=NAVY, bold=True, align=PP_ALIGN.CENTER)


def picture(s, path, x, y, w=None, h=None):
    return s.shapes.add_picture(path, Inches(x), Inches(y), Inches(w) if w else None, Inches(h) if h else None)


# 1. title
s = prs.slides.add_slide(BLANK)
text(s, 0.8, 1.7, 11.7, 1.6, "One contract, any agent: code as policy for two robot arms", size=40, color=NAVY, bold=True)
text(s, 0.8, 3.4, 11.7, 1.0, "The agent writes Python. The Python moves the arm. The robot only understands joint angles.", size=22, color=GREY)
text(s, 0.8, 5.3, 11.7, 1.2, ["Atharva Bhorpe, Arian Malekzadeh, Alexander Adamov, Rayane Bendjelida", "HackLab Robotics x BRISA, NormaCore track, Berlin, 22 November 2026"], size=18, color=GREY)
s.notes_slide.notes_text_frame.text = ("Hi, we built a harness that lets any coding agent, Claude Code, Codex or pi, operate a real robot arm. "
                                       "The idea is simple: the robot only understands joint angles, so that is the only contract.")

# 2. complication
s = slide("Before the harness: Dream-1 positions were off, and the fast decision model kept nothing it learned",
          "We started with models that decide the motion directly. We used Dream-1 from DreamLabs as a vision to gripper position API, and its positions were off on both the ElRobot and the SO-101. "
          "A fast OpenAI decision model drove the SO-101 well, with no camera calibration, but it plans again each time, and it keeps nothing it learns.",
          source="Dream-1 and decision-model observations: team report during the hackathon; no measurements recorded in the repository.")
text(s, 0.6, 1.9, 5.9, 4.6, ["Dream-1 (DreamLabs), vision to gripper position: positions off on both arms",
                             "OpenAI decision model on the SO-101: fast, cheap, no calibration step",
                             "But: a new plan each time, nothing kept"], size=21, bullets=True)
picture(s, IMG + "camera_across.jpg", 7.0, 1.9, w=5.7)

# 3. the contract
s = slide("The robot understands joint angles, so joint angles are the whole contract",
          "So we inverted it. The agent starts in a workspace folder of Python tools. It reads them, runs them, and writes new ones. "
          "Everything, inverse kinematics, grasp planning, camera geometry, collision checks, lives in that Python. "
          "The robot side accepts exactly one thing: a list of joint angles, sent through the NormaCore Station.")
box(s, 0.6, 2.6, 2.0, 1.2, "Person\ntext or speech")
box(s, 3.2, 2.6, 2.3, 1.2, "Coding agent\nClaude Code, Codex, pi")
box(s, 6.1, 2.1, 2.7, 2.2, "workspace/\ntools/*.py, skills/*.md\nAGENTS.md", size=17)
box(s, 9.7, 2.6, 1.6, 1.2, "Daemon\nlimit check")
box(s, 11.6, 2.6, 1.3, 1.2, "Station\nservos")
arrow(s, 2.6, 3.2, 3.2, 3.2); arrow(s, 5.5, 3.2, 6.1, 3.2); arrow(s, 8.8, 3.2, 9.7, 3.2); arrow(s, 11.3, 3.2, 11.6, 3.2)
text(s, 0.6, 4.4, 12.1, 0.5, "The agent writes and runs the tools.   The tools send joint angles, nothing else.", size=18, color=NAVY, bold=True)
text(s, 0.6, 5.1, 12.1, 1.2, 'POST /api/joints   {"path": [[0.0, -0.37, -0.51, 0.63, 0.0, 1.28, 0.0]], "gripper": 0.5}', size=20, color=NAVY, bold=True)
text(s, 0.6, 5.8, 12.1, 0.9, "Same task, second time: the same script, the same commands.", size=20)

# 4. zero shot ring
s = slide("On a first-time object, the agent found a way and wrote it into its skill",
          "Here is what that buys. The pick tools knew cubes, not rings, and a ring's centre is a hole. "
          "Asked to move the rubber ring, the agent read a point on the rim from the camera and put one finger down the hole. "
          "Then it wrote that method into its pick-and-place skill, so the next agent starts from it.",
          source="First-attempt success: team report. The skill text is in workspace/skills/pick-and-place/SKILL.md.")
picture(s, IMG + "camera_top.jpg", 0.6, 1.9, w=6.2)
text(s, 7.2, 1.9, 5.5, 4.8, ["Tracked centre of a ring is the hole",
                             "Agent: one point on the rim, one finger in the hole",
                             "Worked on the first attempt (team report)",
                             "Method written into skills/pick-and-place"], size=21, bullets=True)

# 5. recovery
s = slide("Recovery is code: each measured failure became a fix that stays in the workspace",
          "Tools do not return OK. They return what the robot measured: the width in the jaws, a height that was 16 millimetres short. "
          "That number is what the agent fixes from, and the fix stays as a tool or a skill.")
rows = [("Jaws close on nothing", "look again, aim again"), ("Move stopped as blocked", "sent again at half speed"),
        ("Arm stops off the pose", "read the joints, correct"), ("Second arm misses the cube", "offer it 1 cm further"),
        ("Path through the other arm", "refused before it moves")]
tbl = s.shapes.add_table(len(rows) + 1, 2, Inches(0.6), Inches(1.9), Inches(12.1), Inches(4.4)).table
for c, h in enumerate(("Failure the tool measures", "What the code does")):
    tbl.cell(0, c).text = h
for r, (a, b) in enumerate(rows, 1):
    tbl.cell(r, 0).text, tbl.cell(r, 1).text = a, b
for r in range(len(rows) + 1):
    for c in range(2):
        for p in tbl.cell(r, c).text_frame.paragraphs:
            for run in p.runs:
                run.font.name, run.font.size = FONT, Pt(20)

# 6. demonstration result
s = slide("After one demonstration, the hand-over went from 1 of 8 to 5 of 8",
          "Then two arms. We put an SO-101 next to the ElRobot, on the same Station, and asked for a hand-over of a cube in the air. "
          "We showed it once, by hand, with the motors off. The agent replayed it, saw why it missed, and changed its own tools. "
          "The first eight attempts gave one success, the last eight gave five, over about ninety minutes.",
          source="Real arms, 22 November 2026. Every attempt in which both arms went for the cube. Run log: docs/RESULTS.md.")
picture(s, OUT + "deck_attempts.png", 0.5, 1.8, w=7.6)
text(s, 8.4, 1.9, 4.3, 4.8, ["Shown once, motors off, 9 poses",
                             "Agent changed its own tools 5 times",
                             "Last 8 attempts: 5 worked",
                             "Small counts: direction, not proof"], size=20, bullets=True)

# 7. ablation
s = slide("Reading the joints back mattered more than where the poses came from",
          "Which changes mattered? Reading the SO-101's joints back and correcting its height took the computed hand-over from zero of three to two of two. "
          "The poses from the demonstration then gave four of six.",
          source="Real arms; small counts. Attempt 8 left out: a hand was on the arm.")
picture(s, OUT + "deck_ablation.png", 0.5, 1.8, w=7.6)
text(s, 8.4, 1.9, 4.3, 4.8, ["Height corrected from joints: 0 of 3 to 2 of 2",
                             "Poses shown by hand: 4 of 6",
                             "One adjusted pose: 1 of 2"], size=20, bullets=True)

# 8. error budget
s = slide("The remaining misses come from errors as large as the room in the jaws",
          "Why not every time? The jaws open 53 millimetres around a 30 millimetre cube: 11 millimetres each side. "
          "The worst errors we measured are bigger than that. That is the next thing to fix: camera guidance at the grip.")
picture(s, OUT + "deck_error_budget.png", 0.5, 1.8, w=7.6)
text(s, 8.4, 1.9, 4.3, 4.8, ["Jaws 53 mm, cube 30 mm: 11 mm each side",
                             "Two errors larger than that",
                             "Next: camera guidance at the grip"], size=20, bullets=True)

# 9. shared space
s = slide("Both arms share one collision-checked model, fed by the depth camera",
          "To keep two arms from hitting each other, both live in one model, with their wrist cameras, and the depth camera adds what stands on the table. "
          "A path that comes within 2 centimetres of the other arm, or through an object, is refused, and the twin in the web UI shows it.",
          source="Depth camera: Orbbec DaBai DCW2; a 30 mm cube read 29 mm.")
picture(s, IMG + "webui_two_arms.jpg", 0.5, 1.8, w=7.6)
text(s, 8.4, 1.9, 4.3, 4.8, ["Two arms as capsules, wrist cameras included",
                             "Depth camera: objects as a height map",
                             "Refused within 2 cm, or through an object",
                             "Stacking by the SO-101: not achieved yet"], size=20, bullets=True)

# 10. conclusions
s = slide("Conclusions: code as policy trades speed for repeatability and memory",
          "So: a direct model is the fast path for simple tasks. Code as policy is the path when the same commands must run every time, "
          "and when a fix must stay. The code, the data and every result are in the repository. Thank you.")
text(s, 0.6, 1.9, 12.1, 3.6, ["Contract: joint angles only; any coding agent",
                              "First-time object handled, method saved as a skill",
                              "Hand-over from one demonstration: 1 of 8 to 5 of 8",
                              "Direct models: faster; harness: repeatable, keeps what it learns"], size=22, bullets=True)
text(s, 0.6, 5.9, 12.1, 0.6, "github.com/rain-droid/norma-core-5", size=22, color=NAVY, bold=True)

# 11. references
s = slide("References", "")
text(s, 0.6, 1.8, 12.1, 4.8, ["Liang, J. et al. (2023). Code as Policies: Language Model Programs for Embodied Control. ICRA 2023.",
                              "NormaCore Station and ElRobot. NormaCore.",
                              "SO-ARM100/101. The Robot Studio and Hugging Face LeRobot.",
                              "Carpentier, J. et al. Pinocchio; Caron, S. Pink: differential inverse kinematics.",
                              "Meta AI. SAM 3: Segment Anything Model 3.",
                              "DreamLabs. Dream-1 API (used as vision to gripper position).",
                              "Orbbec. OrbbecSDK v1 and pyorbbecsdk."], size=18)

# appendix
s = slide("Appendix: the stacking by the second arm failed because its position is 2.5 to 4.5 cm off at the table",
          "", source="docs/RESULTS.md, section 'Stacking with the second arm'.")
picture(s, IMG + "stacking_top_camera.jpg", 0.5, 1.8, w=7.6)
text(s, 8.4, 1.9, 4.3, 4.8, ["Calibrated in the air: 4 to 8 mm", "At the table: 2.5 to 4.5 cm off", "Fix: calibrate at table level"], size=20, bullets=True)

prs.save(OUT + "code_as_policy.pptx")
script = "\n\n".join(f"Slide {i}\n{sl.notes_slide.notes_text_frame.text}" for i, sl in enumerate(prs.slides, 1) if sl.notes_slide.notes_text_frame.text)
open(OUT + "video_script.md", "w").write("# Video script, about 3 minutes\n\nRead one block for each slide. About 430 words.\n\n" + script + "\n")
print(len(prs.slides), "slides;", len(script.split()), "words of script")
