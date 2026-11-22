# Video script, about 3 minutes

Read one block for each slide. About 430 words.

Slide 1
Hi, we built a harness that lets any coding agent, Claude Code, Codex or pi, operate a real robot arm. The idea is simple: the robot only understands joint angles, so that is the only contract.

Slide 2
We started with models that decide the motion directly. We used Dream-1 from DreamLabs as a vision to gripper position API, and its positions were off on both the ElRobot and the SO-101. A fast OpenAI decision model drove the SO-101 well, with no camera calibration, but it plans again each time, and it keeps nothing it learns.

Slide 3
So we inverted it. The agent starts in a workspace folder of Python tools. It reads them, runs them, and writes new ones. Everything, inverse kinematics, grasp planning, camera geometry, collision checks, lives in that Python. The robot side accepts exactly one thing: a list of joint angles, sent through the NormaCore Station.

Slide 4
Here is what that buys. The pick tools knew cubes, not rings, and a ring's centre is a hole. Asked to move the rubber ring, the agent read a point on the rim from the camera and put one finger down the hole. Then it wrote that method into its pick-and-place skill, so the next agent starts from it.

Slide 5
Tools do not return OK. They return what the robot measured: the width in the jaws, a height that was 16 millimetres short. That number is what the agent fixes from, and the fix stays as a tool or a skill.

Slide 6
Then two arms. We put an SO-101 next to the ElRobot, on the same Station, and asked for a hand-over of a cube in the air. We showed it once, by hand, with the motors off. The agent replayed it, saw why it missed, and changed its own tools. The first eight attempts gave one success, the last eight gave five, over about ninety minutes.

Slide 7
Which changes mattered? Reading the SO-101's joints back and correcting its height took the computed hand-over from zero of three to two of two. The poses from the demonstration then gave four of six.

Slide 8
Why not every time? The jaws open 53 millimetres around a 30 millimetre cube: 11 millimetres each side. The worst errors we measured are bigger than that. That is the next thing to fix: camera guidance at the grip.

Slide 9
To keep two arms from hitting each other, both live in one model, with their wrist cameras, and the depth camera adds what stands on the table. A path that comes within 2 centimetres of the other arm, or through an object, is refused, and the twin in the web UI shows it.

Slide 10
So: a direct model is the fast path for simple tasks. Code as policy is the path when the same commands must run every time, and when a fix must stay. The code, the data and every result are in the repository. Thank you.
