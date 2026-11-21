# Hand-guided touch calibration

The click GUI runs at http://127.0.0.1:8780: `python -m so101_station.touch_ui --bus-serial BUS --camera-queue QUEUE`. It opens with a fresh photo, supports Fit/150%/200% zoom, and saves midpoint clicks in original image pixels. Confirm physical contact before Capture touch; a photo-only snapshot cannot be saved as a touch. Existing points can be reviewed or excluded without deleting them. Replace obscured points with clearer captures. The service has no motor-write capability. Excluded records must be omitted from fitting.

`python -m so101_station.touch capture --bus-serial BUS --camera-queue QUEUE --contact-confirmed` records a stable downward fingertip contact and its camera image. It never sends motor commands. The operator supports the arm, touches both fingertips to bare table and leaves the base/camera fixed. Wrist torque may remain on; do not force that joint. Other motors must be unpowered.

View each saved image, then annotate the midpoint between the two fingertip contact points with `python -m so101_station.touch mark RECORD.json --pixel U V`. Do not use the gripper housing center. If contact is obscured, retake or have the operator identify the point. Image selection uncertainty must be checked physically before motion.

Capture six points spread across the reachable pickup and drop area plus an independent seventh point inside their hull. Fit with `python -m so101_station.touch fit FIRST.json SECOND.json ... CHECK.json --drop-pixel U V --out logs/touch-calibration/workspace.json`. The last point is held out from fitting. Height disagreement above 4 mm or held-out position error above 5 mm fails. The output uses provisional grasp/travel/drop heights and remains bench-unverified: verify clearance and the plastic cover height before execution.

2026-11-21: first contact saved as logs/touch-calibration/19dd1e292ac64fbaa2974cfccf8f8dd9.json. FK tip height 1.81 mm in the robot base frame; image midpoint estimated at (493,322). More points and independent validation are required; no calibrated pickup yet.
