# Station calibration (ElRobot, 2026-11-21)

`station-calib.zip` holds the frozen ST3215 servo calibration saved by NormaCore Station:
`station_data/.crypto_seed` plus `station_data/<id>/st3215/meta/`. The `<id>` directory name is derived
from the seed, so both are needed together.

Restore (with Station stopped), from the project root:

    cd station && unzip -o ../calibration/station-calib.zip

Then start Station from `station/` as usual (`../bin/station --web 127.0.0.1:8889 --tcp`).

## Overload threshold (not in the zip: it lives in the servos)

The arm servos' overload-protection threshold was raised from the Station's 25 % to 60 % on
2026-11-21 (`uv run python scripts/servo_overload.py 60`, arm resting, torque off). Running the
Station's auto-calibration writes 25 back; set it again afterwards, or the shoulder drops out
mid-lift on the 7.5 V supply.
