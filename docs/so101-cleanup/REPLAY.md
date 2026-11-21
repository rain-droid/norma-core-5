# Offline cleanup run replay

Analyze a saved `SessionHarness` run without contacting Station, importing a robot backend, changing motor state, or modifying its log:

```sh
.venv/bin/python -m so101_station.cleanup.replay logs/cleanup-session/RUN_ID
.venv/bin/python -m so101_station.cleanup.replay logs/cleanup-session/RUN_ID --json
.venv/bin/python -m so101_station.cleanup.replay logs/cleanup-session/RUN_ID --out /tmp/cleanup-report.html
```

`--json` and `--out` can be combined. JSON goes to stdout and includes `schema_version: 1`. The only write is the explicitly requested HTML file. The report needs `events.jsonl`; camera images are optional. HTML has no scripts, remote assets, or embedded image payloads. Its evidence links point to local raster files under the run directory, relative to the HTML location. Keep that relative layout when sharing or moving the report. Copied logs with old absolute image paths can resolve matching image basenames in the copied run directory.

For Python callers, use `build_report(run_directory)` and `render_html(report, output_path)` from `so101_station.cleanup.replay`. Rendering returns HTML text; it does not write the file.

The report contains:

- A timeline in original file order, with final recorded stage and conservative outcome.
- Every request and result, including missing results, orphan results and repeated action IDs. Repeated IDs are diagnosed and paired in file order; their correlations and assessments are marked ambiguous.
- Request-to-result timing grouped by action kind, and separately the harness's reported elapsed time. Both include post-action observation; neither is motor-only execution time.
- Gaps from a preceding action result to the next request, when that ordering is known. These are unattributed waiting periods, which may include observation, operator work or orchestration.
- Per-camera logged frame ages (or a timestamp-derived fallback), frame age when a request referenced that observation, and a limited check of the logged observation revision and 30-second age window. This cannot establish physical freshness or reconstruct unlogged state changes.
- Recorded grasp attempt numbers, first-pass/retry labels, assessments, failures, stop/hold errors and explicit `resume_holding` requests. Recovery does not itself verify delivery and may start with an unknown prior attempt history.
- Diagnostics for malformed JSON, invalid timestamps, clock regression, unsafe image paths and ambiguous correlations. Malformed lines are skipped; subsequent valid events are retained.

`verified_delivery` requires a completed, unambiguous delivered assessment with `source_clear`, `on_cover`, a matching assessment result and final stage `complete`. This reports the recorded assessment, not an independent review of the images. Completed release motion alone is `placement_unverified`. Missing results remain incomplete; an explicit stopped stage remains stopped. Earlier verification records remain visible even if a later task is unfinished. A single run is not a reliability estimate.

The schema's main sections are `summary`, `timing_by_kind`, `inter_action_gap_s`, `frame_age_ms_by_camera`, `actions`, `observations`, `assessments`, `grasp_attempts`, `recoveries`, `failures`, `stops`, `timeline`, `diagnostics` and `notes`. Unknown event types remain in the timeline. Image/base64 payload keys are removed from exported data, and HTML text and attributes are escaped.

Validation:

```sh
.venv/bin/python -m pytest -q tests/test_cleanup_replay.py
```
