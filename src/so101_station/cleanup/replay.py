"""Offline, read-only analysis of SessionHarness JSONL logs. No robot imports."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
import html
import json
import math
import os
from pathlib import Path
import statistics
from urllib.parse import quote, urlsplit


SCHEMA_VERSION = 1
COMPLETED = {"completed", "completed_without_observation"}
NOTES = [
    "Request-to-result latency includes planning, execution, telemetry and post-action observation; motor-only time is not logged.",
    "Inter-action gaps include observation, operator, orchestration and other waiting; their cause is not logged.",
    "Frame ages are recorded values or timestamp differences, not a replay-time freshness guarantee. Observation eligibility uses the logged revision and a 30-second window; physical state changes cannot be reconstructed.",
    "Recorded assessments are operator/model evidence, not independent image verification. A single run does not establish reliability.",
    "Repeated action IDs are ambiguous: results are paired with preceding requests in file order, and every request/result is retained.",
]


def _number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    try:
        return value if math.isfinite(value) else None
    except OverflowError:
        return None


def _stamp(value):
    return value if isinstance(value, int) and not isinstance(value, bool) and 0 <= value < 2**63 else None


def _text(value):
    if not isinstance(value, str):
        return None
    return "[inline data omitted]" if "data:" in value.lower() and ";base64," in value.lower() else value


def _clean(value):
    """Exclude image payloads even if an older exporter embedded tool responses."""
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items()
                if k.lower() not in {"images", "image", "b64", "base64", "jpeg", "data"}}
    if isinstance(value, list):
        return [_clean(v) for v in value]
    if isinstance(value, str):
        return _text(value)
    return value


def _stats(values):
    values = [v for v in values if v is not None]
    return {"count": len(values), "min": min(values) if values else None,
            "median": statistics.median(values) if values else None,
            "max": max(values) if values else None, "total": sum(values)}


def _kind(request):
    tool = _text(request.get("tool")) or "unknown"
    label = request.get({"act": "action", "assess": "outcome", "set_torque": "mode"}.get(tool, "tool"))
    return f"{tool}:{_text(label) or 'unknown'}" if tool != "unknown" else tool


def _image_path(raw, directory):
    """Only link raster files under this run, including logs moved from another host."""
    if not isinstance(raw, str) or not raw or "\x00" in raw:
        return None
    try:
        if urlsplit(raw).scheme:
            return None
    except ValueError:
        return None
    candidate = Path(raw)
    if candidate.suffix.lower() not in {".jpg", ".jpeg", ".png", ".webp", ".gif"}:
        return None
    if candidate.is_absolute():
        try:
            candidate.relative_to(directory)
        except ValueError:
            candidate = directory / candidate.name
    else:
        candidate = directory / candidate
    try:
        return candidate.resolve().relative_to(directory).as_posix()
    except (ValueError, OSError):
        return None


def build_report(run_directory: str | Path) -> dict:
    """Return a versioned, JSON-serializable report without changing the run."""
    directory = Path(run_directory).resolve()
    source = directory / "events.jsonl"
    diagnostics, events = [], []

    def diagnostic(line, code, message):
        diagnostics.append({"line": line, "code": code, "message": message})

    def reject_constant(value):
        raise ValueError(f"non-finite JSON value: {value}")

    def finite_float(value):
        parsed = float(value)
        if not math.isfinite(parsed):
            reject_constant(value)
        return parsed

    previous_time = None
    with source.open(encoding="utf-8", errors="replace") as stream:
        for line, raw in enumerate(stream, 1):
            if not raw.strip():
                continue
            try:
                event = json.loads(raw, parse_constant=reject_constant, parse_float=finite_float)
            except (ValueError, RecursionError):
                diagnostic(line, "malformed_json", "Invalid JSON line; raw content omitted.")
                continue
            if not isinstance(event, dict) or not isinstance(event.get("event"), str):
                diagnostic(line, "invalid_event", "Expected an object with an event name.")
                continue
            event = _clean(event)
            event["line"] = line
            timestamp = _stamp(event.get("time_ns"))
            if timestamp is None:
                diagnostic(line, "invalid_timestamp", "Missing or invalid time_ns; timing is unavailable.")
            elif previous_time is not None and timestamp < previous_time:
                diagnostic(line, "clock_regression", "Timestamp moved backwards; file order is retained.")
            if timestamp is not None:
                previous_time = timestamp
            event["time_ns"] = timestamp
            events.append(event)

    actions, observations, timeline, stops = [], [], [], []
    pending, ids, observation_ids = defaultdict(deque), defaultdict(list), defaultdict(list)
    final_stage, last_attempt, latest_revision = None, None, None
    start_ns = next((e["time_ns"] for e in events if e["time_ns"] is not None), None)
    run_ids = sorted({e["run_id"] for e in events if isinstance(e.get("run_id"), str)})
    if len(run_ids) > 1:
        diagnostic(None, "mixed_run_ids", "The log contains multiple run IDs; treat correlations as untrusted.")

    for event in events:
        line, timestamp, event_kind = event["line"], event["time_ns"], event["event"]
        stage = _text(event.get("stage"))
        if stage is not None:
            final_stage = stage
        if isinstance(event.get("attempt"), int) and not isinstance(event["attempt"], bool) and event["attempt"] >= 0:
            last_attempt = event["attempt"]
        entry = {"line": line, "event": event_kind, "time_ns": timestamp, "stage": stage,
                 "offset_s": (timestamp - start_ns) / 1e9 if timestamp is not None and start_ns is not None else None}
        timeline.append(entry)
        if event_kind == "observation":
            observation = {"line": line, "time_ns": timestamp, "observation_id": _text(event.get("observation_id")),
                           "stage": stage, "attempt": event.get("attempt"), "revision": event.get("revision"), "views": {}}
            if isinstance(observation["revision"], int) and not isinstance(observation["revision"], bool):
                latest_revision = observation["revision"]
            views = event.get("views")
            if not isinstance(views, dict):
                diagnostic(line, "invalid_views", "Observation has no valid camera views.")
                views = {}
            for name, view in views.items():
                if not isinstance(view, dict):
                    diagnostic(line, "invalid_view", "Camera view is not an object.")
                    continue
                captured = _stamp(view.get("captured_ns"))
                age = _number(view.get("age_ms"))
                age_source = "logged" if age is not None else "unavailable"
                if age is None and timestamp is not None and captured is not None:
                    age, age_source = (timestamp - captured) / 1e6, "observation_event_timestamp"
                path = _image_path(view.get("path"), directory)
                if view.get("path") and path is None:
                    diagnostic(line, "unsafe_image_path", "Image link omitted: expected a local raster file within the run.")
                if age is not None and age < 0:
                    diagnostic(line, "negative_frame_age", "Frame age is negative; clocks or metadata disagree.")
                observation["views"][name] = {"camera_id": _text(view.get("camera_id")), "captured_ns": captured,
                    "age_ms": age, "age_source": age_source, "path": path,
                    "exists": bool(path and (directory / path).is_file())}
            observations.append(observation)
            if observation["observation_id"]:
                observation_ids[observation["observation_id"]].append(observation)
                if len(observation_ids[observation["observation_id"]]) > 1:
                    diagnostic(line, "duplicate_observation_id", "Repeated observation ID; request freshness is ambiguous.")
            entry["observation_index"] = len(observations) - 1
        elif event_kind == "request":
            request = event.get("request")
            if not isinstance(request, dict):
                diagnostic(line, "invalid_request", "Request payload is not an object.")
                request = {}
            ident = _text(event.get("action_id"))
            action = {"action_id": ident, "kind": _kind(request), "request_line": line,
                      "request_time_ns": timestamp, "request_stage": stage, "request": request,
                      "result_line": None, "result_time_ns": None, "result_stage": None,
                      "execution": "missing_result", "request_to_result_s": None, "reported_elapsed_s": None,
                      "gap_before_s": None, "attempt": None, "correlation": "unique",
                      "attempt_before": last_attempt,
                      "observation_freshness": {"status": "unknown", "frame_age_at_request_ms": {}},
                      "reason": None, "observation_error": None, "detail": {}}
            candidates = observation_ids.get(request.get("observation_id"), []) if isinstance(request.get("observation_id"), str) else []
            if len(candidates) == 1:
                observation = candidates[0]
                age_s = (timestamp - observation["time_ns"]) / 1e9 if timestamp is not None and observation["time_ns"] is not None else None
                revision_changed = isinstance(observation["revision"], int) and latest_revision is not None and latest_revision != observation["revision"]
                status = "unknown" if age_s is None else ("clock_anomaly" if age_s < 0 else ("stale" if age_s > 30 or revision_changed else "within_logged_window"))
                action["observation_freshness"] = {"status": status, "observation_age_s": age_s,
                    "revision_changed": revision_changed, "frame_age_at_request_ms": {
                        name: (timestamp - view["captured_ns"]) / 1e6 if timestamp is not None and view["captured_ns"] is not None else None
                        for name, view in observation["views"].items()}}
            elif len(candidates) > 1:
                action["observation_freshness"]["status"] = "ambiguous_observation_id"
            actions.append(action)
            entry["action_index"] = len(actions) - 1
            if not ident:
                diagnostic(line, "missing_action_id", "Request has no usable action ID; it cannot be paired.")
            else:
                pending[ident].append(action)
                ids[ident].append(action)
                if len(ids[ident]) > 1:
                    diagnostic(line, "duplicate_action_id", "Repeated action ID; all occurrences retained and marked ambiguous.")
                    for occurrence in ids[ident]:
                        occurrence["correlation"] = "ambiguous"
        elif event_kind == "result":
            ident = _text(event.get("action_id"))
            if ident and pending[ident]:
                action = pending[ident].popleft()
            else:
                diagnostic(line, "orphan_result", "Result has no preceding unmatched request; retained without inferred request.")
                action = {"action_id": ident, "kind": "unknown", "request_line": None, "request_time_ns": None,
                          "request_stage": None, "request": {}, "gap_before_s": None,
                          "attempt_before": None,
                          "request_to_result_s": None, "correlation": "orphan_result", "observation_freshness": {"status": "unknown", "frame_age_at_request_ms": {}}}
                actions.append(action)
            action.update(result_line=line, result_time_ns=timestamp, result_stage=stage,
                          execution=_text(event.get("execution")) or "unknown", attempt=event.get("attempt"),
                          reported_elapsed_s=_number(event.get("elapsed_s")), reason=_text(event.get("reason")),
                          observation_error=_text(event.get("observation_error")),
                          detail=event.get("detail") if isinstance(event.get("detail"), dict) else {})
            if timestamp is not None and action["request_time_ns"] is not None:
                elapsed = (timestamp - action["request_time_ns"]) / 1e9
                if elapsed >= 0:
                    action["request_to_result_s"] = elapsed
                else:
                    diagnostic(line, "negative_duration", "Result precedes request timestamp; duration omitted.")
            entry["action_index"] = next(i for i, item in enumerate(actions) if item is action)
        elif event_kind == "stop":
            if latest_revision is not None:
                latest_revision += 1  # SessionHarness.stop invalidates prior observations.
            stops.append({"line": line, "time_ns": timestamp, "stage": stage, "hold_error": _text(event.get("hold_error"))})
            entry["hold_error"] = stops[-1]["hold_error"]
        else:
            diagnostic(line, "unknown_event", "Unknown event type retained in timeline.")

    previous = None
    by_kind = defaultdict(list)
    for action in actions:
        if action["request_line"] is not None:
            if previous and previous["result_line"] is not None and previous["result_line"] < action["request_line"]:
                begin, end = action["request_time_ns"], previous["result_time_ns"]
                if begin is not None and end is not None and begin >= end:
                    action["gap_before_s"] = (begin - end) / 1e9
            previous = action
        by_kind[action["kind"]].append(action)
    timing = {kind: {"requests": sum(a["request_line"] is not None for a in group),
                     "results": sum(a["result_line"] is not None for a in group),
                     "execution_counts": dict(Counter(a["execution"] for a in group)),
                     "request_to_result_s": _stats(a["request_to_result_s"] for a in group),
                     "reported_elapsed_s": _stats(a["reported_elapsed_s"] for a in group)}
              for kind, group in sorted(by_kind.items())}
    camera_ages = defaultdict(list)
    for observation in observations:
        for name, view in observation["views"].items():
            camera_ages[name].append(view["age_ms"])
    assessments = [{"action_index": i, "outcome": a["request"].get("outcome"), "evidence": a["request"].get("evidence"),
                    "execution": a["execution"], "stage": a["result_stage"], "attempt": a["attempt"],
                    "accepted": a["execution"] in COMPLETED and a["detail"].get("assessment") == a["request"].get("outcome") and a["correlation"] == "unique"}
                   for i, a in enumerate(actions) if a["request"].get("tool") == "assess"]
    recoveries = [{"action_index": i, "execution": a["execution"], "evidence": a["request"].get("params", {}).get("evidence") if isinstance(a["request"].get("params"), dict) else None,
                   "accepted": a["execution"] in COMPLETED and a["correlation"] == "unique"}
                  for i, a in enumerate(actions) if a["kind"] == "act:resume_holding"]
    verified = [a for a in assessments if a["accepted"] and a["outcome"] == "delivered" and a["stage"] == "complete"
                and actions[a["action_index"]]["request"].get("source_clear") is True
                and actions[a["action_index"]]["request"].get("on_cover") is True]
    missing = sum(a["execution"] == "missing_result" for a in actions)
    if final_stage == "stopped":
        outcome = "stopped"
    elif missing:
        outcome = "incomplete"
    elif final_stage == "complete":
        outcome = "verified_delivery" if verified and len(run_ids) <= 1 else "completion_unverified"
    elif final_stage == "awaiting_placement":
        outcome = "placement_unverified"
    elif final_stage in {"retry_required", "blocked"}:
        outcome = final_stage
    else:
        outcome = "incomplete" if actions else "no_actions"
    failures = [{"action_index": i, "execution": a["execution"], "reason": a["reason"], "observation_error": a["observation_error"]}
                for i, a in enumerate(actions) if a["execution"] not in COMPLETED or a["observation_error"]]
    grasp_attempts = []
    for i, action in enumerate(actions):
        if action["kind"] != "act:grasp":
            continue
        attempt, before = action["attempt"], action["attempt_before"]
        started = None
        if isinstance(attempt, int) and not isinstance(attempt, bool):
            if isinstance(before, int):
                started = attempt > before
            elif action["execution"] in COMPLETED | {"empty_grasp"}:
                started = attempt > 0
        label = "not_attempted" if started is False else "unknown"
        if started:
            label = "first_pass" if attempt == 1 else "retry"
        grasp_attempts.append({"action_index": i, "attempt": attempt, "attempt_started": started,
                               "execution": action["execution"], "pass": label})
    end_ns = next((e["time_ns"] for e in reversed(events) if e["time_ns"] is not None), None)
    return {"schema_version": SCHEMA_VERSION, "run_directory": str(directory), "run_ids": run_ids,
            "summary": {"final_stage": final_stage, "outcome": outcome, "last_recorded_attempt": last_attempt,
                        "verified_delivery_records": len(verified), "missing_results": missing,
                        "event_count": len(events), "action_requests": sum(a["request_line"] is not None for a in actions),
                        "start_time_ns": start_ns, "end_time_ns": end_ns,
                        "observed_duration_s": (end_ns - start_ns) / 1e9 if start_ns is not None and end_ns is not None and end_ns >= start_ns else None},
            "timing_by_kind": timing, "inter_action_gap_s": _stats(a["gap_before_s"] for a in actions),
            "frame_age_ms_by_camera": {name: _stats(ages) for name, ages in sorted(camera_ages.items())},
            "actions": actions, "observations": observations, "assessments": assessments,
            "grasp_attempts": grasp_attempts, "recoveries": recoveries, "failures": failures,
            "stops": stops, "timeline": timeline, "diagnostics": diagnostics, "notes": NOTES}


def render_html(report: dict, output_path: str | Path) -> str:
    """Render escaped text and local evidence links, without scripts or remote assets."""
    output = Path(output_path).resolve()
    directory = Path(report["run_directory"])

    def esc(value):
        return html.escape("—" if value is None else str(value), quote=True)

    def table(headers, rows):
        return "<table><thead><tr>" + "".join(f"<th>{esc(h)}</th>" for h in headers) + "</tr></thead><tbody>" + "".join("<tr>" + "".join(f"<td>{cell}</td>" for cell in row) + "</tr>" for row in rows) + "</tbody></table>"

    summary = report["summary"]
    sections = ["<!doctype html><html lang='en'><meta charset='utf-8'><meta name='viewport' content='width=device-width'><title>Cleanup run replay</title>",
                "<style>body{font:15px system-ui;margin:2rem;color:#20242a}table{border-collapse:collapse;width:100%;margin:1rem 0}th,td{text-align:left;vertical-align:top;border-bottom:1px solid #ddd;padding:.5rem;overflow-wrap:anywhere}th{background:#eee}pre{white-space:pre-wrap}a{color:#1453a3}</style><body>",
                "<h1>Cleanup run replay</h1>", f"<p>Run: {esc(', '.join(report['run_ids']) or directory.name)}</p>",
                f"<p>Final recorded stage: <strong>{esc(summary['final_stage'])}</strong>; outcome: <strong>{esc(summary['outcome'])}</strong>. "
                f"Verified delivery records: {esc(summary['verified_delivery_records'])}; missing results: {esc(summary['missing_results'])}.</p>",
                "<ul>" + "".join(f"<li>{esc(note)}</li>" for note in report["notes"]) + "</ul>",
                "<h2>Request-to-result timing by kind (seconds)</h2>"]
    sections.append(table(["Kind", "Requests / results", "Median", "Maximum", "Executions"],
        [[esc(kind), esc(f"{v['requests']} / {v['results']}"), esc(v["request_to_result_s"]["median"]), esc(v["request_to_result_s"]["max"]), esc(json.dumps(v["execution_counts"]))] for kind, v in report["timing_by_kind"].items()]))
    sections += ["<h2>Inter-action gaps (seconds)</h2>", f"<pre>{esc(json.dumps(report['inter_action_gap_s'], indent=2))}</pre>", "<h2>Frame ages by camera (milliseconds)</h2>"]
    sections.append(table(["Camera", "Samples", "Median", "Maximum"], [[esc(k), esc(v["count"]), esc(v["median"]), esc(v["max"])] for k, v in report["frame_age_ms_by_camera"].items()]))
    sections.append("<h2>Actions, attempts and evidence</h2>")
    sections.append(table(["Request / result lines", "Action", "Attempt", "Execution / stage", "Latency / preceding gap (s)", "Evidence / errors"], [
        [esc(f"{a['request_line']} / {a['result_line']}"), esc(f"{a['kind']} ({a['action_id']}; {a['correlation']})"), esc(a["attempt"]),
         esc(f"{a['execution']} / {a['result_stage']}"), esc(f"{a['request_to_result_s']} / {a['gap_before_s']}"),
         esc(json.dumps({"request": a["request"], "reason": a["reason"], "observation_error": a["observation_error"], "freshness": a["observation_freshness"]}, ensure_ascii=False))]
        for a in report["actions"]]))
    sections.append("<h2>Event timeline and camera evidence</h2>")
    rows = []
    for event in report["timeline"]:
        evidence = ""
        if "observation_index" in event:
            obs = report["observations"][event["observation_index"]]
            links = []
            for camera, view in obs["views"].items():
                label = esc(f"{camera}: {view['age_ms']} ms ({view['age_source']})")
                safe_path = _image_path(view["path"], directory) if view["path"] else None
                if safe_path and view["exists"]:
                    relative = quote(os.path.relpath(directory / safe_path, output.parent), safe="/")
                    links.append(f'<a href="{esc(relative)}">{label}</a>')
                else:
                    links.append(label + " (image unavailable)")
            evidence = "<br>".join(links)
        elif "action_index" in event:
            a = report["actions"][event["action_index"]]
            evidence = esc(f"{a['kind']} ({a['action_id']})")
        elif event["event"] == "stop":
            evidence = esc(event.get("hold_error"))
        rows.append([esc(event["line"]), esc(event["offset_s"]), esc(event["event"]), esc(event["stage"]), evidence])
    sections.append(table(["Line", "Offset (s)", "Event", "Stage", "Evidence"], rows))
    sections.extend(["<h2>Diagnostics</h2>", "<pre>" + esc(json.dumps(report["diagnostics"], indent=2)) + "</pre>", "</body></html>"])
    return "\n".join(sections)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path, help="Directory containing events.jsonl and saved camera images")
    parser.add_argument("--json", action="store_true", help="Print the versioned report as JSON")
    parser.add_argument("--out", type=Path, help="Write a self-contained HTML report with relative local image links")
    args = parser.parse_args(argv)
    if args.out and args.out.suffix.lower() not in {".html", ".htm"}:
        parser.error("--out must be an .html or .htm file")
    if args.out and args.out.resolve().suffix.lower() not in {".html", ".htm"}:
        parser.error("--out must not resolve to a non-HTML source file")
    try:
        report = build_report(args.run_directory)
        if args.out:
            args.out.write_text(render_html(report, args.out), encoding="utf-8")
    except OSError as exc:
        parser.exit(2, f"replay: {exc}\n")
    if args.json:
        print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
    else:
        summary = report["summary"]
        print(f"Final stage: {summary['final_stage'] or 'unknown'}; outcome: {summary['outcome']}")
        print(f"Requests: {summary['action_requests']}; missing results: {summary['missing_results']}; verified delivery records: {summary['verified_delivery_records']}")
        for kind, data in report["timing_by_kind"].items():
            print(f"  {kind}: {data['requests']} requests, request-to-result seconds {data['request_to_result_s']}")
        print(f"Inter-action gaps (seconds): {report['inter_action_gap_s']}")
        print(f"Diagnostics: {len(report['diagnostics'])}; assessments: {len(report['assessments'])}; explicit recovery requests: {len(report['recoveries'])}")
        if args.out:
            print(f"HTML: {args.out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
