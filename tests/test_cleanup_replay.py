import json

import pytest

from so101_station.cleanup.replay import build_report, main, render_html


def event(kind, seconds, stage="aligning", **data):
    return dict(event=kind, time_ns=int(seconds * 1e9), run_id="run", stage=stage, **data)


def request(ident, seconds, action="nudge", **data):
    return event("request", seconds, action_id=ident, request=dict(tool="act", action=action, **data))


def result(ident, seconds, execution="completed", stage="aligning", **data):
    return event("result", seconds, stage, action_id=ident, execution=execution, **data)


def report(tmp_path, events):
    (tmp_path / "events.jsonl").write_text("\n".join(json.dumps(e) if isinstance(e, dict) else e for e in events) + "\n")
    return build_report(tmp_path)


def test_timings_gaps_frame_ages_and_freshness_are_separate(tmp_path):
    obs = event("observation", 10, observation_id="obs", revision=0, attempt=0, views={
        "wrist": dict(captured_ns=9_900_000_000, age_ms=90),
        "overhead": dict(captured_ns=9_800_000_000),
    })
    data = report(tmp_path, [obs, request("a", 11, observation_id="obs"),
        result("a", 13, elapsed_s=1.9), request("b", 45, observation_id="obs"), result("b", 49, "rejected", reason="stale_observation")])
    a, b = data["actions"]
    assert a["request_to_result_s"] == 2
    assert a["reported_elapsed_s"] == 1.9
    assert b["request_to_result_s"] == 4 and b["gap_before_s"] == 32
    assert data["timing_by_kind"]["act:nudge"]["request_to_result_s"]["median"] == 3
    assert data["inter_action_gap_s"] == dict(count=1, min=32, median=32, max=32, total=32)
    assert a["observation_freshness"]["status"] == "within_logged_window"
    assert a["observation_freshness"]["frame_age_at_request_ms"]["wrist"] == 1100
    assert b["observation_freshness"]["status"] == "stale"
    assert data["frame_age_ms_by_camera"]["wrist"]["median"] == 90
    assert data["frame_age_ms_by_camera"]["overhead"]["median"] == 200
    assert data["observations"][0]["views"]["overhead"]["age_source"] == "observation_event_timestamp"


def test_pending_stop_malformed_and_orphan_results_survive(tmp_path):
    data = report(tmp_path, [request("pending", 1), '{"unfinished":', '[]',
        event("stop", 2, "stopped", hold_error="telemetry failed"),
        result("orphan", 3, "fault", "stopped", reason="missing request")])
    assert data["summary"]["final_stage"] == "stopped"
    assert data["summary"]["outcome"] == "stopped"
    assert data["summary"]["missing_results"] == 1
    assert data["actions"][0]["execution"] == "missing_result"
    assert data["actions"][0]["result_line"] is None
    assert data["actions"][1]["request_line"] is None
    assert data["stops"][0]["hold_error"] == "telemetry failed"
    assert {d["code"] for d in data["diagnostics"]} == {"malformed_json", "invalid_event", "orphan_result"}
    assert len(data["timeline"]) == 3


def test_duplicate_ids_preserve_requests_and_mark_ambiguous(tmp_path):
    data = report(tmp_path, [request("repeat", 1), request("repeat", 2, "grasp"), result("repeat", 3),
                            result("repeat", 4), result("repeat", 5), request("pending", 6)])
    assert len(data["actions"]) == 4
    assert [a["correlation"] for a in data["actions"][:3]] == ["ambiguous", "ambiguous", "orphan_result"]
    assert data["actions"][0]["result_line"] == 3
    assert data["actions"][1]["result_line"] == 4
    assert data["actions"][1]["gap_before_s"] is None
    assert data["summary"]["missing_results"] == 1
    assert data["summary"]["outcome"] == "incomplete"


def delivered_events():
    return [event("request", 1, "awaiting_placement", action_id="verify", request={"tool": "assess", "outcome": "delivered",
        "source_clear": True, "on_cover": True, "evidence": "paper on cover"}),
        result("verify", 2, stage="complete", attempt=2, detail={"assessment": "delivered", "verified_cleanups": 1})]


def test_verified_delivery_requires_accepted_assessment_and_current_completion(tmp_path):
    data = report(tmp_path, delivered_events())
    assert data["summary"]["outcome"] == "verified_delivery"
    assert data["summary"]["verified_delivery_records"] == 1
    assert data["assessments"][0]["accepted"] is True
    data = report(tmp_path, delivered_events() + [request("new", 3, "begin")])
    assert data["summary"]["outcome"] == "incomplete"
    assert data["summary"]["verified_delivery_records"] == 1
    data = report(tmp_path, [request("release", 1, "release"), result("release", 2, stage="awaiting_placement")])
    assert data["summary"]["outcome"] == "placement_unverified"
    assert data["summary"]["verified_delivery_records"] == 0
    data = report(tmp_path, [result("orphan", 2, stage="complete")])
    assert data["summary"]["outcome"] == "completion_unverified"


def test_failures_retry_and_recovery_have_explicit_evidence(tmp_path):
    data = report(tmp_path, [request("first", 1, "grasp"), result("first", 2, "empty_grasp", "retry_required", attempt=1, reason="empty"),
        request("retry", 3, "grasp"), result("retry", 4, stage="grasped", attempt=2),
        event("stop", 5, "stopped"), request("resume", 6, "resume_holding", params={"operator_confirmed": True, "evidence": "operator confirmed held paper"}),
        result("resume", 7, stage="awaiting_lift", attempt=2, detail={"motion_sent": False})])
    assert [a["pass"] for a in data["grasp_attempts"]] == ["first_pass", "retry"]
    assert len(data["failures"]) == 1
    assert data["recoveries"] == [{"action_index": 2, "execution": "completed", "evidence": "operator confirmed held paper", "accepted": True}]
    assert data["summary"]["outcome"] == "incomplete"


def test_html_is_escaped_local_and_base64_is_omitted(tmp_path):
    image = tmp_path / 'camera "shot".jpg'
    image.write_bytes(b"local image")
    data = report(tmp_path, [event("observation", 0, observation_id="obs", views={
        "<script>alert(1)</script>": dict(path=str(image), age_ms=12),
        "remote": dict(path="https://example.com/image.jpg"),
        "escape": dict(path="../outside.jpg"),
    }, images=[{"b64": "SECRET-IMAGE-BYTES"}]),
        request("<script>alert(2)</script>", 1, params={"evidence": '<img src=x onerror="alert(3)">', "b64": "MORE-SECRET"}),
        result("<script>alert(2)</script>", 2, "fault", reason="<script>alert(4)</script>")])
    html = render_html(data, tmp_path / "report.html")
    assert "<script>" not in html and "<img " not in html
    assert "&lt;script&gt;" in html and "&lt;img" in html
    assert 'href="camera%20%22shot%22.jpg"' in html
    assert "https://example.com" not in html and "../outside.jpg" not in html
    assert "SECRET" not in json.dumps(data) and "SECRET" not in html
    assert sum(d["code"] == "unsafe_image_path" for d in data["diagnostics"]) == 2


def test_negative_or_missing_timestamps_do_not_invent_durations(tmp_path):
    data = report(tmp_path, [request("bad-clock", 5), result("bad-clock", 4),
                            dict(event="request", action_id="no-clock", request={"tool": "act", "action": "open"})])
    assert all(a["request_to_result_s"] is None for a in data["actions"])
    assert {d["code"] for d in data["diagnostics"]} == {"clock_regression", "negative_duration", "invalid_timestamp"}
    assert data["summary"]["missing_results"] == 1


def test_cli_json_and_html_are_offline_and_preserve_log(tmp_path, capsys):
    report(tmp_path, delivered_events())
    before = (tmp_path / "events.jsonl").read_bytes()
    out = tmp_path / "report.html"
    assert main([str(tmp_path), "--json", "--out", str(out)]) == 0
    assert json.loads(capsys.readouterr().out)["schema_version"] == 1
    assert "verified_delivery" in out.read_text()
    assert (tmp_path / "events.jsonl").read_bytes() == before
    with pytest.raises(SystemExit) as exc:
        main([str(tmp_path), "--out", str(tmp_path / "events.jsonl")])
    assert exc.value.code == 2


def test_duplicate_observations_and_changed_revision_are_not_fresh(tmp_path):
    first = event("observation", 1, observation_id="first", revision=0, views={})
    second = event("observation", 2, observation_id="second", revision=1, views={})
    data = report(tmp_path, [first, second, request("stale", 3, observation_id="first"), first,
                            request("ambiguous", 4, observation_id="first")])
    assert data["actions"][0]["observation_freshness"]["status"] == "stale"
    assert data["actions"][1]["observation_freshness"]["status"] == "ambiguous_observation_id"


def test_completed_without_observation_and_stop_do_not_hide_camera_failure(tmp_path):
    data = report(tmp_path, [request("release", 1, "release"), result("release", 2, "completed_without_observation",
                            "awaiting_placement", observation_error="camera disconnected")])
    assert data["summary"]["outcome"] == "placement_unverified"
    assert data["failures"][0]["observation_error"] == "camera disconnected"


def test_stop_invalidates_observation_and_rejected_grasp_is_not_an_attempt(tmp_path):
    data = report(tmp_path, [event("observation", 1, observation_id="obs", revision=0, attempt=2, views={}),
        event("stop", 2, "stopped"), request("denied", 3, "grasp", observation_id="obs"),
        result("denied", 4, "rejected", "stopped", attempt=2, reason="attempt budget exhausted")])
    assert data["actions"][0]["observation_freshness"]["status"] == "stale"
    assert data["grasp_attempts"][0]["pass"] == "not_attempted"
    assert data["grasp_attempts"][0]["attempt_started"] is False


def test_nonfinite_json_and_unsafe_output_symlink_are_rejected(tmp_path):
    data = report(tmp_path, ['{"event":"request","time_ns":1e999}', request("pending", 1)])
    assert data["diagnostics"][0]["code"] == "malformed_json"
    assert data["summary"]["missing_results"] == 1
    target = tmp_path / "report.html"
    target.symlink_to(tmp_path / "events.jsonl")
    before = (tmp_path / "events.jsonl").read_bytes()
    with pytest.raises(SystemExit):
        main([str(tmp_path), "--out", str(target)])
    assert (tmp_path / "events.jsonl").read_bytes() == before
