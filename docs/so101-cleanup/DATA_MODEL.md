# Contracts

Observations contain observation_id, camera_id, image_size, captured_ns, image_path, state, busy and motion epoch. JPEGs and events.jsonl are kept in the configured log directory. MCP returns native images; CLI returns absolute paths.

plan_pick accepts observation_id, label (crumpled_paper/snack_wrapper), confidence, grasp_pixel [u,v] and scene_clear. Plans include start angles, Cartesian targets and checked joint paths, expire after 60 seconds, are tied to an epoch and are single-use.

execute_plan returns run_id, contact estimate, before/after observation ids and awaiting_visual_verification. verify requires run_id, a post-action observation_id, source_clear, trash_in_drop_zone and evidence. Only both checks true count as verified_cleaned; otherwise needs_inspection. Verification cannot be repeated.

Workspace JSON defines fixed camera identity/resolution, >=6 pixel/table pairs, a convex polygon, drop_xy_m and table/grasp/travel/drop heights. Robot distances are metres, angles radians, image coordinates pixels. Real startup refuses sim_only profiles. The supplied template is intentionally incomplete.
