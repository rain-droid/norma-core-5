# Remember and locate a selected target

`select_target` and `locate_target` turn a Codex-selected image patch into a reusable target identity. They process the observation already supplied by the caller: no motor commands, new camera requests or task-stage transitions. Both tools are available through session HTTP, CLI and MCP.

Codex still identifies the object and chooses the patch. This feature does not automatically detect trash, associate an object between cameras, estimate depth or produce robot XYZ. A tracked patch is not necessarily the correct grasp point. `robot_xyz_m` stays null and `grasp_ready` stays false.

## Use

After inspecting a fresh `observe` result, select a distinctive patch on the paper, avoiding gripper/background pixels:

```sh
.venv/bin/python scripts/visual_alignment.py select \
  --observation-id OBSERVATION_ID --target-id paper-1 \
  --label crumpled_paper --view wrist --bbox X Y WIDTH HEIGHT
```

For a later freshly inspected observation:

```sh
.venv/bin/python scripts/visual_alignment.py locate \
  --observation-id NEW_OBSERVATION_ID --target-id paper-1
```

Results identify the exact source observation, camera, resolution, timestamp and image path. Coordinates are pixels in that source image. `center_pixel` is the tracked patch center; `patch_bbox` describes the patch rather than an automatically segmented object boundary. `normalized_correlation` is an image-match score, not a probability of object identity. The label comes from Codex's selection.

| Status | Meaning | Next step |
| --- | --- | --- |
| selected | Explicitly selected a new patch | Inspect later frames and locate it |
| tracked | Patch matched within the local search | Use this observation's image position |
| shifted | Patch shifted by more than 5 px since its last match | Inspect the change before deciding motion |
| lost | No reliable local match | Inspect a new image and explicitly reselect |
| stale | Selection exceeded its 300 s lifetime | Explicitly select again |
| view_changed | Camera identity or resolution changed | Inspect and explicitly select again |

A wrist-image shift can be caused by the moving camera. It is not evidence that the object moved on the table. Search stays bounded; an occluded or distant target is not silently replaced by another similar object. Lost/stale/changed-view states stay latched until reselection. The last 64 processed observation IDs are cached for idempotent retries; older or equal timestamps under a different observation ID are rejected. Up to sixteen targets can be held per process; restart clears them all.

## Feed a wrist target into probing

Once the target has been selected or located in the exact observation being used for the next decision, the operator can pass its ID directly:

```sh
.venv/bin/python scripts/visual_alignment.py probe \
  --observation-id NEW_OBSERVATION_ID --target-id paper-1 --scene-clear
```

Provide either `--target-id` or `--bbox`. A target ID uses its stored tracker choice and current wrist patch. Overhead targets cannot feed wrist probing without a separate wrist selection; no cross-camera transform is assumed. The existing clearance, pose, stable-frame, stop and fit checks still apply.

The resulting probe records the target ID and selection generation. Reselecting that ID, losing it, or letting it expire invalidates that probe for alignment. Tracking success alone never enables grasp/carry or clears a controller fault. Independent image inspection and the existing task gates remain required.

## Ownership and validation

Claude currently owns the physical arm. Codex's motion service is off and stays off until the user hands ownership back. These commands document the next coordinated run, not permission to start a competing controller. Offline tests use synthetic images and an in-process mock service; physical acquisition and pickup remain unvalidated.
