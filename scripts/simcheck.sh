#!/bin/sh
# Run every tool once against a simulated daemon on port 8766.
call() { ELROBOT_PORT=8766 uv run elrobot call "$@" 2>&1 | head -2; }
call status; call home
for g in nod wave dance sad bow happy oops celebrate shrug curious peek shake dap; do call gesture "{\"name\":\"$g\"}"; done
call observe
call pick '{"x":0.05,"y":0.25}'
call pick '{"x":-0.12,"y":0.30}'
call pick '{"x":0.0,"y":0.15}'
