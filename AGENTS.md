# AGENTS.md

## Repository expectations
- This repository is a ROS 2 + Nav2 research project.
- Prefer minimal, incremental changes over broad refactors.
- Do not rename or move files unless explicitly requested.
- Do not change launch behavior, topic names, or message contracts unless the task explicitly asks for it.
- Keep backward compatibility with the current simulation pipeline when possible.

## Architecture rules
- Keep the supervisor as the final /cmd_vel authority.
- Treat baseline_nav_node as baseline/fallback only, not the long-term primary path.
- New navigation intelligence should prefer Nav2-native extension points:
  - controller plugin
  - costmap layer plugin
  - BT plugin / supervisor logic
- Keep experiment modules decoupled from runtime modules.

## Safety rules
- Never remove safety checks.
- Never bypass collision-related logic without explicit approval.
- Preserve deterministic fallback behavior.

## Coding rules
- First read relevant files and summarize your understanding before editing.
- Then propose a step-by-step plan.
- Then wait for approval if the requested change is large.
- Prefer small commits and minimal diffs.
- Add comments only where they clarify non-obvious logic.

## Validation rules
- After changes, run the narrowest possible validation first.
- If tests exist, run the smallest relevant subset first, then broader checks if needed.
- Report:
  1. files changed
  2. commands run
  3. results
  4. remaining risks / TODOs

## Forbidden actions
- Do not rewrite unrelated files.
- Do not silently revert user changes.
- Do not introduce placeholder TODO implementations unless explicitly requested.