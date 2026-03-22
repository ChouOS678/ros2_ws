# AGENTS.md

## Current project status
- This project is currently a research platform skeleton, not yet a fully completed benchmark platform.
- Do not assume planned features are already implemented.
- The active Nav2 controller may still be DWB unless the repository clearly shows otherwise.
- Frontend experiment presentation may still be incomplete:
  - robot model
  - Gazebo/RViz visibility
  - baseline worlds
- Do not describe planned work as already complete.

## Recent engineering updates (2026-03-22)
- Nav2 controller profile switching is implemented in `evaluation.launch.py`:
  - `planner_profile:=rpp` maps to `config/nav2_params_rpp.yaml`
  - default/other profiles map to `config/nav2_params.yaml` (DWB baseline path)
  - optional explicit `params_file` override is supported
- RPP profile file is present (`config/nav2_params_rpp.yaml`) and uses
  `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.
- Minimal compatibility fixes already applied in Nav2 params:
  - behavior plugin naming aligned to `nav2_behaviors::...`
  - `collision_monitor` minimal required parameters added
  - minimal docking plugin section added for `docking_server`
  - costmap size numeric typing corrected for startup compatibility
- TF and visualization stabilization updates:
  - `nav_tf_bridge` is launched in `agent_nav2.launch.py`
  - `nav_tf_bridge_node` supports stable configured-frame publishing
    (`use_msg_frame_ids`, `use_msg_stamp`)
  - RViz default fixed frame/target frame moved to `odom` in
    `rviz/nav2_default.rviz` to reduce `Message Filter queue full` under odom-mode
- Current global-frame mode in local experiments has been set to odom-centric
  (`bt_navigator` / `global_costmap` using `odom`) for stack continuity when
  no stable `map->odom` chain exists.

## Current validation status (as observed, not ideal target)
- Full chain process startup is available (Gazebo bridge + Nav2 + supervisor + monitor + mutator),
  but lifecycle stability is not yet consistently clean.
- In recent comparative runs (DWB x3, RPP x3), runs were mostly recorded as
  `mission_completion_status=degraded`, with starvation risk recurring and
  limited evidence of effective navigation progression.
- Therefore, the platform is suitable for integration/debug pre-experiments,
  but not yet ready for formal algorithm benchmark claims.
- Recommended gate before formal algorithm experiments:
  - 3 consecutive runs with no Nav2 lifecycle transition failures
  - 3 consecutive runs with non-degraded mission completion
  - repeatable comparable navigation outcomes (not only safety override dominance)

## Module responsibilities
- supervisor: final /cmd_vel arbitration and safety gating
- task_agent / mode_manager: high-level mode decision only
- baseline_nav_node: baseline / fallback only
- monitor_logger: experiment logging and timeline capture
- scenario_mutator / evaluation: experiment setup and reproducible benchmarking

## Documentation rules
- Clearly distinguish implemented, partially implemented, and planned work.
- If code and docs disagree, prefer the code state.
- Avoid marketing-style wording; use engineering-status wording.

## Frontend experiment rules
- Preserve or improve Gazebo/RViz visibility of robot, paths, TF, and costmaps.
- Prefer lightweight, reproducible experiment assets over decorative complexity.

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
