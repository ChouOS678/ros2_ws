# AGENTS.md

## Current project status
- This repository is a ROS 2 + Nav2 benchmark platform with a converged dual-path structure:
  - formal benchmark backbone
  - manual demo / debug chain
- The formal benchmark backbone is:
  - `evaluation.launch.py`
  - `monitor_logger`
  - `evaluation_metrics`
- The manual demo / debug chain is:
  - `benchmark_demo.launch.py`
  - `benchmark_gui`
  - `benchmark_visualizer`
- Do not assume planned features beyond what the repository currently shows.
- Do not describe demo/debug tooling as the formal benchmark result path.

## Recent engineering updates (2026-03-26)
- Benchmark entrypoints were converged to exactly two public paths:
  - formal: `evaluation.launch.py`
  - manual demo/debug: `benchmark_demo.launch.py`
- Historical duplicate launch entrypoints were removed:
  - `marl_stack.launch.py`
  - `marl_stack_minimal.launch.py`
  - `nav2_visualization.launch.py`
  - `baseline_world_narrow.launch.py`
  - `baseline_world_turns.launch.py`
  - `baseline_world_dynamic.launch.py`
- A single benchmark default parameter source was added:
  - `config/benchmark_defaults.yaml`
- Shared benchmark/demo config loading was centralized in:
  - `marl_car_ros2/benchmark_config.py`
- Scenario registration remains centralized in:
  - `config/baseline_world_scenarios.yaml`
- Scenario categories are now explicitly organized as:
  - high curvature / sharp turns
  - narrow passage
  - dynamic obstacle extension
- `evaluation.launch.py` remains the only formal scenario-based benchmark entrypoint.
- `benchmark_demo.launch.py` is explicitly a debug/demo tool entrypoint and is not the formal metrics source.
- `wsl2_demo_ctl.sh` now launches `benchmark_demo.launch.py`.
- `auto_eval_pipeline.py` was removed as a historical coupled path superseded by the converged structure.

## Benchmark configuration rules
- Use `config/benchmark_defaults.yaml` as the single default parameter source for:
  - default benchmark scenario
  - planner/controller profile defaults
  - default world/spawn/goal values
  - default dynamic obstacle parameters
  - default benchmark-mode semantics
- Use `config/baseline_world_scenarios.yaml` as the single scenario registration source.
- If default values differ between files, treat `benchmark_defaults.yaml` and `baseline_world_scenarios.yaml` as the source of truth, then fix the launch code or docs.
- All planner profiles use config/nav2_params.yaml; planner_profile:=rpp selects the RPP controller ID.
- default/other profiles still map to `config/nav2_params.yaml`.
- optional explicit `params_file` override is still supported.

## Current validation status
- The formal benchmark backbone is available for scenario-based evaluation runs and result generation.
- The manual demo/debug chain is available for interactive verification, visualization, and integration debugging.
- Lifecycle stability and run quality should still be validated before making strong comparative claims.
- Recommended gate for formal comparison campaigns:
  - 3 consecutive runs with no Nav2 lifecycle transition failures
  - 3 consecutive runs with non-degraded mission completion
  - repeatable comparable navigation outcomes across the same scenario/profile inputs

## Module responsibilities
- supervisor: final /cmd_vel arbitration and safety gating
- task_agent / mode_manager: high-level mode decision only
- baseline_nav_node: baseline / fallback only
- monitor_logger: formal benchmark logging and timeline capture
- evaluation_metrics: formal benchmark result computation
- benchmark_gui / benchmark_visualizer: manual demo/debug tooling only
- scenario_mutator / evaluation: experiment setup and reproducible scenario benchmarking

## Documentation rules
- Clearly distinguish implemented, partially implemented, and planned work.
- If code and docs disagree, prefer the code state.
- Use engineering-status wording, not marketing wording.
- Describe the formal benchmark backbone and the manual demo/debug chain separately.
- Do not describe deleted historical entrypoints as still supported.

## Frontend experiment rules
- Preserve or improve Gazebo/RViz visibility of robot, paths, TF, and costmaps.
- Prefer lightweight, reproducible experiment assets over decorative complexity.
- Keep demo/debug frontend tooling out of formal benchmark semantics.

## Repository expectations
- This repository is a ROS 2 + Nav2 benchmark research project.
- Prefer minimal, incremental changes over broad refactors.
- Do not rename or move files unless explicitly requested.
- Do not change launch behavior, topic names, or message contracts unless the task explicitly asks for it.
- Keep backward compatibility with the current simulation pipeline when possible.

## Architecture rules
- Keep the supervisor as the final /cmd_vel authority.
- Treat baseline_nav_node as baseline/fallback only, not the long-term primary path.
- Keep `evaluation.launch.py` as the only formal benchmark entrypoint.
- Keep `benchmark_demo.launch.py` as demo/debug only.
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
