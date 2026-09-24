# Benchmark Observatory

Independent local web application for importing ROS 2 benchmark run reports, associating monitor telemetry, and comparing PP-family controller profiles by trajectory scenario. It does not alter the ROS packages, launch files, benchmark behavior, or result-generation pipeline.

## Run

From the `ros2_ws` root in WSL:

```bash
cd /home/grok/ros2_ws
python3 -m benchmark_portal.backend
```

Open `http://localhost:8765`. The application uses only the Python standard library and stores its normalized data in `benchmark_portal/data/benchmark.sqlite`.

On startup it looks for `acceptance_*.json` and `last_run*report.json` in the repository root. It deliberately does not auto-import historical `br_*.json` world-benchmark outputs. It attempts to associate telemetry from `/tmp/marl_logs/timeline.db` using the run ID stored in each telemetry payload; telemetry-only runs are also recorded if a matching report has not been generated yet. Override locations with `BENCHMARK_REPORT_DIR`, `BENCHMARK_TELEMETRY_DB`, and `BENCHMARK_PORTAL_DB`. The UI can import additional JSON reports and re-sync telemetry on demand.

## Data and interpretation

- Report metrics are preserved as reported; raw telemetry is independently copied into the portal database and keyed by run ID.
- Telemetry-derived values are explicitly labeled and include speed variability, angular-speed RMS, observed minimum range, and traveled distance.
- Comparison aggregates use successful runs only for numeric means; completion rate includes failed and unknown runs. The run log keeps failures visible.
- Campaigns remain separated from historical runs; the dashboard defaults to the newest campaign and allows an explicit historical or combined view.
- A comparison is marked repeat-ready at three successful runs for one controller and one scenario. Explanations describe observed metric differences and state that these observations do not establish causality.
- Controller descriptions are explanatory context, not claims that the profile must outperform another controller.

## Tests

```bash
cd /home/grok/ros2_ws/benchmark_portal
python3 -m unittest discover -s tests -v
```

## Run a complete comparison campaign

From the `ros2_ws` root, after sourcing ROS and the workspace overlay, run:

```bash
python3 -m benchmark_portal.collect_campaign
```

This runs the five registered trajectories against `pp`, `app`, `rpp`, and `dwpp` (20 runs total), uses the monitor's configured `/tmp/marl_logs` directory one run at a time, archives that directory after each run, and immediately imports the matching report and telemetry into the portal database. Existing `/tmp/marl_logs` contents are copied into the campaign archive before the runner's clean-start behavior. Reports, logs, telemetry archives, and a campaign summary are written beneath `benchmark_portal/data/campaigns/`.
