from __future__ import annotations

import json
import math
import sqlite3
import statistics
from collections import defaultdict
from typing import Any
from urllib.parse import unquote


CONTROLLERS = {
    "pp": {"name": "PP", "label": "Pure Pursuit", "description": "几何追踪基线：实现简单、计算开销低；固定前视距离在速度与弯道精度之间存在取舍。"},
    "app": {"name": "APP", "label": "Adaptive Pure Pursuit", "description": "自适应 Pure Pursuit 配置：适合研究前视参数随工况调整的收益；实际效果依赖本项目插件实现与参数。"},
    "rpp": {"name": "RPP", "label": "Regulated Pure Pursuit", "description": "Nav2 Regulated Pure Pursuit 配置：在几何追踪外调节速度；应同时观察误差、速度和任务完成情况。"},
    "dwpp": {"name": "DWPP", "label": "Dynamic Window PP", "description": "项目 DWPP controller profile；具体决策行为以当前插件实现及配置为准，不预设理论优势。"},
    "unknown": {"name": "未知", "label": "未识别 profile", "description": "报告未包含可识别的 controller profile。"},
}

METRIC_LABELS = {
    "mean_tracking_error_m": ("平均跟踪误差", "m", "lower"),
    "average_tracking_error_m": ("平均跟踪误差", "m", "lower"),
    "max_tracking_error_m": ("最大跟踪误差", "m", "lower"),
    "max_overshoot_m": ("最大偏离", "m", "lower"),
    "average_speed_mps": ("平均速度", "m/s", "higher"),
    "mission_duration_s": ("任务耗时", "s", "lower"),
    "min_obstacle_clearance_m": ("最小障碍间距", "m", "higher"),
    "mean_lateral_offset_m": ("平均横向偏移", "m", "lower"),
    "command_constraint_compliance_rate": ("控制约束符合率", "%", "higher"),
    "turn_completion_time_s": ("转弯耗时", "s", "lower"),
    "speed_std_mps": ("速度波动（遥测）", "m/s", "lower"),
    "angular_speed_rms_radps": ("角速度 RMS（遥测）", "rad/s", "lower"),
    "min_clearance_observed_m": ("最小观测间距（遥测）", "m", "higher"),
    "telemetry_mean_speed_mps": ("遥测平均速度", "m/s", "higher"),
    "distance_travelled_m": ("行驶距离（遥测）", "m", "lower"),
}


def _decode(value: str) -> dict[str, Any]:
    try:
        output = json.loads(value)
        return output if isinstance(output, dict) else {}
    except (TypeError, json.JSONDecodeError):
        return {}


def _telemetry_metrics(db: sqlite3.Connection, run_id: str) -> dict[str, Any]:
    rows = db.execute(
        "SELECT sim_time, x, y, linear_speed, angular_speed, min_range FROM telemetry_samples WHERE run_id=? ORDER BY sample_index",
        (run_id,),
    ).fetchall()
    linear = [float(row["linear_speed"]) for row in rows if row["linear_speed"] is not None]
    angular = [float(row["angular_speed"]) for row in rows if row["angular_speed"] is not None]
    clearance = [float(row["min_range"]) for row in rows if row["min_range"] is not None and float(row["min_range"]) >= 0]
    result: dict[str, Any] = {"telemetry_sample_count": len(rows)}
    if linear:
        result["speed_std_mps"] = statistics.pstdev(linear)
        result["telemetry_mean_speed_mps"] = statistics.fmean(abs(value) for value in linear)
    if angular:
        result["angular_speed_rms_radps"] = math.sqrt(statistics.fmean(value * value for value in angular))
    if clearance:
        result["min_clearance_observed_m"] = min(clearance)
    points = [(float(row["x"]), float(row["y"])) for row in rows if row["x"] is not None and row["y"] is not None]
    if len(points) > 1:
        result["distance_travelled_m"] = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    return result


def _mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def _campaign_for(source_name: str) -> str:
    segments = unquote(str(source_name)).replace("\\", "/").split("/")
    return next((segment for segment in segments if segment.startswith("campaign_")), "historical")


def overview(db: sqlite3.Connection, campaign_id: str | None = None) -> dict[str, Any]:
    all_records = db.execute("SELECT * FROM runs ORDER BY imported_at DESC, run_id").fetchall()
    available_campaigns = {_campaign_for(record["source_name"]) for record in all_records}
    campaign_ids = sorted((item for item in available_campaigns if item != "historical"), reverse=True)
    if "historical" in available_campaigns:
        campaign_ids.append("historical")
    if campaign_id is None or (campaign_id not in campaign_ids and campaign_id != "all"):
        campaign_id = next((item for item in campaign_ids if item != "historical"), "historical" if campaign_ids else "all")
    records = all_records if campaign_id == "all" else [record for record in all_records if _campaign_for(record["source_name"]) == campaign_id]
    runs: list[dict[str, Any]] = []
    by_case: dict[str, dict[str, list[dict[str, Any]]]] = defaultdict(lambda: defaultdict(list))
    metric_names: set[str] = set()
    for record in records:
        primary = _decode(record["primary_metrics_json"])
        supporting = _decode(record["supporting_metrics_json"])
        telemetry = _telemetry_metrics(db, record["run_id"])
        metrics = {**primary, **supporting, **telemetry}
        if record["mission_duration_s"] is not None:
            metrics["mission_duration_s"] = record["mission_duration_s"]
        metric_names.update(key for key, value in metrics.items() if key in METRIC_LABELS and isinstance(value, (int, float)) and not isinstance(value, bool))
        item = {
            "run_id": record["run_id"], "scenario": record["scenario"], "controller": record["controller"],
            "campaign_id": _campaign_for(record["source_name"]),
            "status": record["status"], "stop_reason": record["stop_reason"],
            "mission_duration_s": record["mission_duration_s"], "metrics": metrics,
            "timeline_metrics": _decode(record["timeline_metrics_json"]), "source_name": record["source_name"],
            "imported_at": record["imported_at"],
        }
        runs.append(item)
        by_case[item["scenario"]][item["controller"]].append(item)

    cases = []
    for scenario, groups in sorted(by_case.items()):
        algorithms = []
        for controller, items in sorted(groups.items()):
            successful = [item for item in items if item["status"] == "success"]
            keys = sorted({key for item in successful for key, value in item["metrics"].items() if key in METRIC_LABELS and isinstance(value, (int, float)) and not isinstance(value, bool)})
            averages = {key: _mean([float(item["metrics"][key]) for item in successful if isinstance(item["metrics"].get(key), (int, float)) and not isinstance(item["metrics"].get(key), bool)]) for key in keys}
            stddev = {key: statistics.pstdev([float(item["metrics"][key]) for item in successful if isinstance(item["metrics"].get(key), (int, float)) and not isinstance(item["metrics"].get(key), bool)]) for key in keys if sum(isinstance(item["metrics"].get(key), (int, float)) and not isinstance(item["metrics"].get(key), bool) for item in successful) > 1}
            algorithms.append({
                **CONTROLLERS.get(controller, CONTROLLERS["unknown"]), "controller": controller,
                "run_count": len(items), "success_count": len(successful),
                "completion_rate": len(successful) / len(items) if items else 0.0,
                "averages": averages, "stddev": stddev,
                "repeat_ready": len(successful) >= 3,
            })
        cases.append({"scenario": scenario, "run_count": sum(len(items) for items in groups.values()), "algorithms": algorithms})

    return {
        "runs": runs,
        "cases": cases,
        "campaigns": [{"id": item, "label": "既有历史数据" if item == "historical" else item} for item in campaign_ids] + ([{"id": "all", "label": "全部批次（混合对比）"}] if len(campaign_ids) > 1 else []),
        "active_campaign": campaign_id,
        "controllers": CONTROLLERS,
        "metric_labels": {key: {"label": value[0], "unit": value[1], "direction": value[2]} for key, value in METRIC_LABELS.items()},
        "summary": {
            "run_count": len(runs), "overall_run_count": len(all_records), "scenario_count": len(by_case),
            "controller_count": len({record["controller"] for record in records}),
            "success_count": sum(record["status"] == "success" for record in records),
            "telemetry_run_count": sum(bool(_telemetry_metrics(db, record["run_id"]).get("telemetry_sample_count")) for record in records),
            "available_metrics": sorted(metric_names),
        },
    }


def explain_case(case: dict[str, Any], metric_labels: dict[str, Any]) -> list[str]:
    algorithms = case["algorithms"]
    explanations = []
    repeats = [item for item in algorithms if item["repeat_ready"]]
    if not repeats:
        explanations.append("重复实验门槛尚未达到：至少需要同一场景、同一算法 3 次成功运行；当前不宜下结论。");
    for metric_key in ("mean_tracking_error_m", "average_tracking_error_m", "max_tracking_error_m", "average_speed_mps", "mission_duration_s"):
        candidates = [(item, item["averages"].get(metric_key)) for item in repeats if item["averages"].get(metric_key) is not None]
        if len(candidates) < 2:
            continue
        direction = metric_labels.get(metric_key, {}).get("direction", "lower")
        candidates.sort(key=lambda pair: pair[1], reverse=(direction == "higher"))
        best, worst = candidates[0], candidates[-1]
        if best[0]["controller"] == worst[0]["controller"] or abs(best[1] - worst[1]) < 1e-9:
            continue
        label = metric_labels.get(metric_key, {}).get("label", metric_key)
        unit = metric_labels.get(metric_key, {}).get("unit", "")
        change = abs(best[1] - worst[1]) / max(abs(worst[1]), 1e-9) * 100
        explanations.append(f"{best[0]['name']} 在成功运行的平均{label}上优于 {worst[0]['name']}，约 {change:.1f}%（{best[1]:.4g} vs {worst[1]:.4g} {unit}）；这是当前样本的关联观察，不代表算法因果优势。")
    for item in algorithms:
        if item["run_count"] and item["success_count"] < item["run_count"]:
            explanations.append(f"{item['name']} 有 {item['run_count'] - item['success_count']} 次未成功运行；失败运行保留在完成率统计中，数值指标仅统计成功运行。")
    if not explanations:
        explanations.append("当前样本不足以支持跨算法优势结论；可查看各次运行、波动范围和完成率。")
    return explanations
