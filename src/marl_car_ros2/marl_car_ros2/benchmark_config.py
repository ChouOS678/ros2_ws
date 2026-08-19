from __future__ import annotations

import json
import os
from typing import Any, Dict


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data if isinstance(data, dict) else {}


def load_benchmark_defaults(pkg_share: str) -> Dict[str, Any]:
    path = os.path.join(pkg_share, "config", "benchmark_defaults.yaml")
    data = _load_yaml(path)
    defaults = data.get("benchmark_defaults", {})
    return defaults if isinstance(defaults, dict) else {}


def load_demo_defaults(pkg_share: str) -> Dict[str, Any]:
    path = os.path.join(pkg_share, "config", "benchmark_defaults.yaml")
    data = _load_yaml(path)
    defaults = data.get("demo_defaults", {})
    return defaults if isinstance(defaults, dict) else {}


def load_scenarios(pkg_share: str) -> Dict[str, Any]:
    path = os.path.join(pkg_share, "config", "baseline_world_scenarios.yaml")
    data = _load_yaml(path)
    scenarios = data.get("scenarios", {})
    return scenarios if isinstance(scenarios, dict) else {}


def resolve_pkg_path(pkg_share: str, path_value: str, *, fallback: str = "") -> str:
    raw = str(path_value or fallback).strip()
    if not raw:
        return ""
    return raw if os.path.isabs(raw) else os.path.join(pkg_share, raw)


def resolve_nav2_params_file(pkg_share: str, planner_profile: str, params_file: str = "") -> str:
    explicit = str(params_file).strip()
    if explicit:
        return explicit if os.path.isabs(explicit) else os.path.join(pkg_share, explicit)
    return os.path.join(pkg_share, "config", "nav2_params.yaml")


def resolve_controller_profile(controller_profile: str, planner_profile: str) -> str:
    explicit = str(controller_profile).strip()
    if explicit:
        return explicit
    aliases = {
        "pp": "PP",
        "pure_pursuit": "PP",
        "pure-pursuit": "PP",
        "app": "APP",
        "adaptive_pure_pursuit": "APP",
        "adaptive-pure-pursuit": "APP",
        "rpp": "RPP",
        "regulated_pure_pursuit": "RPP",
        "regulated-pure-pursuit": "RPP",
        "dwpp": "DWPP",
        "dynamic_window_pure_pursuit": "DWPP",
        "dynamic-window-pure-pursuit": "DWPP",
        "dwb": "DWBLegacy",
        "dwb_legacy": "DWBLegacy",
        "dwb-legacy": "DWBLegacy",
        "followpath": "FollowPath",
    }
    return aliases.get(str(planner_profile).strip().lower(), "PP")
