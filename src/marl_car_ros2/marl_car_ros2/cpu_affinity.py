from __future__ import annotations

import json
import os
import shutil
from typing import Any, Dict, Optional

from launch import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data if isinstance(data, dict) else {}


def load_affinity_config(pkg_share: str) -> Dict[str, Any]:
    """Load CPU affinity configuration from config/cpu_affinity.yaml."""
    path = os.path.join(pkg_share, "config", "cpu_affinity.yaml")
    data = _load_yaml(path)
    affinity = data.get("cpu_affinity", {})
    return affinity if isinstance(affinity, dict) else {}


def _has_taskset() -> bool:
    """Check if taskset command is available on the system."""
    return shutil.which("taskset") is not None


def get_taskset_prefix(
    node_name: str,
    pkg_share: str,
    *,
    use_affinity: bool = False,
) -> str:
    """
    Return a taskset prefix string for the given node name, or empty if
    affinity is disabled, node is not configured, or taskset is unavailable.

    Returns a string like "taskset -c 0,1" or "".
    """
    if not use_affinity:
        return ""
    if not _has_taskset():
        return ""
    config = load_affinity_config(pkg_share)
    enabled = bool(config.get("enabled", False))
    if not enabled:
        return ""
    affinity_map = config.get("affinity_map", {})
    if not isinstance(affinity_map, dict):
        return ""
    cores = affinity_map.get(node_name, "")
    if not cores or not str(cores).strip():
        return ""
    return f"taskset -c {cores}"


class CpuAffinityPrefix(Substitution):
    """
    A Launch Substitution that resolves to a taskset prefix string.

    Usage in a launch file::

        Node(
            ...,
            prefix=CpuAffinityPrefix("controller_server", pkg_share, use_affinity_subst),
        )
    """

    def __init__(
        self,
        node_name: str,
        pkg_share: Substitution,
        use_affinity: Substitution,
    ):
        super().__init__()
        self.__node_name = node_name
        self.__pkg_share = normalize_to_list_of_substitutions(pkg_share)
        self.__use_affinity = normalize_to_list_of_substitutions(use_affinity)

    def describe(self) -> str:
        return f"CpuAffinityPrefix({self.__node_name})"

    def perform(self, context) -> str:
        pkg_share_str = perform_substitutions(context, self.__pkg_share)
        use_affinity_str = perform_substitutions(context, self.__use_affinity)
        use_affinity_bool = use_affinity_str.strip().lower() in ("true", "1", "yes")
        return get_taskset_prefix(
            self.__node_name,
            pkg_share_str,
            use_affinity=use_affinity_bool,
        )
