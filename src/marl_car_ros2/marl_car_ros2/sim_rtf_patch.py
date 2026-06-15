"""Dynamic world-file RTF patching for Gazebo Sim benchmarks.

Creates temporary copies of world files with modified <real_time_factor>
so that benchmark runs can be accelerated without touching the original
world definitions.
"""

import hashlib
import os
import re


def patched_world_path(world_file: str, rtf_str: str) -> str:
    """Return path to a world file with the requested real_time_factor.

    If rtf_str ≈ 1.0, returns the original path unchanged.
    Otherwise writes a patched copy to /tmp/marl_rtf_<hash>.world
    and returns the temporary path.

    Args:
        world_file: Absolute or relative path to the .world SDF file.
        rtf_str: Real-time factor as a string (e.g. "1.0", "2.5").

    Returns:
        Path to the world file to use (original or patched temp copy).
    """
    try:
        rtf = float(rtf_str)
    except (ValueError, TypeError):
        return world_file

    # No-op for real-time — avoid unnecessary I/O.
    if abs(rtf - 1.0) < 0.001:
        return world_file

    # Deterministic temp path so repeated runs reuse the same file.
    key = hashlib.md5(f"{world_file}_{rtf}".encode()).hexdigest()[:8]
    patched = f"/tmp/marl_rtf_{key}.world"

    if os.path.exists(patched):
        return patched

    if not os.path.exists(world_file):
        return world_file

    with open(world_file, "r") as f:
        content = f.read()

    # Replace the first <real_time_factor> element.
    content = re.sub(
        r"<real_time_factor>[^<]*</real_time_factor>",
        f"<real_time_factor>{rtf}</real_time_factor>",
        content,
        count=1,
    )

    with open(patched, "w") as f:
        f.write(content)

    return patched
