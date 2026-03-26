from __future__ import annotations

"""
Legacy compatibility entrypoint.

Prefer using `scenario_mutator.py` for all new experiments.
"""

from .scenario_mutator import ScenarioMutator


class WorldModelMutator(ScenarioMutator):
    pass


def main() -> None:
    from .scenario_mutator import main as _main

    _main()


if __name__ == "__main__":
    main()
