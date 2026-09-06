#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration_replay.importer import seed_default_imports
from calibration_replay.storage import PlanStore


def main() -> None:
    parser = argparse.ArgumentParser(description="Seed head/waist plans from solved 2D sessions")
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--base-url-2d", default="http://127.0.0.1:8131")
    args = parser.parse_args()
    created = seed_default_imports(PlanStore(args.data_root), args.base_url_2d)
    for plan in created:
        samples = sum(node.role == "sample" for node in plan.nodes)
        transits = sum(node.role == "transit" for node in plan.nodes)
        print(
            f"{plan.id}  {plan.name}  "
            f"{samples} capture samples + {transits} transit nodes (draft)"
        )
    if not created:
        print("No plans created; both source sessions were already imported.")


if __name__ == "__main__":
    main()
