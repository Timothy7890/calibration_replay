#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration_replay.importer import import_session
from calibration_replay.storage import PlanStore


def main() -> None:
    parser = argparse.ArgumentParser(description="Import selected solved 2D sessions")
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--base-url-2d", default="http://127.0.0.1:18005")
    parser.add_argument("--head-session")
    parser.add_argument("--waist-session")
    args = parser.parse_args()
    sessions = {
        "hand_eye_2D_head": args.head_session,
        "hand_eye_2D_waist": args.waist_session,
    }
    if not any(sessions.values()):
        parser.error("请至少指定 --head-session 或 --waist-session")
    store = PlanStore(args.data_root)
    imported = {
        plan.metadata.get("imported_from") for plan in store.list() if plan.metadata
    }
    created = []
    for target, session in sessions.items():
        if not session:
            continue
        if str(Path(session).expanduser().resolve()) in imported:
            continue
        label = "Imported 2D head" if target.endswith("head") else "Imported 2D waist"
        plan = import_session(session, target=target, name=label, base_url=args.base_url_2d)
        store.save(plan)
        created.append(plan)
    for plan in created:
        samples = sum(node.role == "sample" for node in plan.nodes)
        transits = sum(node.role == "transit" for node in plan.nodes)
        print(
            f"{plan.id}  {plan.name}  "
            f"{samples} capture samples + {transits} transit nodes (draft)"
        )
    if not created:
        print("No plans created; selected sessions were already imported.")


if __name__ == "__main__":
    main()
