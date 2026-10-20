"""CLI of the brain: ``python -m software.brain --strategy <name|path>``.

Examples::

    python -m software.brain --strategy eurobot_main --transport mock
    python -m software.brain --strategy strategies/qualif.json --repeat
    python -m software.brain --list

Environment fallbacks: ``STRATEGY`` (name or path), ``ROBOT_TRANSPORT``.

File:    software/brain/__main__.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
from dataclasses import replace
from pathlib import Path

from software.config import settings
from software.core.mission.strategy import Strategy
from software.core.mission.teams import mirror_strategy

from .runner import Brain


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    """Parse the command line arguments."""
    parser = argparse.ArgumentParser(
        prog="software.brain",
        description="Play a saved strategy on the robot - the RPi's main job.",
    )
    parser.add_argument(
        "--strategy",
        default=os.environ.get("STRATEGY", ""),
        help="strategy name (in STRATEGY_DIR) or path to a .json file",
    )
    parser.add_argument("--dir", default=settings.strategy_dir, help="strategies directory")
    parser.add_argument(
        "--transport",
        default=settings.transport,
        choices=("i2c", "mock"),
        help="robot link: real board or simulator",
    )
    parser.add_argument("--repeat", action="store_true", help="replay the strategy in a loop")
    parser.add_argument(
        "--team",
        default=settings.team,
        choices=("yellow", "blue"),
        help="which side we play; a strategy authored for the other side is mirrored",
    )
    parser.add_argument("--plan-only", action="store_true", help="plan and print, do not move ('elements' documents)")
    parser.add_argument("--list", action="store_true", help="list saved strategies and exit")
    return parser.parse_args(argv)


def _load_strategy(name_or_path: str, directory: str) -> Strategy:
    """Load a strategy by file path or by name from the strategy directory."""
    if name_or_path.endswith(".json") or os.sep in name_or_path or "/" in name_or_path:
        return Strategy.load(Path(name_or_path))
    return Strategy.load_named(directory, name_or_path)


async def _run(args: argparse.Namespace) -> int:
    """Start the brain, play the requested strategy and stop it again."""
    config = replace(settings, transport=args.transport, strategy_dir=args.dir, team=args.team)
    try:
        strategy = _load_strategy(args.strategy, args.dir)
    except (ValueError, OSError) as exc:
        print(f"cannot load strategy '{args.strategy}': {exc}", file=sys.stderr)
        rows = Strategy.list_saved(args.dir)
        if rows:
            print("available strategies:", file=sys.stderr)
            for row in rows:
                print(f"  - {row['name']}", file=sys.stderr)
        return 2
    if strategy.team and strategy.team != config.team:
        # the table is symmetric: play the same strategy mirrored for our side
        mirror_strategy(strategy, config.field_width_mm)
        note = " (v1 geometry only - the TREE is played as authored)" if strategy.kind == "bt" else ""
        print(
            f"strategy '{strategy.name}' was authored for the {strategy.team} side - "
            f"mirrored for {config.team}{note}"
        )

    brain = Brain(config)
    await brain.start()
    try:
        if args.plan_only:
            if strategy.kind == "bt":
                print(
                    "--plan-only needs an 'elements' document: a behavior tree plans "
                    "inside its run_document leaves when it is played",
                    file=sys.stderr,
                )
                return 2
            from software.core.mission.orchestrator import execute_plan

            try:
                result = await execute_plan(
                    brain.ctx, strategy.elements, strategy.goal, strategy.obstacles, strategy.options
                )
            except ValueError as exc:
                print(f"cannot plan strategy '{strategy.name}': {exc}", file=sys.stderr)
                return 2
            stats = result["stats"]
            print(
                f"plan: {stats['n_segments']} segments, {stats['length_mm'] / 1000.0:.2f} m, "
                f"~{stats['est_s']:.0f} s"
            )
            return 0
        final = await brain.play(strategy, repeat=args.repeat)
        return 0 if final == "done" else 1
    finally:
        await brain.stop()


def main(argv: list[str] | None = None) -> int:
    """CLI entry point: list strategies, plan only, or play."""
    args = _parse_args(argv)
    if args.list:
        rows = Strategy.list_saved(args.dir)
        if not rows:
            print(f"no strategies in {args.dir}")
        for row in rows:
            print(f"{row['name']:<32} {row['n_elements']:>3} elements  {row['notes']}")
        return 0
    if not args.strategy:
        print("no strategy given: use --strategy <name|path> (or STRATEGY env)", file=sys.stderr)
        return 2
    return asyncio.run(_run(args))


if __name__ == "__main__":
    raise SystemExit(main())
