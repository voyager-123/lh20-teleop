#!/usr/bin/env python3
from runtime_paths import vision_python as resolve_vision_python
"""Replay all 16 recorded joints using relative or absolute vendor angle mapping."""
import argparse
from pathlib import Path
import sys

from probe_index_joint import ROOT, run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, default=ROOT / "data/right_hand_01.jsonl")
    parser.add_argument("--mode", choices=("relative", "absolute"), default="relative",
                        help="absolute: every source frame; execution requires official G20 or measured mapping")
    parser.add_argument("--mapping-profile", type=Path, help="official G20/L20_10 factory or measured right-hand mapping JSON")
    parser.add_argument("--vision-python", type=Path,
                        default=resolve_vision_python(),
                        help="vision environment Python used for automatic offline CAD preflight")
    parser.add_argument("--span", type=int, default=20, help="maximum relative change per active slot, 1..20")
    parser.add_argument("--middle-tip-min", type=int, default=10,
                        help="absolute mode: middle-tip protocol soft lower bound, 10..64 (default 10)")
    parser.add_argument("--execute", action="store_true", help="publish real full-hand targets")
    parser.add_argument("--return", dest="return_to_start", action="store_true",
                        help="after playback, ramp back to the sampled starting pose")
    parser.add_argument("--fault-policy", choices=("stop", "pause-thumb-128"), default="pause-thumb-128")
    args = parser.parse_args()
    if not 1 <= args.span <= 20:
        parser.error("--span must be 1..20")
    if not 10 <= args.middle_tip_min <= 64:
        parser.error("--middle-tip-min must be 10..64")
    args.full_hand = True
    try:
        run(args)
    except KeyboardInterrupt:
        return 130
    except (RuntimeError, ValueError) as error:
        print(f"Replay refused/stopped: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
