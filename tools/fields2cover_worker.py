"""Generate sparse coverage-swath endpoints with the installed Fields2Cover API.

This offline worker intentionally stops before Fields2Cover path planning.  It
outputs only each ordered swath's start and end points; the caller supplies a
formation-aware swath spacing rather than one UAV's sensor width.  The real
lower layer is responsible for transitions, turn geometry, interpolation, and
PX4 setpoints.

The locally installed Fields2Cover Python binding can abort during interpreter
shutdown on this machine.  The worker therefore flushes one JSON result and
uses ``os._exit`` after all native objects have been consumed.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate sparse Fields2Cover swath endpoints"
    )
    parser.add_argument("--min-x", type=float, required=True)
    parser.add_argument("--max-x", type=float, required=True)
    parser.add_argument("--min-y", type=float, required=True)
    parser.add_argument("--max-y", type=float, required=True)
    parser.add_argument("--coverage-width", type=float, required=True)
    parser.add_argument(
        "--start-variant",
        type=int,
        default=1,
        choices=(0, 1, 2, 3),
        help="Fields2Cover Boustrophedon start/direction variant",
    )
    return parser


def _finish(payload: dict, exit_code: int) -> None:
    sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)


def _rectangle_cells(f2c, args):
    points = [
        f2c.Point(args.min_x, args.min_y),
        f2c.Point(args.max_x, args.min_y),
        f2c.Point(args.max_x, args.max_y),
        f2c.Point(args.min_x, args.max_y),
        f2c.Point(args.min_x, args.min_y),
    ]
    return f2c.Cells(f2c.Cell(f2c.LinearRing(f2c.VectorPoint(points))))


def _waypoint(point, yaw: float) -> dict:
    result = {
        "x": round(float(point.getX()), 4),
        "y": round(float(point.getY()), 4),
        "yaw": round(float(yaw), 6),
    }
    if not all(math.isfinite(value) for value in result.values()):
        raise RuntimeError("Fields2Cover produced a non-finite swath endpoint")
    return result


def main() -> None:
    args = _parser().parse_args()
    try:
        import fields2cover as f2c

        if args.max_x <= args.min_x or args.max_y <= args.min_y:
            raise ValueError("coverage rectangle maximums must exceed minimums")
        if args.coverage_width <= 0:
            raise ValueError("coverage width must be positive")

        cells = _rectangle_cells(f2c, args)
        robot = f2c.Robot(2.0, args.coverage_width)
        swath_generator = f2c.SG_BruteForce()
        swaths = swath_generator.generateSwaths(
            math.pi, robot.getCovWidth(), cells.getGeometry(0)
        )
        swaths = f2c.RP_Boustrophedon().genSortedSwaths(
            swaths, args.start_variant
        )
        if swaths.size() == 0:
            raise RuntimeError("Fields2Cover produced no coverage swaths")

        points = []
        swath_records = []
        for index in range(swaths.size()):
            swath = swaths.at(index)
            start = swath.startPoint()
            end = swath.endPoint()
            yaw = math.atan2(
                float(end.getY()) - float(start.getY()),
                float(end.getX()) - float(start.getX()),
            )
            start_waypoint = _waypoint(start, yaw)
            end_waypoint = _waypoint(end, yaw)
            points.extend((start_waypoint, end_waypoint))
            swath_records.append(
                {
                    "index": index,
                    "start": start_waypoint,
                    "end": end_waypoint,
                }
            )

        _finish(
            {
                "ok": True,
                "generator": "fields2cover",
                "ordering": "boustrophedon",
                "swath_count": swaths.size(),
                "points": points,
                "swaths": swath_records,
            },
            0,
        )
    except BaseException as exc:  # Native binding errors must stay machine-readable.
        _finish({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, 1)


if __name__ == "__main__":
    main()
