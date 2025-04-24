import argparse, sys
from typing import Sequence
import rclpy
rclpy.init()

from .core import (
    Pose2D,
    RoomToRoomNavigator,
    MultiPoseNavigator,
    WaypointNavigator,
    PatrolNavigator,
)
from . import presets as _pz


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser("TurtleBot 4 Navigation")
    sub = p.add_subparsers(dest="mode", required=True)
    r2r = sub.add_parser("room_to_room")
    r2r.add_argument("--x", type=float, required=True)
    r2r.add_argument("--y", type=float, required=True)
    r2r.add_argument("--heading", default="NORTH")
    sub.add_parser("tour_rooms")
    sub.add_parser("follow_wp")
    sub.add_parser("patrol")
    return p


def main(argv: Sequence[str] | None = None):
    args = _parser().parse_args(argv if argv is not None else sys.argv[1:])
    match args.mode:
        case "room_to_room": navigator = RoomToRoomNavigator(Pose2D(args.x, args.y, args.heading))
        case "tour_rooms":   navigator = MultiPoseNavigator(_pz.DEFAULT_POSE_SEQUENCE)
        case "follow_wp":    navigator = WaypointNavigator(_pz.DEFAULT_WAYPOINTS)
        case "patrol":       navigator = PatrolNavigator(_pz.DEFAULT_PATROL_LOOP)
    try:
        navigator.execute()
    finally:
        navigator.destroy(); rclpy.shutdown()