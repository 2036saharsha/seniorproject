from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from math import floor
from threading import Lock, Thread
from time import sleep
from typing import List, Sequence

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Navigator,
    TurtleBot4Directions,
)

__all__ = [
    "Pose2D",
    "BaseNavigator",
    "RoomToRoomNavigator",
    "MultiPoseNavigator",
    "WaypointNavigator",
    "PatrolNavigator",
]


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    heading: TurtleBot4Directions | str | int

    def to_pose_stamped(self, nav: TurtleBot4Navigator):
        hdg = (
            self.heading
            if isinstance(self.heading, TurtleBot4Directions)
            else TurtleBot4Directions[str(self.heading)]
            if isinstance(self.heading, str)
            else TurtleBot4Directions(self.heading)
        )
        return nav.getPoseStamped([self.x, self.y], hdg)


class NavLifecycle(Enum):
    CREATED = auto()
    READY = auto()
    ACTIVE = auto()
    SHUTDOWN = auto()


class BaseNavigator(Node):

    def __init__(self, name: str, *, start_docked: bool = True):
        super().__init__(name)
        self._navigator = TurtleBot4Navigator()

        if start_docked and not self._navigator.getDockedStatus():
            self.get_logger().info("Docking before setting initial pose …")
            self._navigator.dock()

        init_pose = self._navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self._navigator.setInitialPose(init_pose)
        self._navigator.waitUntilNav2Active()

    def info(self, m: str):
        self.get_logger().info(m)

    def warn(self, m: str):
        self.get_logger().warning(m)

    def error(self, m: str):
        self.get_logger().error(m)

    def _undock_if_needed(self):
        if self._navigator.getDockedStatus():
            self._navigator.undock()

    def _dock(self):
        self._navigator.dock()


class RoomToRoomNavigator(BaseNavigator):
    def __init__(self, target: Pose2D):
        super().__init__("room_to_room_nav")
        self._target = target

    def execute(self):
        self._undock_if_needed()
        self.info(f"Navigating to {self._target} …")
        self._navigator.startToPose(self._target.to_pose_stamped(self._navigator))
        self.info("Arrived – docking …")
        self._dock()


class MultiPoseNavigator(BaseNavigator):
    def __init__(self, poses: Sequence[Pose2D]):
        super().__init__("multi_pose_nav")
        self._poses = poses

    def execute(self):
        self._undock_if_needed()
        self._navigator.startThroughPoses([
            p.to_pose_stamped(self._navigator) for p in self._poses
        ])
        self._dock()


class WaypointNavigator(BaseNavigator):
    def __init__(self, waypoints: Sequence[Pose2D]):
        super().__init__("waypoint_nav")
        self._wps = waypoints

    def execute(self):
        self._undock_if_needed()
        self._navigator.startFollowWaypoints([
            p.to_pose_stamped(self._navigator) for p in self._wps
        ])
        self._dock()


class PatrolNavigator(BaseNavigator):
    BATTERY_LOW = 0.30
    BATTERY_HIGH = 0.90
    BATTERY_CRITICAL = 0.10

    class _Battery(Node):
        def __init__(self, lock: Lock):
            super().__init__("battery_monitor")
            self._lock = lock
            self._percent: float | None = None
            self.create_subscription(
                BatteryState, "battery_state", self._cb, rclpy.qos.qos_profile_sensor_data
            )

        def _cb(self, msg: BatteryState):
            with self._lock:
                self._percent = msg.percentage

        def percent(self):
            with self._lock:
                return self._percent

        def spin_bg(self):
            ex = SingleThreadedExecutor(); ex.add_node(self); ex.spin()

    def __init__(self, loop: Sequence[Pose2D]):
        super().__init__("patrol_nav")
        self._loop = loop
        self._lock = Lock()
        self._bat = PatrolNavigator._Battery(self._lock)
        Thread(target=self._bat.spin_bg, daemon=True).start()

    def execute(self):
        self._undock_if_needed()
        idx = 0
        while rclpy.ok():
            pct = self._bat.percent()
            if pct is None:
                sleep(1); continue
            self.info(f"Battery {pct*100:.1f}%")
            if pct < self.BATTERY_CRITICAL:
                self.error("Battery critically low – aborting patrol."); break
            if pct < self.BATTERY_LOW:
                self._recharge(); idx = 0; continue
            pose = self._loop[idx]
            self._navigator.startToPose(pose.to_pose_stamped(self._navigator))
            idx = (idx + 1) % len(self._loop)

    def _recharge(self):
        self.warn("Battery low – heading to dock …")
        self._navigator.startToPose(Pose2D(-1.0, 1.0, "EAST").to_pose_stamped(self._navigator))
        self._dock()
        while (pct := self._bat.percent()) < self.BATTERY_HIGH:
            sleep(15); self.info(f"Charging {pct*100:.1f}%")
        self._undock_if_needed()