# TurtleBot 4 Navigation Package

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blueviolet?logo=ros)
![Python](https://img.shields.io/badge/python-3.10%2B-yellow?logo=python)
![License](https://img.shields.io/badge/License-MIT-green)

A lightweight, production-ready set of navigation behaviours for the **Amazon Robotics TurtleBot 4**.  
It ships reusable classes for pose navigation, waypoint following, multi-room tours, and battery-aware patrol loops—wrapped in a clean CLI and ROS 2 launch files.

---

## ✨ Features

| Behaviour | Class | Command | Description |
|-----------|-------|---------|-------------|
| **Room → Room** | `RoomToRoomNavigator` | `ros2 run my_nav_pkg room_to_room --x -2.0 --y 3.0 --heading EAST` | One-shot trip from the dock to a target pose. |
| **Tour Rooms** | `MultiPoseNavigator` | `ros2 run my_nav_pkg tour_rooms` | Drives through a fixed list of poses, then re-docks. |
| **Follow Way-points** | `WaypointNavigator` | `ros2 run my_nav_pkg follow_wp` | Re-plans at each waypoint—great for narrow corridors. |
| **Patrol & Recharge** | `PatrolNavigator` | `ros2 run my_nav_pkg patrol` | Infinite loop with battery monitor that auto-docks to recharge. |

---

## 📂 Package Layout

```text
navigation/
├── core.py        # Navigator class hierarchy (no CLI)
├── presets.py     # Default pose & waypoint lists
├── cli.py         # Argument parsing + entry-point
├── __init__.py
└── …