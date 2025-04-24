from .core import Pose2D

DEFAULT_POSE_SEQUENCE = [
    Pose2D(-3.0, 0.0, "EAST"),
    Pose2D(-3.0, -3.0, "NORTH"),
    Pose2D(3.0, -3.0, "NORTH_WEST"),
    Pose2D(9.0, -1.0, "WEST"),
    Pose2D(9.0, 1.0, "SOUTH"),
    Pose2D(-1.0, 1.0, "EAST"),
]

DEFAULT_WAYPOINTS = [
    Pose2D(-3.3, 5.9, "NORTH"),
    Pose2D(2.1, 6.3, "EAST"),
    Pose2D(2.0, 1.0, "SOUTH"),
    Pose2D(-1.0, 0.0, "NORTH"),
]

DEFAULT_PATROL_LOOP = [
    Pose2D(-5.0, 1.0, "EAST"),
    Pose2D(-5.0, -23.0, "NORTH"),
    Pose2D(9.0, -23.0, "NORTH_WEST"),
    Pose2D(10.0, 2.0, "WEST"),
]

