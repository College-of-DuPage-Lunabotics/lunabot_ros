import os
import shutil
import subprocess
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration

TOPICS = [
    # Raw lidar and IMU
    "/livox/lidar",
    "/livox/imu",
    # Camera footage
    "/camera_front/color/image_compressed",
    "/camera_back/color/image_compressed",
    "/camera_fisheye/color/image_compressed",
    "/camera_front/color/camera_info",
    "/camera_back/color/camera_info",
    # Transforms, robot model and odometry
    "/tf",
    "/tf_static",
    "/clock",
    "/robot_description",
    "/joint_states",
    "/lio_odom",
    "/lio_odom_imu",
    "/wheel_odom",
    "/diff_drive_controller/odom",
    # Maps
    "/map",
    "/mapData",
    "/mapGraph",
    "/cloud_obstacles",
    "/cloud_ground",
    "/local_obstacles",
    "/local_costmap/costmap",
    # Navigation
    "/plan",
    "/goal_pose",
    "/lookahead_point",
    "/received_global_plan",
    "/navigate_to_pose/_action/status",
    "/navigate_to_pose/_action/feedback",
    "/drive_on_heading/_action/status",
    "/drive_on_heading/_action/feedback",
    "/auto_excavation_action/_action/status",
    "/auto_excavation_action/_action/feedback",
    "/auto_depositing_action/_action/status",
    "/auto_depositing_action/_action/feedback",
    # Commands
    "/cmd_vel",
    "/diff_drive_controller/cmd_vel_unstamped",
    "/motor_commands",
    "/position_controller/commands",
    "/camera_controller/commands",
    # Robot state and logs
    "/rosout",
    "/robot_disabled",
    "/manual_mode",
    "/emergency_stop",
    "/mode_switch",
    "/power_monitor",
    "/vibration_duty_cycle",
    "/bucket_angle",
    "/actuator_position",
    "/launch_manager/status",
    "/joy",
]

SPLIT_SECONDS = 300

MIN_FREE_GB = 20


def _default_bag_root():
    # <ws>/install/lunabot_bringup/share/lunabot_bringup -> <ws>/src/bags
    return (
        Path(get_package_share_directory("lunabot_bringup")).parents[3] / "src" / "bags"
    )


def _git_info():
    repo = Path(__file__).resolve().parents[2]
    try:
        commit = subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "-C", str(repo), "status", "--short"],
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout
    except Exception:
        commit, status = "", ""
    return commit or "unknown", status


def launch_setup(context, *args, **kwargs):
    use_sim = LaunchConfiguration("use_sim").perform(context).lower() == "true"
    label = LaunchConfiguration("label").perform(context)
    bag_root_arg = LaunchConfiguration("bag_root").perform(context)
    bag_root = Path(bag_root_arg) if bag_root_arg else _default_bag_root()

    bag_root.mkdir(parents=True, exist_ok=True)
    (bag_root / "COLCON_IGNORE").touch()

    mode = "sim" if use_sim else "real"
    name = (
        datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        + f"_{mode}"
        + (f"_{label}" if label else "")
    )
    bag_dir = bag_root / name

    free_gb = shutil.disk_usage(bag_root).free / 1e9
    commit, status = _git_info()
    info = (
        f"mode: {mode}\nstarted: {datetime.now()}\nhost: {os.uname().nodename}\n"
        f"commit: {commit}\nuncommitted changes:\n{status}"
    )

    def write_info(context, *args, **kwargs):
        # ros2 bag creates the directory itself, so write once it exists
        if not bag_dir.is_dir():
            return [LogInfo(msg=f"WARNING: {bag_dir} not created yet, test_info.txt not written")]
        (bag_dir / "test_info.txt").write_text(info)
        return []

    actions = [
        LogInfo(msg=f"Recording to {bag_dir} ({free_gb:.0f} GB free)"),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "--include-hidden-topics",
                "--max-bag-duration",
                str(SPLIT_SECONDS),
                "-o",
                str(bag_dir),
                *TOPICS,
            ],
            output="screen",
        ),
        TimerAction(period=5.0, actions=[OpaqueFunction(function=write_info)]),
    ]
    if free_gb < MIN_FREE_GB:
        actions.insert(
            0, LogInfo(msg=f"WARNING: only {free_gb:.0f} GB free for recording")
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim",
                default_value="false",
                description="Whether we are in simulation or not",
            ),
            DeclareLaunchArgument(
                "label",
                default_value="",
                description="Optional label appended to the bag name",
            ),
            DeclareLaunchArgument(
                "bag_root",
                default_value="",
                description="Folder for bags, defaults to <workspace>/src/bags",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
