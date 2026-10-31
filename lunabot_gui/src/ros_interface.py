#!/usr/bin/env python3
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, Float32, Float64, Float64MultiArray

from lunabot_logger import Logger
from lunabot_msgs.action import Depositing, Excavation
from lunabot_msgs.msg import LaunchStatus, PowerMonitor
from lunabot_msgs.srv import LaunchSystem, StopSystem

try:
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


# Service / process / SSH timeout constants (seconds)
_TIMEOUT_SERVICE_WAIT = 10.0
_TIMEOUT_SSH_COMMAND = 10
_TIMEOUT_PROCESS_TERMINATE = 5

# Fail fast instead of hanging on a password or host key prompt
_SSH_OPTS = ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3']

_LAUNCH_FILES = {
    'lio':          'fastlio_launch.py',
    'mapping':      'mapping_launch.py',
    'nav2':         'nav2_stack_launch.py',
    'hardware':     'hardware_launch.py',
    'record':       'record_launch.py',
}

_DISPLAY_NAMES = {
    'lio':          'FAST-LIO',
    'mapping':      'RTAB-Map',
    'nav2':         'Navigation2',
    'hardware':     'Hardware',
    'record':       'Recording',
}

class RobotInterface:
    def __init__(self, mode_param):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('lunabot_gui')
        self._shutdown = False
        self.log = Logger(self.node)

        self.node.declare_parameter('network.robot_host', '192.168.0.250')
        self.node.declare_parameter('network.robot_user', 'codetc')
        self.node.declare_parameter('network.robot_workspace', '~/lunabot_ws')
        
        # mode_param uses True=sim convention (from the 'mode' ROS param default)
        if isinstance(mode_param, bool):
            self.is_real_mode = not mode_param
        elif isinstance(mode_param, str):
            mode_lower = mode_param.lower()
            self.is_real_mode = (mode_lower == 'false' or mode_lower == 'real')
        else:
            self.is_real_mode = False
        
        self.robot_mode_type = 'real' if self.is_real_mode else 'sim'
        self.log.info(f'ROS interface running in {self.robot_mode_type.upper()} mode')
        
        self.robot_host = self.node.get_parameter('network.robot_host').value
        self.robot_user = self.node.get_parameter('network.robot_user').value
        self.robot_workspace = self.node.get_parameter('network.robot_workspace').value
        self.is_remote = self.is_real_mode  # remote SSH launch only applies in real mode
        
        if self.is_remote:
            self.log.info(f'Remote robot: {self.robot_user}@{self.robot_host}:{self.robot_workspace}')
        
        self.bridge = CvBridge() if CV_AVAILABLE else None
        
        self.bandwidth_total = 0.0
        self.bandwidth_rx = 0.0
        self.bandwidth_tx = 0.0
        self.bandwidth_avg_total = 0.0
        self.bandwidth_avg_rx = 0.0
        self.bandwidth_avg_tx = 0.0

        self.power_voltage = 0.0
        self.power_current = 0.0
        self.power_watts = 0.0
        self.power_temp = 0.0
        self.power_energy_wh = 0.0
        self.power_alert_status = "Unknown"

        self.front_camera_image = None
        self.rear_camera_image = None
        self.fisheye_camera_image = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.orientation_roll = 0.0
        self.orientation_pitch = 0.0
        self.orientation_yaw = 0.0
        self.bucket_position = 0.0
        self.actuator_position = -5.25

        self.realsense_enabled = False
        self.front_camera_sub = None
        self.rear_camera_sub = None

        self.robot_mode = "MANUAL"
        self.is_excavating = False
        self.is_depositing = False
        self.robot_disabled = False
        self.vibration_duty_cycle = 0.0
        
        # True = running on the robot, Popen = local subprocess
        self.launch_processes = {
            'hardware': None,
            'actions': None,
            'lio': None,
            'mapping': None,
            'nav2': None,
            'record': None
        }
        
        # Robot systems with a dead node, per /launch_manager/status
        self.degraded_systems = set()

        self.excavation_client = ActionClient(self.node, Excavation, 'auto_excavation_action')
        self.depositing_client = ActionClient(self.node, Depositing, 'auto_depositing_action')

        # Killing the navigation client does not cancel the goals it sent, so cancel them directly
        self.auto_cancel_clients = [
            self.node.create_client(CancelGoal, f'{action}/_action/cancel_goal')
            for action in ('navigate_to_pose', 'drive_on_heading', 'spin', 'auto_excavation_action', 'auto_depositing_action')
        ]
        
        self.excavation_goal_handle = None
        self.depositing_goal_handle = None
        self.navigation_client_process = None
        self.auto_mode = 'ucf'
        self.auto_dry_run = False
        
        self.emergency_stop_pub = self.node.create_publisher(Bool, '/emergency_stop', 10)
        self.mode_switch_pub = self.node.create_publisher(Bool, '/mode_switch', 10)
        self.position_cmd_pub = self.node.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.camera_cmd_pub = self.node.create_publisher(Float64MultiArray, '/camera_controller/commands', 10)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        self.launch_client = self.node.create_client(LaunchSystem, 'launch_system')
        self.stop_client = self.node.create_client(StopSystem, 'stop_system')
        
        self._create_subscriptions()
        
        # Callbacks for UI updates (set by GUI)
        self.on_robot_state_update = None
        self.on_realsense_toggle = None
        self.on_log = None
    
    def _create_subscriptions(self):
        self.node.create_subscription(
            Float32, '/bandwidth/total_mbps',
            lambda msg: setattr(self, 'bandwidth_total', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/rx_mbps',
            lambda msg: setattr(self, 'bandwidth_rx', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/tx_mbps',
            lambda msg: setattr(self, 'bandwidth_tx', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/avg_total_mbps',
            lambda msg: setattr(self, 'bandwidth_avg_total', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/avg_rx_mbps',
            lambda msg: setattr(self, 'bandwidth_avg_rx', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/avg_tx_mbps',
            lambda msg: setattr(self, 'bandwidth_avg_tx', msg.data), 10)

        if CV_AVAILABLE:
            if self.realsense_enabled:
                self._enable_realsense_subscriptions()
            self.node.create_subscription(
                CompressedImage, '/camera_fisheye/color/image_compressed',
                lambda msg: self._camera_callback(msg, 'fisheye', 'fisheye_camera_image'), 1)
        
        self.node.create_subscription(
            Odometry, '/lio_odom', self._odom_callback, 10)

        self.node.create_subscription(
            PowerMonitor, '/power_monitor', self._power_monitor_callback, 10)

        self.node.create_subscription(
            Bool, '/manual_mode', self._manual_mode_callback, 10)

        self.node.create_subscription(
            Bool, '/robot_disabled', self._robot_disabled_callback, 10)

        self.node.create_subscription(
            Float32, '/vibration_duty_cycle', self._vibration_duty_cycle_callback, 10)

        self.node.create_subscription(
            Float64, '/actuator_position',
            lambda msg: setattr(self, 'actuator_position', msg.data), 10)
        self.node.create_subscription(
            Float64, '/bucket_angle',
            lambda msg: setattr(self, 'bucket_position', msg.data), 10)

        self.node.create_subscription(
            JointState, '/joint_states', self._joint_states_callback, 10)

        if self.is_remote:
            status_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.node.create_subscription(
                LaunchStatus, '/launch_manager/status', self._launch_status_callback, status_qos)
        
        # /rosout publishers use RELIABLE + TRANSIENT_LOCAL
        rosout_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1000
        )
        
        self.node.create_subscription(
            Log, '/rosout', self._log_callback, rosout_qos)
    
    def _camera_callback(self, msg, camera_name, attr_name):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            setattr(self, attr_name, cv_image)
        except Exception as e:
            self.log.failure(f'{camera_name.capitalize()} camera error: {e}')
    
    def _enable_realsense_subscriptions(self):
        if not CV_AVAILABLE:
            return
        
        # Queue size = 1 for minimal latency (we only care about latest frame)
        if self.front_camera_sub is None:
            self.front_camera_sub = self.node.create_subscription(
                CompressedImage, '/camera_front/color/image_compressed',
                lambda msg: self._camera_callback(msg, 'front', 'front_camera_image'), 1)
            self.log.info('Front RealSense camera subscription enabled')
        
        if self.rear_camera_sub is None:
            self.rear_camera_sub = self.node.create_subscription(
                CompressedImage, '/camera_back/color/image_compressed',
                lambda msg: self._camera_callback(msg, 'rear', 'rear_camera_image'), 1)
            self.log.info('Rear RealSense camera subscription enabled')
    
    def _disable_realsense_subscriptions(self):
        if self.front_camera_sub is not None:
            self.node.destroy_subscription(self.front_camera_sub)
            self.front_camera_sub = None
            self.front_camera_image = None
            self.log.info('Front RealSense camera subscription disabled')
        
        if self.rear_camera_sub is not None:
            self.node.destroy_subscription(self.rear_camera_sub)
            self.rear_camera_sub = None
            self.rear_camera_image = None
            self.log.info('Rear RealSense camera subscription disabled')
    
    def toggle_realsense_cameras(self):
        self.realsense_enabled = not self.realsense_enabled
        
        if self.realsense_enabled:
            self._enable_realsense_subscriptions()
            self.log.action('RealSense cameras enabled')
        else:
            self._disable_realsense_subscriptions()
            self.log.action('RealSense cameras disabled')
        
        if self.on_realsense_toggle:
            self.on_realsense_toggle(self.realsense_enabled)
        
        return self.realsense_enabled
    
    def restart_can_interface(self):
        try:
            script_path = f'{self.robot_workspace}/src/lunabot_ros/scripts/canable_restart.sh'
            
            if self.is_remote:
                self.log.action(f'Restarting CAN interface on {self.robot_user}@{self.robot_host}')
                cmd = ['ssh', *_SSH_OPTS, f'{self.robot_user}@{self.robot_host}', f'bash -l {script_path}']
            else:
                self.log.action(f'Restarting CAN interface: {script_path}')
                cmd = ['bash', os.path.expanduser(script_path)]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=_TIMEOUT_SSH_COMMAND)
            
            if result.stdout:
                for line in result.stdout.strip().split('\n'):
                    if line:
                        self.log.info(f'CAN: {line}')
            if result.stderr:
                for line in result.stderr.strip().split('\n'):
                    if line:
                        self.log.warning(f'CAN: {line}')
            
            if result.returncode == 0:
                self.log.success('CAN interface restarted successfully')
                return True
            else:
                self.log.failure(f'CAN restart failed with exit code {result.returncode}')
                return False
        except subprocess.TimeoutExpired:
            self.log.failure('CAN restart timed out')
            return False
        except Exception as e:
            self.log.failure(f'Error restarting CAN: {e}')
            return False
    
    def _odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.position_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.orientation_roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            self.orientation_pitch = math.copysign(math.pi / 2, sinp)
        else:
            self.orientation_pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.orientation_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z
        if self.on_robot_state_update:
            self.on_robot_state_update()
    
    def _manual_mode_callback(self, msg):
        is_manual = msg.data
        self.robot_mode = "MANUAL" if is_manual else "AUTO"
        if self.on_robot_state_update:
            self.on_robot_state_update()
    
    def _robot_disabled_callback(self, msg):
        self.robot_disabled = msg.data
        if self.on_robot_state_update:
            self.on_robot_state_update()
    
    def _vibration_duty_cycle_callback(self, msg):
        self.vibration_duty_cycle = msg.data
        if self.on_robot_state_update:
            self.on_robot_state_update()
    
    def _joint_states_callback(self, msg):
        try:
            if not self.is_real_mode and 'base_bucket_joint' in msg.name:
                idx = msg.name.index('base_bucket_joint')
                if idx < len(msg.position):
                    self.bucket_position = msg.position[idx]
        except Exception as e:
            self.log.warning(f'Error processing joint states: {e}', throttle_duration_sec=5.0)
    
    def _log_callback(self, msg):
        if self.on_log:
            level_map = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
            level = level_map.get(msg.level, 'UNKNOWN')
            log_text = f"[{level}] {msg.name}: {msg.msg}"
            try:
                self.on_log(log_text)
            except Exception as e:
                print(f"Failed to log message from {msg.name}: {e}", file=sys.stderr)
    
    def _launch_status_callback(self, msg):
        changed = False
        for name in _LAUNCH_FILES:
            running = True if name in msg.running else None
            if self.launch_processes[name] != running:
                self.launch_processes[name] = running
                changed = True
        if set(msg.degraded) != self.degraded_systems:
            self.degraded_systems = set(msg.degraded)
            changed = True
        if changed and self.on_robot_state_update:
            self.on_robot_state_update()

    def _power_monitor_callback(self, msg):
        self.power_voltage = msg.voltage
        self.power_current = msg.current
        self.power_watts = msg.power
        self.power_temp = msg.temperature
        self.power_energy_wh = msg.energy_wh
        self.power_alert_status = msg.alert_status
    
    def publish_emergency_stop(self):
        msg = Bool()
        msg.data = True
        self.emergency_stop_pub.publish(msg)
    
    def publish_re_enable(self):
        msg = Bool()
        msg.data = False
        self.emergency_stop_pub.publish(msg)
    
    def publish_mode_switch(self):
        msg = Bool()
        msg.data = not (self.robot_mode == "MANUAL")
        self.mode_switch_pub.publish(msg)
        self.log.action(f'Mode switch requested: {"MANUAL" if msg.data else "AUTO"}')
    
    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
    
    def publish_bucket_position(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        self.position_cmd_pub.publish(msg)
    
    def publish_camera_position(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        self.camera_cmd_pub.publish(msg)
    
    def start_can_interface(self):
        try:
            script_path = f'{self.robot_workspace}/src/lunabot_ros/scripts/canable_start.sh'
            
            if self.is_remote:
                self.log.action(f'Running CAN interface on {self.robot_user}@{self.robot_host}')
                cmd = ['ssh', *_SSH_OPTS, f'{self.robot_user}@{self.robot_host}', f'bash -l {script_path}']
            else:
                self.log.action(f'Running CAN interface script: {script_path}')
                cmd = ['bash', os.path.expanduser(script_path)]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=_TIMEOUT_SSH_COMMAND)
            
            if result.stdout:
                for line in result.stdout.strip().split('\n'):
                    if line:
                        self.log.info(f'CAN: {line}')
            if result.stderr:
                for line in result.stderr.strip().split('\n'):
                    if line:
                        self.log.warning(f'CAN: {line}')
            
            if result.returncode == 0:
                self.log.success('CAN interface started successfully')
            else:
                self.log.failure(f'CAN script exited with code {result.returncode}')
        except subprocess.TimeoutExpired:
            self.log.failure('CAN script timed out')
        except Exception as e:
            self.log.failure(f'Failed to start CAN interface: {str(e)}')
    
    def _kill_proc(self, process, name):
        # Wait for the whole process group, nodes can outlive ros2 launch
        pgid = process.pid
        try:
            for sig in (signal.SIGINT, signal.SIGKILL):   # SIGINT = graceful ROS 2 shutdown
                os.killpg(pgid, sig)
                deadline = time.monotonic() + _TIMEOUT_PROCESS_TERMINATE
                while time.monotonic() < deadline:
                    process.poll()
                    os.killpg(pgid, 0)   # raises ProcessLookupError once the group is gone
                    time.sleep(0.1)
                self.log.warning(f'{name} still running after {sig.name}')
            self.log.failure(f'Could not stop {name} process group')
        except ProcessLookupError:
            self.log.info(f'Terminated {name} process')
        except Exception as e:
            self.log.warning(f'Error terminating {name}: {e}')

    def is_running(self, system_name):
        process = self.launch_processes.get(system_name)
        if isinstance(process, subprocess.Popen):
            return process.poll() is None
        return process is True

    def _call_launch_manager(self, client, request, verb):
        """Send a launch manager request off the Qt thread. Service discovery over Wi-Fi can
        take a few seconds after the GUI starts, so wait for it in the background instead of
        failing right away. Button state follows /launch_manager/status, so the response is
        only logged."""
        display = _DISPLAY_NAMES.get(request.system_name, request.system_name)

        def send():
            if not client.wait_for_service(timeout_sec=_TIMEOUT_SERVICE_WAIT):
                self.log.failure(f'Launch manager not available, cannot {verb} {display}')
                return
            client.call_async(request).add_done_callback(done)

        def done(future):
            try:
                response = future.result()
            except Exception as e:
                self.log.failure(f'Failed to {verb} {display}: {e}')
                return
            if response.success:
                self.log.success(response.message)
            else:
                self.log.failure(f'Failed to {verb} {display}: {response.message}')

        threading.Thread(target=send, daemon=True).start()

    def launch_system(self, system_name):
        """Launch a system (hardware, FAST-LIO, mapping, Nav2)"""
        display = _DISPLAY_NAMES.get(system_name, system_name)
        self.log.action(f'Launching {display}')

        if self.is_running(system_name):
            self.log.warning(f'{display} already running')
            return

        try:
            use_sim = not self.is_real_mode

            if self.is_remote:
                request = LaunchSystem.Request()
                request.system_name = system_name
                request.use_sim = use_sim
                self._call_launch_manager(self.launch_client, request, 'launch')
                return

            # Output goes to the GUI's terminal; an unread PIPE fills up and stalls the launch
            use_sim_arg = 'true' if use_sim else 'false'
            self.launch_processes[system_name] = subprocess.Popen(
                ['ros2', 'launch', 'lunabot_bringup', _LAUNCH_FILES[system_name],
                 f'use_sim:={use_sim_arg}'],
                stdin=subprocess.DEVNULL,
                start_new_session=True
            )
            self.log.success(f'{display} launched locally')

            if self.on_robot_state_update:
                self.on_robot_state_update()
        except Exception as e:
            self.log.failure(f'Failed to launch {display}: {e}')

    def stop_system(self, system_name):
        display = _DISPLAY_NAMES.get(system_name, system_name)
        self.log.action(f'Stopping {display}')

        try:
            if self.is_remote:
                request = StopSystem.Request()
                request.system_name = system_name
                self._call_launch_manager(self.stop_client, request, 'stop')
                return

            process = self.launch_processes[system_name]
            if not isinstance(process, subprocess.Popen):
                self.log.warning(f'{display} is not running')
                return

            self._kill_proc(process, system_name)
            self.launch_processes[system_name] = None
            self.log.success(f'{display} stopped')

            if self.on_robot_state_update:
                self.on_robot_state_update()
        except Exception as e:
            self.log.failure(f'Error stopping {display}: {e}')

    def _get_workspace_setup_cmd(self):
        ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
        if ament_prefix:
            install_paths = ament_prefix.split(':')
            workspace_install = None
            for path in install_paths:
                if 'lunabot' in path and 'install' in path:
                    workspace_install = path.split('/install/')[0] + '/install'
                    break
            if not workspace_install and install_paths:
                workspace_install = install_paths[0].split('/install/')[0] + '/install'
            return f'source {workspace_install}/setup.bash'
        return 'source install/setup.bash'

    def launch_rviz(self):
        self.log.action('Launching RViz2')
        try:
            config_path = '$(ros2 pkg prefix lunabot_config)/share/lunabot_config/rviz/robot_view.rviz'
            subprocess.Popen(
                f'{self._get_workspace_setup_cmd()} && rviz2 -d {config_path}',
                shell=True, executable='/bin/bash')
            self.log.success('RViz2 launched')
        except Exception as e:
            self.log.failure(f'Failed to launch RViz2: {e}')
    
    def send_excavate_goal(self, response_callback, result_callback, cancel_callback):
        self.log.action('Sending excavation goal')
        goal_msg = Excavation.Goal()
        send_goal_future = self.excavation_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self._handle_excavate_response(future, response_callback, result_callback))
        return send_goal_future
    
    def _handle_excavate_response(self, future, response_callback, result_callback):
        goal_handle = future.result()
        self.excavation_goal_handle = goal_handle
        response_callback(goal_handle)
        if goal_handle.accepted:
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_callback)
    
    def cancel_excavate_goal(self, cancel_callback):
        if self.excavation_goal_handle is not None and self.excavation_goal_handle.accepted:
            self.log.action('Canceling excavation goal')
            cancel_future = self.excavation_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(cancel_callback)
        else:
            self.log.warning('No active excavation goal to cancel')
            cancel_callback(None)
    
    def send_deposit_goal(self, response_callback, result_callback, cancel_callback):
        self.log.action('Sending depositing goal')
        goal_msg = Depositing.Goal()
        send_goal_future = self.depositing_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self._handle_deposit_response(future, response_callback, result_callback))
        return send_goal_future
    
    def _handle_deposit_response(self, future, response_callback, result_callback):
        goal_handle = future.result()
        self.depositing_goal_handle = goal_handle
        response_callback(goal_handle)
        if goal_handle.accepted:
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_callback)
    
    def cancel_deposit_goal(self, cancel_callback):
        if self.depositing_goal_handle is not None and self.depositing_goal_handle.accepted:
            self.log.action('Canceling depositing goal')
            cancel_future = self.depositing_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(cancel_callback)
        else:
            self.log.warning('No active depositing goal to cancel')
            cancel_callback(None)
    
    def launch_navigation_client(self):
        dry_run = 'true' if self.auto_dry_run else 'false'
        self.log.action(f'Starting navigation client ({self.auto_mode.upper()} mode, dry run {dry_run})')
        try:
            use_sim_arg = 'false' if self.is_real_mode else 'true'
            self.navigation_client_process = subprocess.Popen(
                f'{self._get_workspace_setup_cmd()} && ros2 run lunabot_nav navigation_client'
                f' --ros-args -p use_sim_time:={use_sim_arg} -p mode:={self.auto_mode}'
                f' -p dry_run:={dry_run}',
                shell=True, executable='/bin/bash', preexec_fn=os.setsid)
            self.log.success('Navigation client launched')
            return True
        except Exception as e:
            self.log.failure(f'Failed to launch navigation client: {e}')
            return False
    
    def stop_navigation_client(self):
        if self.navigation_client_process:
            try:
                os.killpg(os.getpgid(self.navigation_client_process.pid), signal.SIGTERM)
                self.navigation_client_process = None
                self.log.success('Navigation client stopped')
            except Exception as e:
                self.log.failure(f'Failed to stop navigation client: {e}')

        # An empty request (zero goal id and stamp) cancels every goal on the server
        for client in self.auto_cancel_clients:
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())
        self.publish_velocity(0.0, 0.0)
    
    def shutdown(self):
        if self._shutdown:
            return
        self._shutdown = True

        # Signal every group first so they stop in parallel
        local = [(n, p) for n, p in self.launch_processes.items() if isinstance(p, subprocess.Popen)]
        for name, process in local:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for name, process in local:
            self._kill_proc(process, name)

        self.stop_navigation_client()

        # Destroy node (don't shutdown rclpy as it may be used by other components)
        try:
            self.node.destroy_node()
        except Exception:
            pass
