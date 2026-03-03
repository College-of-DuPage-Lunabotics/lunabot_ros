#!/usr/bin/env python3
import os
import subprocess
import time
import rclpy
from PyQt5.QtCore import QCoreApplication
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32, String, Bool, Float64MultiArray
from sensor_msgs.msg import Image, CompressedImage, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray
from lunabot_msgs.msg import ControlState, PowerMonitor
from lunabot_msgs.action import Excavation, Depositing, Homing
from lunabot_msgs.srv import LaunchSystem, StopSystem

try:
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


# Service / process timeout constants (seconds)
_TIMEOUT_SERVICE_WAIT = 5.0
_TIMEOUT_SERVICE_WAIT_LONG = 10.0
_TIMEOUT_SERVICE_RESPONSE = 30.0
_TIMEOUT_STOP_RESPONSE = 10.0
_TIMEOUT_PROCESS_TERMINATE = 5


class RobotInterface:
    """ROS interface for robot communication and control"""
    
    def __init__(self, mode_param):
        """
        Initialize ROS interface
        
        Args:
            mode_param: Robot mode (True=sim, False=real or 'sim'/'real')
        """
        # Initialize ROS if not already initialized
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('lunabot_gui')
        self._shutdown = False
        
        # Declare and read network parameters from config
        self.node.declare_parameter('network.robot_host', '192.168.0.250')
        self.node.declare_parameter('network.robot_user', 'codetc')
        self.node.declare_parameter('network.robot_workspace', '~/lunabot_ws')
        self.node.declare_parameter('steam_mode', False)
        
        # Parse mode parameter
        if isinstance(mode_param, bool):
            self.is_real_mode = not mode_param
        elif isinstance(mode_param, str):
            mode_lower = mode_param.lower()
            self.is_real_mode = (mode_lower == 'false' or mode_lower == 'real')
        else:
            self.is_real_mode = False
        
        self.robot_mode_type = 'real' if self.is_real_mode else 'sim'
        self.node.get_logger().info(f'ROS interface running in {self.robot_mode_type.upper()} mode')
        
        # Remote robot configuration from parameters
        self.robot_host = self.node.get_parameter('network.robot_host').value
        self.robot_user = self.node.get_parameter('network.robot_user').value
        self.robot_workspace = self.node.get_parameter('network.robot_workspace').value
        self.is_remote = self.is_real_mode
        self.steam_mode = self.node.get_parameter('steam_mode').value
        
        self.node.get_logger().info(f'Controller mode: {"Steam Deck" if self.steam_mode else "Xbox"}')
        
        if self.is_remote:
            self.node.get_logger().info(f'Remote robot: {self.robot_user}@{self.robot_host}:{self.robot_workspace}')
        
        # Initialize CV bridge
        self.bridge = CvBridge() if CV_AVAILABLE else None
        
        # Data storage - bandwidth (current)
        self.bandwidth_total = 0.0
        self.bandwidth_rx = 0.0
        self.bandwidth_tx = 0.0
        
        # Data storage - bandwidth (running average)
        self.bandwidth_avg_total = 0.0
        self.bandwidth_avg_rx = 0.0
        self.bandwidth_avg_tx = 0.0
        
        # Data storage - power monitoring
        self.power_voltage = 0.0
        self.power_current = 0.0
        self.power_watts = 0.0
        self.power_temp = 0.0
        self.power_energy_kwh = 0.0
        self.power_alert_status = "Unknown"
        
        # Data storage - robot state
        self.front_camera_image = None
        self.rear_camera_image = None
        self.fisheye_camera_image = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.bucket_position = -0.2
        
        # System enable states
        self.hardware_enabled = False
        self.pointlio_enabled = False
        self.mapping_enabled = False
        self.nav2_enabled = False
        self.localization_enabled = False
        
        # Robot mode and operation states
        self.robot_mode = "MANUAL"
        self.is_excavating = False
        self.is_depositing = False
        self.is_homing = False
        self.is_navigating = False
        self.robot_disabled = False
        self.vibration_state = False
        self.vibration_duty_cycle = 0.0
        
        # Launch process tracking (True = remote via service, Process = local subprocess)
        self.launch_processes = {
            'hardware': None,
            'actions': None,
            'pointlio': None,
            'mapping': None,
            'nav2': None,
            'localization': None
        }
        
        # Action clients and goal handles
        self.excavation_client = ActionClient(self.node, Excavation, 'excavation_action')
        self.depositing_client = ActionClient(self.node, Depositing, 'depositing_action')
        self.homing_client = ActionClient(self.node, Homing, 'homing_action')
        
        self.excavation_goal_handle = None
        self.depositing_goal_handle = None
        self.homing_goal_handle = None
        self.navigation_client_process = None
        
        # Create publishers
        self.emergency_stop_pub = self.node.create_publisher(Bool, '/emergency_stop', 10)
        self.control_state_pub = self.node.create_publisher(ControlState, '/control_state', 10)
        self.mode_switch_pub = self.node.create_publisher(Bool, '/mode_switch', 10)
        self.position_cmd_pub = self.node.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.camera_cmd_pub = self.node.create_publisher(Float64MultiArray, '/camera_controller/commands', 10)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create service clients for launch manager
        self.launch_client = self.node.create_client(LaunchSystem, 'launch_system')
        self.stop_client = self.node.create_client(StopSystem, 'stop_system')
        
        # Create subscriptions
        self._create_subscriptions()
        
        # Callbacks for UI updates (set by GUI)
        self.on_bandwidth_update = None
        self.on_camera_update = None
        self.on_robot_state_update = None
        self.on_control_state_update = None
    
    def _create_subscriptions(self):
        """Create all ROS topic subscriptions"""
        # Bandwidth monitoring - Current
        self.node.create_subscription(
            Float32, '/bandwidth/total_mbps',
            lambda msg: setattr(self, 'bandwidth_total', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/rx_mbps',
            lambda msg: setattr(self, 'bandwidth_rx', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/tx_mbps',
            lambda msg: setattr(self, 'bandwidth_tx', msg.data), 10)
        
        # Bandwidth monitoring - Running Average
        self.node.create_subscription(
            Float32, '/bandwidth/avg_total_mbps',
            lambda msg: setattr(self, 'bandwidth_avg_total', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/avg_rx_mbps',
            lambda msg: setattr(self, 'bandwidth_avg_rx', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/avg_tx_mbps',
            lambda msg: setattr(self, 'bandwidth_avg_tx', msg.data), 10)
        
        # Camera feeds
        if CV_AVAILABLE:
            self.node.create_subscription(
                CompressedImage, '/camera_front/color/image_compressed',
                lambda msg: self._camera_callback(msg, 'front', 'front_camera_image'), 10)
            self.node.create_subscription(
                CompressedImage, '/camera_back/color/image_compressed',
                lambda msg: self._camera_callback(msg, 'rear', 'rear_camera_image'), 10)
            self.node.create_subscription(
                CompressedImage, '/camera_fisheye/color/image_compressed',
                lambda msg: self._camera_callback(msg, 'fisheye', 'fisheye_camera_image'), 10)
        
        # Robot odometry
        self.node.create_subscription(
            Odometry, '/odometry/filtered', self._odom_callback, 10)

        # Control state
        self.node.create_subscription(
            ControlState, '/control_state', self._control_state_callback, 10)
        
        # Power monitoring
        self.node.create_subscription(
            PowerMonitor, '/power_monitor', self._power_monitor_callback, 10)
        
        # Robot mode
        self.node.create_subscription(
            Bool, '/manual_mode', self._manual_mode_callback, 10)
        
        # Robot disabled status
        self.node.create_subscription(
            Bool, '/robot_disabled', self._robot_disabled_callback, 10)
        
        # Vibration duty cycle
        self.node.create_subscription(
            Float32, '/vibration_duty_cycle', self._vibration_duty_cycle_callback, 10)
        
        # Joint states
        self.node.create_subscription(
            JointState, '/joint_states', self._joint_states_callback, 10)
    
    # Camera callbacks
    def _camera_callback(self, msg, camera_name, attr_name):
        """Generic handler for compressed camera image messages"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            setattr(self, attr_name, cv_image)
            if self.on_camera_update:
                self.on_camera_update(camera_name, cv_image)
        except Exception as e:
            self.node.get_logger().error(f'{camera_name.capitalize()} camera error: {e}')
    
    def _odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
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
            if 'base_bucket_joint' in msg.name:
                idx = msg.name.index('base_bucket_joint')
                if idx < len(msg.position):
                    self.bucket_position = msg.position[idx]
        except Exception as e:
            self.node.get_logger().warn(f'Error processing joint states: {e}', throttle_duration_sec=5.0)
    
    def _control_state_callback(self, msg):
        self.robot_mode = "MANUAL" if msg.mode == 0 else "AUTO"
        self.hardware_enabled = msg.hardware_enabled
        self.pointlio_enabled = msg.pointlio_enabled
        self.mapping_enabled = msg.mapping_enabled
        self.nav2_enabled = msg.nav2_enabled
        self.localization_enabled = msg.localization_enabled
        self.is_excavating = msg.is_excavating
        self.is_depositing = msg.is_depositing
        self.is_navigating = msg.is_navigating
        
        if self.on_control_state_update:
            self.on_control_state_update(msg)
    
    def _power_monitor_callback(self, msg):
        """Handle power monitor telemetry updates"""
        self.power_voltage = msg.voltage
        self.power_current = msg.current
        self.power_watts = msg.power
        self.power_temp = msg.temperature
        self.power_energy_kwh = msg.energy_kwh
        self.power_alert_status = msg.alert_status
    
    # Command publishing methods
    def publish_emergency_stop(self):
        """Publish emergency stop command"""
        msg = Bool()
        msg.data = True
        self.emergency_stop_pub.publish(msg)
    
    def publish_re_enable(self):
        """Publish re-enable command (clear emergency stop)"""
        msg = Bool()
        msg.data = False
        self.emergency_stop_pub.publish(msg)
    
    def publish_mode_switch(self):
        """Publish mode switch command to toggle mode"""
        msg = Bool()
        # Toggle: if currently manual, switch to auto (False), if currently auto, switch to manual (True)
        msg.data = not (self.robot_mode == "MANUAL")
        self.mode_switch_pub.publish(msg)
        self.node.get_logger().info(f'Mode switch requested: {"MANUAL" if msg.data else "AUTO"}')
    
    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
    
    def publish_bucket_position(self, position):
        """Publish bucket position command"""
        msg = Float64MultiArray()
        msg.data = [position]
        self.position_cmd_pub.publish(msg)
    
    def publish_camera_position(self, position):
        """Publish fisheye camera position command"""
        msg = Float64MultiArray()
        msg.data = [position]
        self.camera_cmd_pub.publish(msg)
    
    # System launch methods
    def start_can_interface(self):
        """Start CAN interface using canable_start.sh script"""
        try:
            script_path = f'{self.robot_workspace}/src/lunabot_ros/scripts/canable_start.sh'
            
            if self.is_remote:
                self.node.get_logger().info(f'Running CAN interface on {self.robot_user}@{self.robot_host}')
                # Use bash -l to ensure proper environment and sudoers is loaded
                cmd = ['ssh', f'{self.robot_user}@{self.robot_host}', f'bash -l {script_path}']
            else:
                self.node.get_logger().info(f'Running CAN interface script: {script_path}')
                cmd = ['bash', os.path.expanduser(script_path)]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=int(_TIMEOUT_SERVICE_WAIT_LONG))
            
            if result.stdout:
                for line in result.stdout.strip().split('\n'):
                    if line:
                        self.node.get_logger().info(f'CAN: {line}')
            if result.stderr:
                for line in result.stderr.strip().split('\n'):
                    if line:
                        self.node.get_logger().warn(f'CAN: {line}')
            
            if result.returncode == 0:
                self.node.get_logger().info('CAN interface started successfully')
            else:
                self.node.get_logger().error(f'CAN script exited with code {result.returncode}')
        except subprocess.TimeoutExpired:
            self.node.get_logger().error('CAN script timed out')
        except Exception as e:
            self.node.get_logger().error(f'Failed to start CAN interface: {str(e)}')
    
    def _wait_for_service_response(self, future, timeout_sec):
        """Block (processing Qt events) until the future completes or times out. Returns response or None."""
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            QCoreApplication.processEvents()
            time.sleep(0.01)
        if future.done() and future.result() is not None:
            return future.result()
        return None

    def _set_system_enabled(self, system_name, value):
        """Set the enabled flag for a named system"""
        flag_map = {
            'pointlio':     'pointlio_enabled',
            'mapping':      'mapping_enabled',
            'nav2':         'nav2_enabled',
            'localization': 'localization_enabled',
            'hardware':     'hardware_enabled',
        }
        attr = flag_map.get(system_name)
        if attr is not None:
            setattr(self, attr, value)

    def launch_hardware(self):
        """Launch hardware system for real robot"""
        if self.launch_processes['hardware'] is not None:
            self.node.get_logger().warning('Hardware already running')
            return
        
        try:
            if self.is_remote:
                self.node.get_logger().info('Launching hardware via launch manager service')
                
                request = LaunchSystem.Request()
                request.system_name = 'hardware'
                request.use_sim = False
                request.steam_mode = self.steam_mode
                
                if not self.launch_client.wait_for_service(timeout_sec=_TIMEOUT_SERVICE_WAIT):
                    self.node.get_logger().error('Launch manager service not available')
                    return

                future = self.launch_client.call_async(request)
                response = self._wait_for_service_response(future, _TIMEOUT_SERVICE_RESPONSE)
                if response is not None:
                    if response.success:
                        self.node.get_logger().info(f'Hardware launched: {response.message}')
                        self.launch_processes['hardware'] = True
                    else:
                        self.node.get_logger().error(f'Failed to launch hardware: {response.message}')
                        return
                else:
                    self.node.get_logger().error('Service call failed for hardware launch')
                    return
            else:
                # Local launch
                steam_arg = 'true' if self.steam_mode else 'false'
                self.launch_processes['hardware'] = subprocess.Popen(
                    ['ros2', 'launch', 'lunabot_bringup', 'hardware_launch.py', f'steam_mode:={steam_arg}'],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            self.hardware_enabled = True
            self.node.get_logger().info('Hardware launched')
            
            # Trigger UI update
            if self.on_robot_state_update:
                self.on_robot_state_update()
        except Exception as e:
            self.node.get_logger().error(f'Failed to launch hardware: {str(e)}')
    
    def launch_system(self, system_name):
        """Launch a system (PointLIO, mapping, Nav2, localization)"""
        self.node.get_logger().info(f'Launching {system_name}')
        
        # Check if already running
        if self.launch_processes[system_name] is not None:
            self.node.get_logger().warning(f'{system_name} already running')
            return
        
        try:
            run_on_robot = self.is_real_mode and self.is_remote
            use_sim = not self.is_real_mode
            
            if run_on_robot:
                # Use service call to launch manager on robot
                self.node.get_logger().info(f'Launching {system_name} via launch manager service')
                
                request = LaunchSystem.Request()
                request.system_name = system_name
                request.use_sim = use_sim
                
                if not self.launch_client.wait_for_service(timeout_sec=_TIMEOUT_SERVICE_WAIT_LONG):
                    self.node.get_logger().error('Launch manager service not available')
                    return

                future = self.launch_client.call_async(request)
                response = self._wait_for_service_response(future, _TIMEOUT_SERVICE_RESPONSE)
                if response is not None:
                    if response.success:
                        self.node.get_logger().info(f'{system_name} launched: {response.message}')
                        self.launch_processes[system_name] = True
                    else:
                        self.node.get_logger().error(f'Failed to launch {system_name}: {response.message}')
                        return
                else:
                    self.node.get_logger().error(f'Service call failed for {system_name}')
                    return

                self._set_system_enabled(system_name, True)
            else:
                # Local launch (simulation mode)
                use_sim_arg = 'true' if use_sim else 'false'
                launch_file = f'{system_name}_launch.py'
                if system_name == 'nav2':
                    launch_file = 'nav2_stack_launch.py'

                self.launch_processes[system_name] = subprocess.Popen(
                    ['ros2', 'launch', 'lunabot_bringup', launch_file, f'use_sim:={use_sim_arg}'],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE
                )
                self._set_system_enabled(system_name, True)
            
            location = "via launch manager" if run_on_robot else "locally"
            self.node.get_logger().info(f'{system_name} launched {location}')
            
            # Trigger UI update
            if self.on_robot_state_update:
                self.on_robot_state_update()
        except Exception as e:
            self.node.get_logger().error(f'Failed to launch {system_name}: {e}')
    
    def stop_system(self, system_name):
        """Stop a running system"""
        self.node.get_logger().info(f'Stopping {system_name}')
        
        try:
            if self.launch_processes[system_name] is None:
                self.node.get_logger().warning(f'{system_name} is not running')
                return
            
            # If launch_processes is True, it's a remote process via service
            if self.launch_processes[system_name] is True and self.is_remote:
                self.node.get_logger().info(f'Stopping {system_name} via launch manager service')
                
                request = StopSystem.Request()
                request.system_name = system_name
                
                if not self.stop_client.wait_for_service(timeout_sec=_TIMEOUT_SERVICE_WAIT):
                    self.node.get_logger().error('Stop service not available')
                    return

                future = self.stop_client.call_async(request)
                response = self._wait_for_service_response(future, _TIMEOUT_STOP_RESPONSE)
                if response is not None:
                    if response.success:
                        self.node.get_logger().info(f'{system_name} stopped: {response.message}')
                    else:
                        self.node.get_logger().warn(f'Stop warning: {response.message}')
                else:
                    self.node.get_logger().error(f'Service call failed for stopping {system_name}')
            
            # If it's a subprocess object, terminate it locally
            elif isinstance(self.launch_processes[system_name], subprocess.Popen):
                try:
                    self.launch_processes[system_name].terminate()
                    self.launch_processes[system_name].wait(timeout=_TIMEOUT_PROCESS_TERMINATE)
                except subprocess.TimeoutExpired:
                    self.launch_processes[system_name].kill()

            self.launch_processes[system_name] = None
            self._set_system_enabled(system_name, False)
            
            self.node.get_logger().info(f'{system_name} stopped')
            
            # Trigger UI update
            if self.on_robot_state_update:
                self.on_robot_state_update()
        except Exception as e:
            self.node.get_logger().error(f'Error stopping {system_name}: {e}')
    
    
    def launch_rviz(self):
        """Launch RViz2 with robot visualization config"""
        self.node.get_logger().info('Launching RViz2')
        try:
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
                
                setup_cmd = f'source {workspace_install}/setup.bash'
            else:
                setup_cmd = 'source $(ros2 pkg prefix lunabot_config)/../setup.bash 2>/dev/null || source install/setup.bash'
            
            config_path = '$(ros2 pkg prefix lunabot_config)/share/lunabot_config/rviz/robot_view.rviz'
            
            subprocess.Popen(
                ['gnome-terminal', '--', 'bash', '-c',
                 f'{setup_cmd} && rviz2 -d {config_path}; read -p "Press Enter to close..."'],
            )
            
            self.node.get_logger().info('RViz2 launched (close terminal to stop)')
        except Exception as e:
            self.node.get_logger().error(f'Failed to launch RViz2: {e}')
    
    # Action client methods
    def send_excavate_goal(self, response_callback, result_callback, cancel_callback):
        """Send excavation action goal"""
        self.node.get_logger().info('Sending excavate goal')
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
        """Cancel excavation action"""
        if self.excavation_goal_handle is not None:
            cancel_future = self.excavation_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(cancel_callback)
    
    def send_deposit_goal(self, response_callback, result_callback, cancel_callback):
        """Send depositing action goal"""
        self.node.get_logger().info('Sending deposit goal')
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
        """Cancel depositing action"""
        if self.depositing_goal_handle is not None:
            cancel_future = self.depositing_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(cancel_callback)
    
    def send_home_goal(self, response_callback, result_callback, cancel_callback):
        """Send homing action goal"""
        self.node.get_logger().info('Sending home goal')
        goal_msg = Homing.Goal()
        send_goal_future = self.homing_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self._handle_home_response(future, response_callback, result_callback))
        return send_goal_future
    
    def _handle_home_response(self, future, response_callback, result_callback):
        goal_handle = future.result()
        self.homing_goal_handle = goal_handle
        response_callback(goal_handle)
        if goal_handle.accepted:
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_callback)
    
    def cancel_home_goal(self, cancel_callback):
        """Cancel homing action"""
        if self.homing_goal_handle is not None:
            cancel_future = self.homing_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(cancel_callback)
    
    def launch_navigation_client(self):
        """Launch navigation client for one cycle auto"""
        self.node.get_logger().info('Starting navigation client...')
        try:
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
                
                setup_cmd = f'source {workspace_install}/setup.bash'
            else:
                setup_cmd = 'source $(ros2 pkg prefix lunabot_nav)/../setup.bash 2>/dev/null || source install/setup.bash'
            
            use_sim_arg = 'false' if self.is_real_mode else 'true'
            use_localization_arg = 'false'
            
            self.navigation_client_process = subprocess.Popen(
                ['gnome-terminal', '--', 'bash', '-c',
                 f'{setup_cmd} && ros2 run lunabot_nav navigation_client --ros-args -p use_sim_time:={use_sim_arg} -p use_localization:={use_localization_arg}; read -p "Press Enter to close..."'],
            )
            
            self.node.get_logger().info('Navigation client launched')
            return True
        except Exception as e:
            self.node.get_logger().error(f'Failed to launch navigation client: {e}')
            return False
    
    def stop_navigation_client(self):
        """Stop navigation client process"""
        if self.navigation_client_process:
            try:
                self.navigation_client_process.terminate()
                self.navigation_client_process = None
                self.node.get_logger().info('Navigation client stopped')
            except Exception as e:
                self.node.get_logger().error(f'Failed to stop navigation client: {e}')
    
    # Utility methods
    def spin_once(self):
        """Spin ROS node once to process callbacks"""
        if not self._shutdown and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0)
            except Exception as e:
                if not self._shutdown:
                    self.node.get_logger().debug(f'Spin error: {e}')
    
    def shutdown(self):
        """Shutdown ROS interface"""
        self._shutdown = True
        
        # Terminate any running launch processes
        for name, process in self.launch_processes.items():
            if process is not None:
                try:
                    process.terminate()
                    process.wait(timeout=_TIMEOUT_PROCESS_TERMINATE)
                    self.node.get_logger().info(f'Terminated {name} process')
                except subprocess.TimeoutExpired:
                    process.kill()
                    self.node.get_logger().warn(f'Killed {name} process (force)')
                except Exception as e:
                    self.node.get_logger().warn(f'Error terminating {name}: {e}')
        
        # Stop navigation client
        self.stop_navigation_client()
        
        # Destroy node (don't shutdown rclpy as it may be used by other components)
        try:
            self.node.destroy_node()
        except Exception as e:
            pass
