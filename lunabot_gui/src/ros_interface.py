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
from lunabot_msgs.action import Excavation, Dumping, Homing
from lunabot_msgs.srv import LaunchSystem, StopSystem

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


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
        self.robot_workspace = '~/lunabot_ws'  # Fixed workspace path
        # In real mode, always use remote execution via SSH
        self.is_remote = self.is_real_mode
        
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
        self.is_dumping = False
        self.is_homing = False
        self.is_navigating = False
        self.robot_disabled = False
        self.vibration_state = False
        
        # Launch process tracking (local processes or SSH connections)
        self.launch_processes = {
            'hardware': None,
            'actions': None,
            'pointlio': None,
            'mapping': None,
            'nav2': None,
            'localization': None
        }
        
        # Remote PID tracking for processes launched on robot
        self.remote_pids = {
            'hardware': None,
            'actions': None,
            'pointlio': None,
            'mapping': None,
            'nav2': None,
            'localization': None
        }
        
        # Action clients and goal handles
        self.excavation_client = ActionClient(self.node, Excavation, 'excavation_action')
        self.dumping_client = ActionClient(self.node, Dumping, 'dumping_action')
        self.homing_client = ActionClient(self.node, Homing, 'homing_action')
        
        self.excavation_goal_handle = None
        self.dumping_goal_handle = None
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
                self._front_camera_callback, 10)
            self.node.create_subscription(
                CompressedImage, '/camera_back/color/image_compressed',
                self._rear_camera_callback, 10)
            self.node.create_subscription(
                CompressedImage, '/camera_fisheye/color/image_compressed',
                self._fisheye_camera_callback, 10)
        
        # Robot odometry
        self.node.create_subscription(
            Odometry, '/odometry/filtered', self._odom_callback, 10)
        
        # Velocity commands
        self.node.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        
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
        
        # Vibration state
        self.node.create_subscription(
            Bool, '/vibration_state', self._vibration_state_callback, 10)
        
        # Joint states
        self.node.create_subscription(
            JointState, '/joint_states', self._joint_states_callback, 10)
    
    # Camera callbacks
    def _front_camera_callback(self, msg):
        try:
            import numpy as np
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.front_camera_image = cv_image
            if self.on_camera_update:
                self.on_camera_update('front', cv_image)
        except Exception as e:
            self.node.get_logger().error(f'Front camera error: {e}')
    
    def _rear_camera_callback(self, msg):
        try:
            import numpy as np
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.rear_camera_image = cv_image
            if self.on_camera_update:
                self.on_camera_update('rear', cv_image)
        except Exception as e:
            self.node.get_logger().error(f'Rear camera error: {e}')
    
    def _fisheye_camera_callback(self, msg):
        try:
            import numpy as np
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.fisheye_camera_image = cv_image
            if self.on_camera_update:
                self.on_camera_update('fisheye', cv_image)
        except Exception as e:
            self.node.get_logger().error(f'Fisheye camera error: {e}')
    
    def _odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z
        if self.on_robot_state_update:
            self.on_robot_state_update()
    
    def _cmd_vel_callback(self, msg):
        pass  # Use odom for actual velocity display
    
    def _manual_mode_callback(self, msg):
        is_manual = msg.data
        self.robot_mode = "MANUAL" if is_manual else "AUTO"
        if self.on_robot_state_update:
            self.on_robot_state_update()
    
    def _robot_disabled_callback(self, msg):
        self.robot_disabled = msg.data
        if self.on_robot_state_update:
            self.on_robot_state_update()
    
    def _vibration_state_callback(self, msg):
        self.vibration_state = msg.data
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
        self.is_dumping = msg.is_dumping
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
        msg.data = True
        self.mode_switch_pub.publish(msg)
        self.node.get_logger().info('Mode switch requested')
    
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
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
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
                
                # Check if service exists first
                service_names = [name for name, _ in self.node.get_service_names_and_types()]
                if '/launch_system' not in service_names:
                    self.node.get_logger().error(f'Launch manager service not in service list. Available: {service_names}')
                    return
                
                # Wait for service with retry
                max_attempts = 3
                for attempt in range(max_attempts):
                    self.node.get_logger().info(f'Waiting for /launch_system service (attempt {attempt+1}/{max_attempts})...')
                    if self.launch_client.wait_for_service(timeout_sec=5.0):
                        self.node.get_logger().info('Service found!')
                        break
                    self.node.get_logger().warn(f'Service not available yet, retrying...')
                    QCoreApplication.processEvents()
                    time.sleep(1)
                else:
                    self.node.get_logger().error('Launch manager service not available after retries')
                    return
                
                future = self.launch_client.call_async(request)
                
                # Wait for response (process Qt events so ROS timer can spin the node)
                start_time = time.time()
                timeout = 30.0
                while not future.done() and (time.time() - start_time) < timeout:
                    QCoreApplication.processEvents()  # Let Qt event loop run (including ROS timer)
                    time.sleep(0.01)
                
                if future.done() and future.result() is not None:
                    response = future.result()
                    if response.success:
                        self.node.get_logger().info(f'Hardware launched: {response.message}')
                        self.launch_processes['hardware'] = True  # Mark as running
                    else:
                        self.node.get_logger().error(f'Failed to launch hardware: {response.message}')
                        return
                else:
                    self.node.get_logger().error('Service call failed for hardware launch')
                    return
            else:
                # Local launch
                self.launch_processes['hardware'] = subprocess.Popen(
                    ['ros2', 'launch', 'lunabot_bringup', 'hardware_launch.py'],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            self.hardware_enabled = True
            self.node.get_logger().info('Hardware launched')
        except Exception as e:
            self.node.get_logger().error(f'Failed to launch hardware: {str(e)}')
    
    def launch_system(self, system_name):
        """Launch a system (PointLIO, mapping, Nav2, localization)"""
        self.node.get_logger().info(f'Launching {system_name}')
        
        # Check if already running
        if self.launch_processes[system_name] is not None or self.remote_pids[system_name] is not None:
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
                
                if not self.launch_client.wait_for_service(timeout_sec=10.0):
                    self.node.get_logger().error('Launch manager service not available')
                    return
                
                future = self.launch_client.call_async(request)
                
                # Wait for response (process Qt events so ROS timer can spin the node)
                start_time = time.time()
                timeout = 30.0
                while not future.done() and (time.time() - start_time) < timeout:
                    QCoreApplication.processEvents()  # Let Qt event loop run (including ROS timer)
                    time.sleep(0.01)
                
                if future.done() and future.result() is not None:
                    response = future.result()
                    if response.success:
                        self.node.get_logger().info(f'{system_name} launched: {response.message}')
                        # Mark as running with placeholder PID
                        self.remote_pids[system_name] = 1
                    else:
                        self.node.get_logger().error(f'Failed to launch {system_name}: {response.message}')
                        return
                else:
                    self.node.get_logger().error(f'Service call failed for {system_name}')
                    return
                
                # Set enabled flags
                if system_name == 'pointlio':
                    self.pointlio_enabled = True
                elif system_name == 'mapping':
                    self.mapping_enabled = True
                elif system_name == 'nav2':
                    self.nav2_enabled = True
                elif system_name == 'localization':
                    self.localization_enabled = True
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
                
                # Set enabled flags
                if system_name == 'pointlio':
                    self.pointlio_enabled = True
                elif system_name == 'mapping':
                    self.mapping_enabled = True
                elif system_name == 'nav2':
                    self.nav2_enabled = True
                elif system_name == 'localization':
                    self.localization_enabled = True
            
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
            run_on_robot = self.is_real_mode and self.is_remote
            
            # Stop remote process via service
            if self.remote_pids[system_name] is not None:
                if run_on_robot:
                    self.node.get_logger().info(f'Stopping {system_name} via launch manager service')
                    
                    request = StopSystem.Request()
                    request.system_name = system_name
                    
                    if not self.stop_client.wait_for_service(timeout_sec=10.0):
                        self.node.get_logger().error('Stop service not available')
                        return
                    
                    future = self.stop_client.call_async(request)
                    
                    # Wait for response (process Qt events so ROS timer can spin the node)
                    start_time = time.time()
                    timeout = 10.0
                    while not future.done() and (time.time() - start_time) < timeout:
                        QCoreApplication.processEvents()  # Let Qt event loop run (including ROS timer)
                        time.sleep(0.01)
                    
                    if future.done() and future.result() is not None:
                        response = future.result()
                        if response.success:
                            self.node.get_logger().info(f'{system_name} stopped: {response.message}')
                        else:
                            self.node.get_logger().warn(f'Stop warning: {response.message}')
                    else:
                        self.node.get_logger().error(f'Service call failed for stopping {system_name}')
                
                self.remote_pids[system_name] = None
            
            # Terminate local process if exists
            if self.launch_processes[system_name] is not None:
                try:
                    self.launch_processes[system_name].terminate()
                    self.launch_processes[system_name].wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.launch_processes[system_name].kill()
                self.launch_processes[system_name] = None
            
            # Update enabled flags
            if system_name == 'pointlio':
                self.pointlio_enabled = False
            elif system_name == 'mapping':
                self.mapping_enabled = False
            elif system_name == 'nav2':
                self.nav2_enabled = False
            elif system_name == 'localization':
                self.localization_enabled = False
            
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
    
    def send_dump_goal(self, response_callback, result_callback, cancel_callback):
        """Send dumping action goal"""
        self.node.get_logger().info('Sending dump goal')
        goal_msg = Dumping.Goal()
        send_goal_future = self.dumping_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self._handle_dump_response(future, response_callback, result_callback))
        return send_goal_future
    
    def _handle_dump_response(self, future, response_callback, result_callback):
        goal_handle = future.result()
        self.dumping_goal_handle = goal_handle
        response_callback(goal_handle)
        if goal_handle.accepted:
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_callback)
    
    def cancel_dump_goal(self, cancel_callback):
        """Cancel dumping action"""
        if self.dumping_goal_handle is not None:
            cancel_future = self.dumping_goal_handle.cancel_goal_async()
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
                    process.wait(timeout=3)
                    self.node.get_logger().info(f'Terminated {name} process')
                except subprocess.TimeoutExpired:
                    process.kill()
                    self.node.get_logger().warn(f'Killed {name} process (force)')
                except Exception as e:
                    self.node.get_logger().warn(f'Error terminating {name}: {e}')
        
        # Kill any remote processes
        for name, pid in self.remote_pids.items():
            if pid is not None:
                self._kill_remote_process(pid, name)
        
        # Stop navigation client
        self.stop_navigation_client()
        
        # Destroy node (don't shutdown rclpy as it may be used by other components)
        try:
            self.node.destroy_node()
        except Exception as e:
            pass
    
    def _kill_remote_process(self, pid, name="process"):
        """Kill a process on the remote robot by name pattern"""
        if not self.is_remote:
            return
        
        # Map system names to launch file patterns for pkill
        launch_patterns = {
            'pointlio': 'pointlio_launch',
            'mapping': 'mapping_launch',
            'nav2': 'nav2_stack_launch',
            'localization': 'localization_launch'
        }
        
        pattern = launch_patterns.get(name, name)
        
        try:
            # Use pkill to kill by process name
            cmd = ['ssh', f'{self.robot_user}@{self.robot_host}', f'pkill -9 -f "{pattern}"']
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            # pkill returns 0 if processes were killed, 1 if none found
            if result.returncode == 0:
                self.node.get_logger().info(f'Killed remote {name} process')
            else:
                self.node.get_logger().warn(f'No remote {name} process found to kill')
        except Exception as e:
            self.node.get_logger().error(f'Error killing remote {name}: {e}')
    
    def _launch_remote_command(self, command, name):
        """Launch a command on remote robot and track PID"""
        if not self.is_remote:
            return None, None
        
        # Launch in true fire-and-forget mode - don't wait for PID
        # The process will run, we just won't track the exact PID
        remote_cmd = (
            f"bash -c 'source {self.robot_workspace}/install/setup.bash && "
            f"{command} > /tmp/{name}.log 2>&1 &' &"
        )
        
        try:
            # Launch without waiting - true fire and forget
            subprocess.Popen(
                ['ssh', '-f', f'{self.robot_user}@{self.robot_host}', remote_cmd],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.node.get_logger().info(f'Launched {name} on remote (no PID tracking)')
            # Return a placeholder PID of 1 to indicate it's running
            return None, 1
        except Exception as e:
            self.node.get_logger().error(f'Error launching {name}: {e}')
            return None, None
