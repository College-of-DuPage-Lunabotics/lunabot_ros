#!/usr/bin/env python3
"""
Lunabot PyQt5 GUI
Custom control interface for NASA Lunabotics competition robot
"""

import sys
import os
import signal
import subprocess
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QLabel, QPushButton, QGridLayout, 
                              QGroupBox, QProgressBar, QCheckBox, QScrollArea, QSizePolicy)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap, QFont, QPalette, QColor

# Import styling constants
from gui_styles import Colors, Styles, MAIN_STYLESHEET

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32, String, Bool
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray
from lunabot_msgs.msg import ControlState
from lunabot_msgs.action import Excavation, Dumping, Homing

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False
    print("Warning: cv_bridge or cv2 not available. Camera display disabled.")



class LunabotGUI(QMainWindow):
    """Main GUI window for Lunabot control and monitoring"""
    
    def __init__(self):
        super().__init__()
        
        # Initialize ROS
        rclpy.init()
        self.node = Node('lunabot_gui')
        self._shutdown = False  # Flag to track shutdown state
        
        # Declare and get mode parameter (accepts 'sim'/'real' or true/false)
        self.node.declare_parameter('mode', True)  # Default to sim mode (True = sim, False = real)
        mode_param = self.node.get_parameter('mode').value
        
        # Handle various formats
        if isinstance(mode_param, bool):
            self.is_real_mode = not mode_param  # False = real mode when it's use_sim
        elif isinstance(mode_param, str):
            mode_lower = mode_param.lower()
            # 'true' or 'sim' = simulation (real_mode=False)
            # 'false' or 'real' = real mode (real_mode=True)
            self.is_real_mode = (mode_lower == 'false' or mode_lower == 'real')
        else:
            self.is_real_mode = False  # Default to sim mode
        
        self.robot_mode_type = 'real' if self.is_real_mode else 'sim'
        self.node.get_logger().info(f'GUI running in {self.robot_mode_type.upper()} mode')
        
        # Remote robot parameters (for SSH execution of hardware/CAN commands)
        self.node.declare_parameter('robot_host', 'localhost')  # Robot hostname or IP
        self.node.declare_parameter('robot_user', os.environ.get('USER', 'user'))  # SSH username
        self.node.declare_parameter('robot_workspace', '~/lunabot_ws')  # Workspace path on robot
        
        self.robot_host = self.node.get_parameter('robot_host').value
        self.robot_user = self.node.get_parameter('robot_user').value
        self.robot_workspace = self.node.get_parameter('robot_workspace').value
        self.is_remote = (self.robot_host != 'localhost')  # Run commands remotely via SSH
        
        if self.is_remote:
            self.node.get_logger().info(f'Remote robot: {self.robot_user}@{self.robot_host}:{self.robot_workspace}')
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge() if CV_AVAILABLE else None
        
        # Data storage - bandwidth
        self.bandwidth_total = 0.0
        self.bandwidth_rx = 0.0
        self.bandwidth_tx = 0.0
        self.bandwidth_status = "Initializing..."
        
        # Data storage - robot state
        self.front_camera_image = None
        self.rear_camera_image = None
        self.fisheye_camera_image = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        
        # System enable states
        self.keyboard_teleop_enabled = False
        self.hardware_enabled = False
        self.pointlio_enabled = False
        self.mapping_enabled = False
        self.nav2_enabled = False
        self.localization_enabled = False
        
        # Robot mode and operation states
        self.robot_mode = "MANUAL"  # MANUAL or AUTO
        self.is_excavating = False
        self.is_dumping = False
        self.is_homing = False
        self.is_navigating = False
        
        # Camera control
        self.fisheye_camera_position = 0.0  # Current position in radians
        self.swappable_camera_showing_front = True  # True = Front, False = Rear
        
        # Sidebar state
        self.sidebar_collapsed = False
        
        # Launch process tracking
        self.launch_processes = {
            'hardware': None,
            'actions': None,
            'pointlio': None,
            'mapping': None,
            'nav2': None,
            'localization': None
        }
        
        # Create action clients
        self.excavation_client = ActionClient(self.node, Excavation, 'excavation_action')
        self.dumping_client = ActionClient(self.node, Dumping, 'dumping_action')
        self.homing_client = ActionClient(self.node, Homing, 'homing_action')
        
        # Goal handle tracking for cancellation
        self.excavation_goal_handle = None
        self.dumping_goal_handle = None
        self.homing_goal_handle = None
        self.full_auto_active = False
        self.navigation_client_process = None
        
        # Create ROS publishers
        self.emergency_stop_pub = self.node.create_publisher(Bool, '/emergency_stop', 10)
        self.control_state_pub = self.node.create_publisher(ControlState, '/control_state', 10)
        self.mode_switch_pub = self.node.create_publisher(Bool, '/mode_switch', 10)
        self.position_cmd_pub = self.node.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.camera_cmd_pub = self.node.create_publisher(Float64MultiArray, '/camera_controller/commands', 10)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Teleop button states and speeds
        self.teleop_keys = {
            'w': False, 'a': False, 's': False, 'd': False,
            'up': False, 'down': False
        }
        self.bucket_position = -0.2  # Initial bucket position
        self.linear_speed = 0.35  # m/s (matching keyboard_teleop)
        self.angular_speed = 0.25  # rad/s (matching keyboard_teleop)
        self.max_linear_speed = 0.75
        self.max_angular_speed = 0.75
        
        # Create ROS subscribers
        self.create_subscriptions()
        
        # Setup UI
        self.init_ui()
        
        # Setup ROS spin timer
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)  # 100 Hz
        
        # Setup UI update timer
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(100)  # 10 Hz
    
    def create_subscriptions(self):
        """Create all ROS topic subscriptions"""
        # Bandwidth monitoring
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
            String, '/bandwidth/status', 
            lambda msg: setattr(self, 'bandwidth_status', msg.data), 10)
        
        # Camera feeds
        if CV_AVAILABLE:
            self.node.create_subscription(
                Image, '/camera_front/color/image_raw', 
                self.front_camera_callback, 10)
            self.node.create_subscription(
                Image, '/camera_back/color/image_raw', 
                self.rear_camera_callback, 10)
            self.node.create_subscription(
                Image, '/camera_fisheye/color/image_raw', 
                self.fisheye_camera_callback, 10)
        
        # Robot odometry (from UKF fusion)
        self.node.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # Velocity commands (echo for display)
        self.node.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Control state
        self.node.create_subscription(
            ControlState, '/control_state', self.control_state_callback, 10)
        
        # Robot mode from controller_teleop
        self.node.create_subscription(
            Bool, '/manual_mode', self.manual_mode_callback, 10)
        
        # Robot disabled status from controller_teleop
        self.node.create_subscription(
            Bool, '/robot_disabled', self.robot_disabled_callback, 10)
        
        # Joint states for bucket angle feedback
        self.node.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
    
    def front_camera_callback(self, msg):
        """Handle front camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.front_camera_image = cv_image
        except Exception as e:
            self.node.get_logger().error(f'Front camera error: {e}')
    
    def rear_camera_callback(self, msg):
        """Handle rear camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rear_camera_image = cv_image
        except Exception as e:
            self.node.get_logger().error(f'Rear camera error: {e}')
    
    def fisheye_camera_callback(self, msg):
        """Handle fisheye camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.fisheye_camera_image = cv_image
        except Exception as e:
            self.node.get_logger().error(f'Fisheye camera error: {e}')
    
    def odom_callback(self, msg):
        """Handle odometry data"""
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        # Get actual velocity from odometry
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z
    
    def cmd_vel_callback(self, msg):
        """Handle velocity command echo (not used for display)"""
        pass  # We use odom for actual velocity display
    
    def manual_mode_callback(self, msg):
        """Handle manual mode state from controller_teleop"""
        is_manual = msg.data
        mode_text = "MANUAL" if is_manual else "AUTO"
        self.robot_mode = mode_text
        
        # Only update UI if mode controls exist and in real mode (sim mode always shows "Simulation")
        if hasattr(self, 'mode_label') and self.is_real_mode:
            # Display text formatting
            display_text = "Manual" if is_manual else "AUTO"
            self.mode_label.setText(display_text)
            
            # Update styling and button text
            if is_manual:
                self.mode_label.setStyleSheet("color: #66bb6a; font-weight: bold; font-size: 14px; background-color: transparent;")
                self.mode_switch_btn.setText("Switch to Auto")
            else:
                self.mode_label.setStyleSheet("color: #d32f2f; font-weight: bold; font-size: 14px; background-color: transparent;")
                self.mode_switch_btn.setText("Switch to Manual")
    
    def robot_disabled_callback(self, msg):
        """Handle robot disabled state from controller_teleop"""
        is_disabled = msg.data
        
        if is_disabled:
            self.status_label.setText("Robot: DISABLED")
            self.status_label.setStyleSheet("color: #d32f2f; font-weight: bold;")
            
            # Cancel any active operations
            if self.excavation_goal_handle is not None:
                self.excavation_goal_handle.cancel_goal_async()
            if self.dumping_goal_handle is not None:
                self.dumping_goal_handle.cancel_goal_async()
            
            # Reset button states
            self.is_excavating = False
            self.is_dumping = False
            self.full_auto_active = False
            self.excavate_btn.setText("Excavate")
            self.excavate_btn.setEnabled(False)
            self.dump_btn.setText("Dump")
            self.dump_btn.setEnabled(False)
            self.auto_btn.setText("One Cycle Auto")
            self.auto_btn.setEnabled(False)
        else:
            self.status_label.setText("Active")
            self.status_label.setStyleSheet("color: #66bb6a; font-weight: bold;")
            # Re-enable buttons if not in operation
            if not (self.is_excavating or self.is_dumping or self.is_navigating):
                self.excavate_btn.setEnabled(True)
                self.dump_btn.setEnabled(True)
                self.auto_btn.setEnabled(True)
    
    def joint_states_callback(self, msg):
        """Handle joint states to update bucket angle"""
        try:
            # Find base_bucket_joint in the joint names
            if 'base_bucket_joint' in msg.name:
                idx = msg.name.index('base_bucket_joint')
                # Update bucket position from joint state feedback
                if idx < len(msg.position):
                    self.bucket_position = msg.position[idx]
        except Exception as e:
            self.node.get_logger().warn(f'Error processing joint states: {e}', throttle_duration_sec=5.0)
    
    def control_state_callback(self, msg):
        """Handle control state messages"""
        # Update robot mode
        mode_text = "MANUAL" if msg.mode == 0 else "AUTO"
        self.robot_mode = mode_text
        
        # Only update UI if mode controls exist (real mode only)
        if hasattr(self, 'mode_label'):
            self.mode_label.setText(mode_text)
            
            if msg.mode == 0:  # MANUAL
                self.mode_label.setStyleSheet("color: #66bb6a; padding: 10px;")
            else:  # AUTO
                self.mode_label.setStyleSheet("color: #2196f3; padding: 10px;")
        
        # Update system states if different
        if msg.hardware_enabled != self.hardware_enabled:
            self.hardware_enabled = msg.hardware_enabled
            if hasattr(self, 'hardware_checkbox'):
                self.hardware_checkbox.setChecked(msg.hardware_enabled)
        
        if msg.pointlio_enabled != self.pointlio_enabled:
            self.pointlio_enabled = msg.pointlio_enabled
            if hasattr(self, 'pointlio_checkbox'):
                self.pointlio_checkbox.setChecked(msg.pointlio_enabled)
        
        if msg.mapping_enabled != self.mapping_enabled:
            self.mapping_enabled = msg.mapping_enabled
            if hasattr(self, 'mapping_checkbox'):
                self.mapping_checkbox.setChecked(msg.mapping_enabled)
        
        if msg.nav2_enabled != self.nav2_enabled:
            self.nav2_enabled = msg.nav2_enabled
            if hasattr(self, 'nav2_checkbox'):
                self.nav2_checkbox.setChecked(msg.nav2_enabled)
        
        if msg.localization_enabled != self.localization_enabled:
            self.localization_enabled = msg.localization_enabled
            if hasattr(self, 'localization_checkbox'):
                self.localization_checkbox.setChecked(msg.localization_enabled)
        
        # Update operation states
        self.is_excavating = msg.is_excavating
        self.is_dumping = msg.is_dumping
        self.is_navigating = msg.is_navigating
        
        # Update button texts based on operation states
        if msg.is_excavating:
            self.excavate_btn.setText("Cancel Excavation")
        else:
            self.excavate_btn.setText("Excavate")
        
        if msg.is_dumping:
            self.dump_btn.setText("Cancel Dump")
        else:
            self.dump_btn.setText("Dump")
        
        # Update operation status display
        if msg.is_excavating:
            self.operation_status_label.setText("Status: Excavating")
            self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
        elif msg.is_dumping:
            self.operation_status_label.setText("Status: Dumping")
            self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
        elif msg.is_navigating:
            self.operation_status_label.setText("Status: Navigating")
            self.operation_status_label.setStyleSheet("color: #2196f3; background-color: transparent;")
        elif msg.status_message:
            self.operation_status_label.setText(f"Status: {msg.status_message}")
            self.operation_status_label.setStyleSheet("color: #aaa; background-color: transparent;")
        else:
            self.operation_status_label.setText("Status: Idle")
            self.operation_status_label.setStyleSheet("color: #aaa; background-color: transparent;")
        
        # Handle emergency stop
        if msg.emergency_stop:
            self.status_label.setText("Robot: EMERGENCY STOP")
            self.status_label.setStyleSheet("color: #d32f2f; font-weight: bold;")
            self.excavate_btn.setEnabled(False)
            self.dump_btn.setEnabled(False)
            self.auto_btn.setEnabled(False)
        else:
            self.status_label.setText("Active")
            self.status_label.setStyleSheet("color: #66bb6a; font-weight: bold;")
            # Re-enable buttons if not in operation
            if not (msg.is_excavating or msg.is_dumping or msg.is_navigating):
                self.excavate_btn.setEnabled(True)
                self.dump_btn.setEnabled(True)
                self.auto_btn.setEnabled(True)
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('Lunabot Control Panel')
        self.setGeometry(100, 100, 1600, 900)
        
        # Set dark mode styling for group boxes
        self.setStyleSheet(MAIN_STYLESHEET)
        
        # Main widget and layout
        main_widget = QWidget()
        main_widget.setObjectName("centralWidget")
        main_widget.setStyleSheet("background-color: #1a1a1a;")
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(6)
        main_widget.setLayout(main_layout)
        
        # ===== MAIN CONTENT AREA =====
        content_widget = QWidget()
        content_layout = QVBoxLayout()
        content_layout.setContentsMargins(4, 0, 0, 4)
        content_layout.setSpacing(10)  # Spacing between cameras and bottom row
        content_widget.setLayout(content_layout)
        
        # TOP ROW: Cameras
        top_row = QWidget()
        top_row_layout = QHBoxLayout()
        top_row_layout.setContentsMargins(0, 0, 0, 0)
        top_row_layout.setSpacing(10)
        top_row.setLayout(top_row_layout)
        
        # Left: Swappable Camera (Front/Rear) with swap button
        swappable_camera_container = QWidget()
        swappable_layout = QVBoxLayout()
        swappable_layout.setContentsMargins(0, 0, 0, 0)
        swappable_camera_container.setLayout(swappable_layout)
        
        self.swappable_camera_group = QGroupBox("Front Camera")
        self.swappable_camera_group.setAutoFillBackground(True)
        self.swappable_camera_group.setStyleSheet("QGroupBox { background-color: #2d2d2d; }")
        swappable_camera_layout = QVBoxLayout()
        self.swappable_camera_label = QLabel("No camera feed")
        self.swappable_camera_label.setAlignment(Qt.AlignCenter)
        self.swappable_camera_label.setMinimumSize(400, 300)
        self.swappable_camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.swappable_camera_label.setScaledContents(False)
        self.swappable_camera_label.setStyleSheet(Styles.camera_label())
        swappable_camera_layout.addWidget(self.swappable_camera_label)
        self.swappable_camera_group.setLayout(swappable_camera_layout)
        swappable_layout.addWidget(self.swappable_camera_group)
        
        self.swap_camera_btn = QPushButton("⇄  Rear Camera")
        self.swap_camera_btn.setStyleSheet("""
            QPushButton {
                background-color: #2d2d2d;
                color: #4da3f0;
                border: none;
                border-top: 2px solid #4da3f0;
                font-size: 12px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #3a3a3a;
                color: #66bb6a;
                border-top: 2px solid #66bb6a;
            }
        """)
        self.swap_camera_btn.setMaximumHeight(30)
        self.swap_camera_btn.clicked.connect(self.swap_camera)
        swappable_layout.addWidget(self.swap_camera_btn)
        
        top_row_layout.addWidget(swappable_camera_container, 1)
        
        # Right: Fisheye Camera
        fisheye_group = QGroupBox("Fisheye Camera")
        fisheye_group.setAutoFillBackground(True)
        fisheye_group.setStyleSheet("QGroupBox { background-color: #2d2d2d; }")
        fisheye_layout = QVBoxLayout()
        self.fisheye_camera_label = QLabel("No camera feed")
        self.fisheye_camera_label.setAlignment(Qt.AlignCenter)
        self.fisheye_camera_label.setMinimumSize(400, 300)
        self.fisheye_camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.fisheye_camera_label.setScaledContents(False)
        self.fisheye_camera_label.setStyleSheet(Styles.camera_label())
        fisheye_layout.addWidget(self.fisheye_camera_label)
        fisheye_group.setLayout(fisheye_layout)
        top_row_layout.addWidget(fisheye_group, 1)
        
        content_layout.addWidget(top_row, 3)  # Give cameras more space
        
        # BOTTOM ROW: Robot Status (top), Telemetry, Fisheye Rotation
        bottom_row = QWidget()
        bottom_row_layout = QHBoxLayout()
        bottom_row_layout.setContentsMargins(0, 0, 0, 0)
        bottom_row_layout.setSpacing(10)
        bottom_row_layout.setAlignment(Qt.AlignBottom)
        bottom_row.setLayout(bottom_row_layout)
        
        # Left Column: Robot Status above Telemetry
        left_column = QWidget()
        left_column_layout = QVBoxLayout()
        left_column_layout.setContentsMargins(0, 0, 0, 0)
        left_column_layout.setSpacing(10)
        left_column.setLayout(left_column_layout)
        
        # Robot Status + Mode side by side at top
        status_mode_container = QWidget()
        status_mode_layout = QHBoxLayout()
        status_mode_layout.setContentsMargins(0, 0, 0, 0)
        status_mode_layout.setSpacing(10)
        status_mode_container.setLayout(status_mode_layout)
        
        status_group = self.create_status_group()
        status_mode_layout.addWidget(status_group, 1)
        
        mode_group = self.create_mode_group()
        status_mode_layout.addWidget(mode_group, 1)
        if not self.is_real_mode:
            mode_group.setEnabled(False)  # Grey out in sim mode
        
        left_column_layout.addWidget(status_mode_container)
        
        # Telemetry below
        telemetry_group = self.create_combined_telemetry_group()
        left_column_layout.addWidget(telemetry_group)
        
        bottom_row_layout.addWidget(left_column, 2)
        
        # Right: Fisheye Rotation Controls
        camera_group = self.create_camera_control_group()
        bottom_row_layout.addWidget(camera_group, 1)
        
        content_layout.addWidget(bottom_row, 1)  # Less space for bottom info
        
        main_layout.addWidget(content_widget, 4)  # Content takes 80% of width
        
        # ===== RIGHT SIDEBAR WITH EDGE TAB =====
        self.sidebar_container = QWidget()
        sidebar_container_layout = QHBoxLayout()
        sidebar_container_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_container_layout.setSpacing(0)
        self.sidebar_container.setLayout(sidebar_container_layout)
        
        # Sidebar content
        self.sidebar_widget = QWidget()
        self.sidebar_widget.setStyleSheet("background-color: #1a1a1a;")
        self.sidebar_widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        sidebar_layout = QVBoxLayout()
        sidebar_layout.setContentsMargins(4, 4, 4, 4)
        sidebar_layout.setSpacing(10)
        self.sidebar_widget.setLayout(sidebar_layout)
        
        # Add control groups to sidebar in order: Hardware, System, Actions, Teleop
        hardware_group = self.create_hardware_group()
        sidebar_layout.addWidget(hardware_group)
        if not self.is_real_mode:
            hardware_group.setEnabled(False)  # Grey out in sim mode
        
        system_group = self.create_system_control_group()
        sidebar_layout.addWidget(system_group)
        
        action_group = self.create_action_control_group()
        sidebar_layout.addWidget(action_group)
        
        teleop_group = self.create_teleop_control_group()
        sidebar_layout.addWidget(teleop_group)
        if self.is_real_mode:
            teleop_group.setEnabled(False)  # Grey out in real mode
        
        sidebar_container_layout.addWidget(self.sidebar_widget, 0, Qt.AlignBottom)
        
        # Edge tab (always visible, on the right edge)
        self.edge_tab = QPushButton("▶\n\nC\nO\nN\nT\nR\nO\nL\nS")
        self.edge_tab.setStyleSheet("""
            QPushButton {
                background-color: #2d2d2d;
                color: #4da3f0;
                border: none;
                border-left: 2px solid #4da3f0;
                font-size: 12px;
                font-weight: bold;
                padding: 10px 5px;
                letter-spacing: 2px;
            }
            QPushButton:hover {
                background-color: #3a3a3a;
                color: #66bb6a;
                border-left: 2px solid #66bb6a;
            }
        """)
        self.edge_tab.setMaximumWidth(30)
        self.edge_tab.clicked.connect(self.toggle_sidebar)
        sidebar_container_layout.addWidget(self.edge_tab, 0, Qt.AlignVCenter)
        
        main_layout.addWidget(self.sidebar_container, 1)  # Sidebar takes 20% of width
        
        # Store references for camera feeds (now referenced differently)
        self.front_camera_label = None  # Will be set by update method
        self.rear_camera_label = None   # Will be set by update method
    
    def swap_camera(self):
        """Toggle between front and rear camera in the swappable display"""
        self.swappable_camera_showing_front = not self.swappable_camera_showing_front
        if self.swappable_camera_showing_front:
            self.swappable_camera_group.setTitle("Front Camera")
            self.swap_camera_btn.setText("⇄  Rear Camera")
        else:
            self.swappable_camera_group.setTitle("Rear Camera")
            self.swap_camera_btn.setText("⇄  Front Camera")
        # Update display immediately
        self.update_ui()
    
    def toggle_sidebar(self):
        """Toggle sidebar visibility"""
        self.sidebar_collapsed = not self.sidebar_collapsed
        if self.sidebar_collapsed:
            self.sidebar_widget.hide()
            self.edge_tab.setText("◀\n\nC\nO\nN\nT\nR\nO\nL\nS")
            self.sidebar_container.setMaximumWidth(30)  # Only edge tab width
        else:
            self.sidebar_widget.show()
            self.edge_tab.setText("▶\n\nC\nO\nN\nT\nR\nO\nL\nS")
            self.sidebar_container.setMaximumWidth(16777215)  # Qt's QWIDGETSIZE_MAX
    
    def create_combined_telemetry_group(self):
        """Create telemetry display with sectioned layout"""
        group = QGroupBox("Telemetry")
        group.setAutoFillBackground(True)
        group.setStyleSheet("QGroupBox { background-color: #2d2d2d; }")
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(4, 10, 4, 4)
        main_layout.setSpacing(7)
        group.setLayout(main_layout)
        
        # Column 1: Bucket Angle and Bandwidth
        col1_layout = QVBoxLayout()
        col1_layout.setSpacing(7)
        
        # Bucket Angle Section
        bucket_group = QGroupBox("Bucket Angle")
        bucket_group.setAutoFillBackground(True)
        bucket_group.setStyleSheet("QGroupBox { background-color: #3a3a3a; }")
        bucket_layout = QVBoxLayout()
        bucket_layout.setSpacing(2)
        bucket_layout.setContentsMargins(6, 6, 6, 6)
        
        bucket_value_container = QHBoxLayout()
        self.bucket_angle_label = QLabel("0.0°")
        self.bucket_angle_label.setFont(QFont("Monospace", 10))
        self.bucket_angle_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        bucket_value_container.addWidget(self.bucket_angle_label)
        bucket_value_container.addStretch()
        bucket_layout.addLayout(bucket_value_container)
        
        bucket_group.setLayout(bucket_layout)
        col1_layout.addWidget(bucket_group, 1)
        
        # Bandwidth Section
        bandwidth_group = QGroupBox("Bandwidth")
        bandwidth_group.setAutoFillBackground(True)
        bandwidth_group.setStyleSheet("QGroupBox { background-color: #3a3a3a; }")
        bandwidth_layout = QVBoxLayout()
        bandwidth_layout.setSpacing(2)
        bandwidth_layout.setContentsMargins(6, 6, 6, 6)
        
        # All bandwidth values on one line
        values_container = QHBoxLayout()
        values_container.setSpacing(10)
        
        # Total
        total_label = QLabel("Total:")
        total_label.setFont(QFont("Monospace", 10))
        total_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        values_container.addWidget(total_label)
        self.bandwidth_total_label = QLabel("0.00 Mbps")
        self.bandwidth_total_label.setFont(QFont("Monospace", 10))
        self.bandwidth_total_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        values_container.addWidget(self.bandwidth_total_label)
        
        values_container.addSpacing(10)
        
        # RX
        rx_label = QLabel("RX:")
        rx_label.setFont(QFont("Monospace", 10))
        rx_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        values_container.addWidget(rx_label)
        self.bandwidth_rx_label = QLabel("0.00 Mbps")
        self.bandwidth_rx_label.setFont(QFont("Monospace", 10))
        self.bandwidth_rx_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        values_container.addWidget(self.bandwidth_rx_label)
        
        values_container.addSpacing(10)
        
        # TX
        tx_label = QLabel("TX:")
        tx_label.setFont(QFont("Monospace", 10))
        tx_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        values_container.addWidget(tx_label)
        self.bandwidth_tx_label = QLabel("0.00 Mbps")
        self.bandwidth_tx_label.setFont(QFont("Monospace", 10))
        self.bandwidth_tx_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        values_container.addWidget(self.bandwidth_tx_label)
        
        values_container.addStretch()
        bandwidth_layout.addLayout(values_container)
        
        # Add small spacing before progress bar
        bandwidth_layout.addSpacing(4)
        
        # Progress bar for bandwidth (at bottom)
        self.bandwidth_progress = QProgressBar()
        self.bandwidth_progress.setMaximum(100)
        self.bandwidth_progress.setValue(0)
        self.bandwidth_progress.setTextVisible(False)
        self.bandwidth_progress.setMaximumHeight(10)
        bandwidth_layout.addWidget(self.bandwidth_progress)
        
        bandwidth_group.setLayout(bandwidth_layout)
        col1_layout.addWidget(bandwidth_group, 1)
        
        main_layout.addLayout(col1_layout, 1)
        
        # Column 2: Velocity and Position
        col2_layout = QVBoxLayout()
        col2_layout.setSpacing(7)
        
        # Velocity Section
        velocity_group = QGroupBox("Velocity")
        velocity_group.setAutoFillBackground(True)
        velocity_group.setStyleSheet("QGroupBox { background-color: #3a3a3a; }")
        velocity_layout = QVBoxLayout()
        velocity_layout.setSpacing(2)
        velocity_layout.setContentsMargins(6, 6, 6, 6)
        
        vel_layout = QGridLayout()
        vel_layout.setSpacing(2)
        vel_layout.setHorizontalSpacing(3)
        vel_layout.setColumnStretch(1, 0)
        
        linear_label = QLabel("Linear:")
        linear_label.setFont(QFont("Monospace", 10))
        linear_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        vel_layout.addWidget(linear_label, 0, 0)
        self.linear_vel_label = QLabel("0.00")
        self.linear_vel_label.setFont(QFont("Monospace", 10))
        self.linear_vel_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        vel_layout.addWidget(self.linear_vel_label, 0, 1)
        
        linear_unit_label = QLabel("m/s")
        linear_unit_label.setFont(QFont("Monospace", 10))
        linear_unit_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        vel_layout.addWidget(linear_unit_label, 0, 2)
        
        angular_label = QLabel("Angular:")
        angular_label.setFont(QFont("Monospace", 10))
        angular_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        vel_layout.addWidget(angular_label, 1, 0)
        self.angular_vel_label = QLabel("0.00")
        self.angular_vel_label.setFont(QFont("Monospace", 10))
        self.angular_vel_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        vel_layout.addWidget(self.angular_vel_label, 1, 1)
        
        angular_unit_label = QLabel("rad/s")
        angular_unit_label.setFont(QFont("Monospace", 10))
        angular_unit_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        vel_layout.addWidget(angular_unit_label, 1, 2)
        
        vel_container = QHBoxLayout()
        vel_container.addLayout(vel_layout)
        vel_container.addStretch()
        velocity_layout.addLayout(vel_container)
        
        velocity_group.setLayout(velocity_layout)
        col2_layout.addWidget(velocity_group, 1)
        
        # Position Section
        position_group = QGroupBox("Position")
        position_group.setAutoFillBackground(True)
        position_group.setStyleSheet("QGroupBox { background-color: #3a3a3a; }")
        position_layout = QVBoxLayout()
        position_layout.setSpacing(2)
        position_layout.setContentsMargins(6, 6, 6, 6)
        
        pos_layout = QGridLayout()
        pos_layout.setSpacing(2)
        pos_layout.setHorizontalSpacing(3)
        pos_layout.setColumnStretch(1, 0)
        
        x_label = QLabel("X:")
        x_label.setFont(QFont("Monospace", 10))
        x_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        pos_layout.addWidget(x_label, 0, 0)
        self.position_x_label = QLabel("0.00")
        self.position_x_label.setFont(QFont("Monospace", 10))
        self.position_x_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        pos_layout.addWidget(self.position_x_label, 0, 1)
        
        x_unit_label = QLabel("m")
        x_unit_label.setFont(QFont("Monospace", 10))
        x_unit_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        pos_layout.addWidget(x_unit_label, 0, 2)
        
        y_label = QLabel("Y:")
        y_label.setFont(QFont("Monospace", 10))
        y_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        pos_layout.addWidget(y_label, 1, 0)
        self.position_y_label = QLabel("0.00")
        self.position_y_label.setFont(QFont("Monospace", 10))
        self.position_y_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        pos_layout.addWidget(self.position_y_label, 1, 1)
        
        y_unit_label = QLabel("m")
        y_unit_label.setFont(QFont("Monospace", 10))
        y_unit_label.setStyleSheet("background-color: transparent; color: #e0e0e0;")
        pos_layout.addWidget(y_unit_label, 1, 2)
        
        pos_container = QHBoxLayout()
        pos_container.addLayout(pos_layout)
        pos_container.addStretch()
        position_layout.addLayout(pos_container)
        
        position_group.setLayout(position_layout)
        col2_layout.addWidget(position_group, 1)
        
        main_layout.addLayout(col2_layout, 1)
        
        return group
    
    def create_status_group(self):
        """Create robot status display"""
        group = QGroupBox("Robot Status")
        group.setAutoFillBackground(True)
        group.setStyleSheet("QGroupBox { background-color: #2d2d2d; }")
        layout = QVBoxLayout()
        
        self.status_label = QLabel("Active")
        self.status_label.setStyleSheet("color: #66bb6a; font-weight: bold; background-color: transparent;")
        
        layout.addWidget(self.status_label)
        group.setLayout(layout)
        return group
    
    def create_mode_group(self):
        """Create mode display and switcher"""
        group = QGroupBox("Robot Mode")
        group.setAutoFillBackground(True)
        group.setStyleSheet("QGroupBox { background-color: #2d2d2d; }")
        layout = QHBoxLayout()
        layout.setSpacing(4)
        
        # In sim mode, show "Simulation" in grey. In real mode, show Manual/AUTO with colors
        if not self.is_real_mode:
            mode_text = "Simulation"
            mode_color = "#888888"  # Grey
            button_text = "Switch to Auto"  # Placeholder, will be disabled anyway
        else:
            # Set mode based on current robot_mode (which is updated from topic)
            is_manual = (self.robot_mode == "MANUAL")
            
            if is_manual:
                mode_text = "Manual"
                mode_color = "#66bb6a"  # Green
                button_text = "Switch to Auto"
            else:
                mode_text = "Auto"
                mode_color = "#d32f2f"  # Red
                button_text = "Switch to Manual"
        
        self.mode_label = QLabel(mode_text)
        self.mode_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.mode_label.setStyleSheet(f"color: {mode_color}; font-weight: bold; font-size: 14px; background-color: transparent;")
        layout.addWidget(self.mode_label, 1)
        
        # Mode switch button
        mode_switch_btn = QPushButton(button_text)
        mode_switch_btn.setStyleSheet("""
            QPushButton {
                background-color: #424242;
                color: white;
                font-size: 11px;
                font-weight: bold;
                padding: 4px 10px;
                border-radius: 4px;
                border: 2px solid #616161;
            }
            QPushButton:hover {
                background-color: #616161;
            }
            QPushButton:pressed {
                background-color: #757575;
            }
            QPushButton:disabled {
                background-color: #1f1f1f;
                color: #555555;
                border-color: #333333;
            }
        """)
        mode_switch_btn.clicked.connect(self.toggle_mode)
        layout.addWidget(mode_switch_btn)
        self.mode_switch_btn = mode_switch_btn
        
        group.setLayout(layout)
        return group
    
    def toggle_mode(self):
        """Toggle between manual and auto mode"""
        is_manual = (self.robot_mode == "MANUAL")
        new_mode_is_manual = not is_manual
        
        msg = Bool()
        msg.data = new_mode_is_manual
        self.mode_switch_pub.publish(msg)
        
        if new_mode_is_manual:
            self.node.get_logger().info('Switching to MANUAL mode')
        else:
            self.node.get_logger().info('Switching to AUTO mode')
    
    def create_teleop_control_group(self):
        """Create keyboard teleop control info"""
        group = QGroupBox("Keyboard Teleop")
        group.setAutoFillBackground(True)
        group.setStyleSheet("QGroupBox { background-color: #2d2d2d; }")
        layout = QVBoxLayout()
        
        # Top row: Bucket controls (left) and Speed info (right)
        top_row = QHBoxLayout()
        
        # Left side: Bucket controls
        bucket_section = QVBoxLayout()
        bucket_section.setSpacing(4)
        
        arrow_label = QLabel("Bucket")
        arrow_label.setStyleSheet("color: #bbb; font-size: 13px; background-color: transparent;")
        arrow_label.setAlignment(Qt.AlignCenter)
        bucket_section.addWidget(arrow_label)
        
        # Style the buttons
        button_style = """
            QPushButton {
                background-color: #424242;
                color: white;
                border: 2px solid #616161;
                border-radius: 5px;
                padding: 10px;
                font-size: 14px;
                font-weight: bold;
                min-width: 40px;
                min-height: 40px;
            }
            QPushButton:hover {
                background-color: #616161;
            }
            QPushButton:pressed {
                background-color: #757575;
            }
            QPushButton:disabled {
                background-color: #1f1f1f;
                color: #555555;
                border-color: #333333;
            }
        """
        active_button_style = """
            QPushButton {
                background-color: #4a7ba7;
                color: white;
                border: 2px solid #5a8bc7;
                border-radius: 5px;
                padding: 10px;
                font-size: 14px;
                font-weight: bold;
                min-width: 40px;
                min-height: 40px;
            }
        """
        
        # Create arrow buttons
        arrow_button_layout = QHBoxLayout()
        arrow_button_layout.setSpacing(4)
        self.btn_up = QPushButton("↑")
        self.btn_down = QPushButton("↓")
        
        for btn in [self.btn_up, self.btn_down]:
            btn.setStyleSheet(button_style)
            btn.setFocusPolicy(Qt.NoFocus)
        
        # Connect arrow button press/release events
        self.btn_up.pressed.connect(lambda: self.teleop_key_press('up'))
        self.btn_up.released.connect(lambda: self.teleop_key_release('up'))
        self.btn_down.pressed.connect(lambda: self.teleop_key_press('down'))
        self.btn_down.released.connect(lambda: self.teleop_key_release('down'))
        
        arrow_button_layout.addWidget(self.btn_up)
        arrow_button_layout.addWidget(self.btn_down)
        bucket_section.addLayout(arrow_button_layout)
        
        top_row.addLayout(bucket_section)
        top_row.addStretch()
        
        # Right side: Speed info
        speed_section = QVBoxLayout()
        speed_section.setSpacing(0)
        
        # Combined speed label with both lines using HTML
        speed_html = f'<div style="text-align: right; line-height: 1.2;"><span style="color: #aaa; font-size: 13px;">Lin: {self.linear_speed:.2f} m/s | Ang: {self.angular_speed:.2f} rad/s</span><br><span style="color: #888; font-size: 11px; font-style: italic;">Q/Z: Linear Speed | E/C: Angular Speed</span></div>'
        self.speed_label = QLabel(speed_html)
        self.speed_label.setStyleSheet("background-color: transparent; padding: 0px; margin: 0px;")
        self.speed_label.setAlignment(Qt.AlignRight)
        speed_section.addWidget(self.speed_label)
        
        # Keep a reference for updating (we'll need to update the combined text)
        self.speed_info_label = self.speed_label
        
        top_row.addLayout(speed_section)
        layout.addLayout(top_row)
        
        # WASD Movement Controls
        wasd_label = QLabel("Movement")
        wasd_label.setStyleSheet("color: #bbb; font-size: 13px; margin-top: 8px; background-color: transparent;")
        wasd_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(wasd_label)
        
        # WASD button grid
        wasd_grid = QGridLayout()
        wasd_grid.setSpacing(4)
        
        # Create WASD buttons
        self.btn_w = QPushButton("W")
        self.btn_a = QPushButton("A")
        self.btn_s = QPushButton("S")
        self.btn_d = QPushButton("D")
        
        for btn in [self.btn_w, self.btn_a, self.btn_s, self.btn_d]:
            btn.setStyleSheet(button_style)
            btn.setFocusPolicy(Qt.NoFocus)
        
        # Connect button press/release events to teleop controls
        self.btn_w.pressed.connect(lambda: self.teleop_key_press('w'))
        self.btn_w.released.connect(lambda: self.teleop_key_release('w'))
        self.btn_a.pressed.connect(lambda: self.teleop_key_press('a'))
        self.btn_a.released.connect(lambda: self.teleop_key_release('a'))
        self.btn_s.pressed.connect(lambda: self.teleop_key_press('s'))
        self.btn_s.released.connect(lambda: self.teleop_key_release('s'))
        self.btn_d.pressed.connect(lambda: self.teleop_key_press('d'))
        self.btn_d.released.connect(lambda: self.teleop_key_release('d'))
        
        # Add to grid (W at top center, ASD on bottom row)
        wasd_grid.addWidget(self.btn_w, 0, 1)
        wasd_grid.addWidget(self.btn_a, 1, 0)
        wasd_grid.addWidget(self.btn_s, 1, 1)
        wasd_grid.addWidget(self.btn_d, 1, 2)
        
        layout.addLayout(wasd_grid)
        
        # Store button references for styling updates
        self.teleop_buttons = {
            'w': self.btn_w,
            'a': self.btn_a,
            's': self.btn_s,
            'd': self.btn_d,
            'up': self.btn_up,
            'down': self.btn_down
        }
        self.teleop_button_style = button_style
        self.teleop_active_style = active_button_style
        
        group.setLayout(layout)
        return group
    
    def adjust_speed(self, speed_type, multiplier):
        """Adjust linear or angular speed"""
        if speed_type == 'linear':
            self.linear_speed *= multiplier
            self.linear_speed = min(self.max_linear_speed, max(0.0, self.linear_speed))
        elif speed_type == 'angular':
            self.angular_speed *= multiplier
            self.angular_speed = min(self.max_angular_speed, max(0.0, self.angular_speed))
        
        # Update speed display
        speed_html = f'<div style="text-align: right; line-height: 1.2;"><span style="color: #aaa; font-size: 13px;">Lin: {self.linear_speed:.2f} m/s | Ang: {self.angular_speed:.2f} rad/s</span><br><span style="color: #888; font-size: 11px; font-style: italic;">Q/Z: Linear Speed | E/C: Angular Speed</span></div>'
        self.speed_label.setText(speed_html)
        self.node.get_logger().info(f"Speed adjusted - Linear: {self.linear_speed:.2f}, Angular: {self.angular_speed:.2f}")
    
    def teleop_key_press(self, key):
        """Handle teleop key press"""
        if self.is_real_mode:
            return
        self.teleop_keys[key] = True
        # Update button styling
        if key in self.teleop_buttons:
            self.teleop_buttons[key].setStyleSheet(self.teleop_active_style)
        # Publish velocity immediately
        self.publish_teleop_velocity()
    
    def teleop_key_release(self, key):
        """Handle teleop key release"""
        if self.is_real_mode:
            return
        self.teleop_keys[key] = False
        # Update button styling
        if key in self.teleop_buttons:
            self.teleop_buttons[key].setStyleSheet(self.teleop_button_style)
        # Publish velocity immediately
        self.publish_teleop_velocity()
    
    def publish_teleop_velocity(self):
        """Publish velocity command based on active teleop keys"""
        msg = Twist()
        
        # Linear velocity (W/S)
        if self.teleop_keys['w']:
            msg.linear.x = self.linear_speed
        elif self.teleop_keys['s']:
            msg.linear.x = -self.linear_speed
        
        # Angular velocity (A/D)
        if self.teleop_keys['a']:
            msg.angular.z = self.angular_speed
        elif self.teleop_keys['d']:
            msg.angular.z = -self.angular_speed
        
        self.cmd_vel_pub.publish(msg)
        
        # Handle bucket control (Up/Down)
        bucket_speed = 0.05  # radians per button press
        if self.teleop_keys['up']:
            self.bucket_position -= bucket_speed  # Up arrow tips bucket down (digging)
            self.bucket_position = max(self.bucket_position, -1.57)  # Min limit (down)
            bucket_msg = Float64MultiArray()
            bucket_msg.data = [self.bucket_position]
            self.position_cmd_pub.publish(bucket_msg)
        elif self.teleop_keys['down']:
            self.bucket_position += bucket_speed  # Down arrow tips bucket up (dumping)
            self.bucket_position = min(self.bucket_position, 0.1)  # Max limit (up)
            bucket_msg = Float64MultiArray()
            bucket_msg.data = [self.bucket_position]
            self.position_cmd_pub.publish(bucket_msg)
    
    def create_system_control_group(self):
        """Create system launch controls"""
        group = QGroupBox("System Controls")
        group.setAutoFillBackground(True)
        group.setStyleSheet("QGroupBox { background-color: #2d2d2d; }")
        layout = QVBoxLayout()
        
        button_style = """
            QPushButton {
                background-color: #424242;
                color: white;
                font-size: 11px;
                font-weight: bold;
                padding: 6px;
                border-radius: 4px;
                border: 2px solid #616161;
                text-align: left;
            }
            QPushButton:hover {
                background-color: #616161;
            }
            QPushButton:pressed {
                background-color: #757575;
            }
        """
        
        # PointLIO button
        pointlio_btn = QPushButton("Launch PointLIO")
        pointlio_btn.setStyleSheet(button_style)
        pointlio_btn.clicked.connect(lambda: self.launch_system('pointlio'))
        layout.addWidget(pointlio_btn)
        
        # Mapping button
        mapping_btn = QPushButton("Launch Mapping")
        mapping_btn.setStyleSheet(button_style)
        mapping_btn.clicked.connect(lambda: self.launch_system('mapping'))
        layout.addWidget(mapping_btn)
        
        # Nav2 button
        nav2_btn = QPushButton("Launch Nav2")
        nav2_btn.setStyleSheet(button_style)
        nav2_btn.clicked.connect(lambda: self.launch_system('nav2'))
        layout.addWidget(nav2_btn)
        
        # RViz button
        rviz_btn = QPushButton("Launch RViz2")
        rviz_btn.setStyleSheet(button_style)
        rviz_btn.clicked.connect(self.launch_rviz)
        layout.addWidget(rviz_btn)
        
        group.setLayout(layout)
        return group
    
    def create_visualization_group(self):
        """Create visualization tool controls"""
        group = QGroupBox("Visualization Tools")
        group.setAutoFillBackground(True)
        group.setStyleSheet("QGroupBox { background-color: #2d2d2d; }")
        layout = QVBoxLayout()
        
        button_style = """
            QPushButton {
                background-color: #424242;
                color: white;
                font-size: 12px;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
                border: 2px solid #616161;
                text-align: left;
            }
            QPushButton:hover {
                background-color: #616161;
            }
            QPushButton:pressed {
                background-color: #757575;
            }
        """
        
        # RViz button
        rviz_btn = QPushButton("Launch RViz2")
        rviz_btn.setStyleSheet(button_style)
        rviz_btn.clicked.connect(self.launch_rviz)
        layout.addWidget(rviz_btn)
        
        group.setLayout(layout)
        return group
    
    def create_action_control_group(self):
        """Create action control buttons"""
        group = QGroupBox("Actions")
        group.setAutoFillBackground(True)
        group.setStyleSheet("QGroupBox { background-color: #2d2d2d; }")
        layout = QVBoxLayout()
        
        # Home button
        self.home_btn = QPushButton("Home Actuators")
        self.home_btn.setStyleSheet(Styles.orange_button())
        self.home_btn.clicked.connect(self.send_home_goal)
        layout.addWidget(self.home_btn)
        
        # Localize button
        self.localize_btn = QPushButton("Localize")
        self.localize_btn.setStyleSheet(Styles.standard_button())
        self.localize_btn.clicked.connect(lambda: self.launch_system('localization'))
        layout.addWidget(self.localize_btn)
        
        # Excavate button
        self.excavate_btn = QPushButton("Excavate")
        self.excavate_btn.setStyleSheet("""
            QPushButton {
                background-color: #424242;
                color: white;
                font-size: 12px;
                font-weight: bold;
                padding: 8px;
                border-radius: 4px;
                border: 2px solid #616161;
            }
            QPushButton:hover {
                background-color: #616161;
            }
            QPushButton:pressed {
                background-color: #757575;
            }
            QPushButton:disabled {
                background-color: #2a2a2a;
                color: #666;
                border: 2px solid #404040;
            }
        """)
        self.excavate_btn.clicked.connect(self.send_excavate_goal)
        layout.addWidget(self.excavate_btn)
        
        # Dump button
        self.dump_btn = QPushButton("Dump")
        self.dump_btn.setStyleSheet("""
            QPushButton {
                background-color: #424242;
                color: white;
                font-size: 12px;
                font-weight: bold;
                padding: 8px;
                border-radius: 4px;
                border: 2px solid #616161;
            }
            QPushButton:hover {
                background-color: #616161;
            }
            QPushButton:pressed {
                background-color: #757575;
            }
            QPushButton:disabled {
                background-color: #2a2a2a;
                color: #666;
                border: 2px solid #404040;
            }
        """)
        self.dump_btn.clicked.connect(self.send_dump_goal)
        layout.addWidget(self.dump_btn)
        
        # One Cycle Auto button
        self.auto_btn = QPushButton("One Cycle Auto")
        self.auto_btn.setStyleSheet(Styles.blue_button())
        self.auto_btn.clicked.connect(self.send_full_auto_goal)
        layout.addWidget(self.auto_btn)
        
        # Operation status label
        self.operation_status_label = QLabel("Status: Idle")
        self.operation_status_label.setWordWrap(True)
        self.operation_status_label.setStyleSheet("color: #aaa; font-style: italic; background-color: transparent;")
        layout.addWidget(self.operation_status_label)
        
        group.setLayout(layout)
        return group
    
    def create_camera_control_group(self):
        """Create camera control buttons"""
        group = QGroupBox("Fisheye Camera Rotation")
        group.setAutoFillBackground(True)
        group.setStyleSheet("""
            QGroupBox { 
                background-color: #2d2d2d; 
                border: 2px solid #505050;
                border-radius: 5px;
                margin: 0px;
                font-weight: bold;
                padding-top: 8px;
                padding-left: 4px;
                padding-right: 4px;
                padding-bottom: 4px;
                color: #e0e0e0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 5px;
                top: 5px;
                left: 2px;
                color: #4da3f0;
            }
        """)
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 0, 10, 10)
        layout.setSpacing(0)
        
        # Camera position display (at top)
        self.camera_pos_label = QLabel("Position: 0°")
        self.camera_pos_label.setStyleSheet("color: #aaa; font-size: 14px; font-weight: bold; background-color: transparent; margin-top: -8px;")
        self.camera_pos_label.setAlignment(Qt.AlignCenter)
        self.camera_pos_label.setMaximumHeight(20)
        layout.addWidget(self.camera_pos_label)
        
        layout.addSpacing(2)  # Small gap between label and buttons
        
        # Control buttons row
        button_row = QHBoxLayout()
        button_row.setSpacing(8)
        
        # Rotate left button
        rotate_left_btn = QPushButton("← 45°")
        rotate_left_btn.setStyleSheet("""
            QPushButton {
                background-color: #424242;
                color: white;
                font-size: 14px;
                font-weight: bold;
                padding: 20px 15px;
                border-radius: 5px;
                border: 2px solid #616161;
            }
            QPushButton:hover {
                background-color: #616161;
            }
            QPushButton:pressed {
                background-color: #757575;
            }
        """)
        rotate_left_btn.clicked.connect(lambda: self.rotate_fisheye_camera(0.785398))  # +45 degrees
        button_row.addWidget(rotate_left_btn)
        
        # Center column with Center and 180° buttons stacked
        center_column = QVBoxLayout()
        center_column.setSpacing(8)
        
        # Center button
        center_btn = QPushButton("Center")
        center_btn.setStyleSheet("""
            QPushButton {
                background-color: #424242;
                color: white;
                font-size: 14px;
                font-weight: bold;
                padding: 15px;
                border-radius: 5px;
                border: 2px solid #616161;
            }
            QPushButton:hover {
                background-color: #616161;
            }
            QPushButton:pressed {
                background-color: #757575;
            }
        """)
        center_btn.clicked.connect(lambda: self.set_fisheye_camera_position(0.0))
        center_column.addWidget(center_btn)
        
        # 180° button
        btn_180 = QPushButton("180°")
        btn_180.setStyleSheet("""
            QPushButton {
                background-color: #424242;
                color: white;
                font-size: 14px;
                font-weight: bold;
                padding: 15px;
                border-radius: 5px;
                border: 2px solid #616161;
            }
            QPushButton:hover {
                background-color: #616161;
            }
            QPushButton:pressed {
                background-color: #757575;
            }
        """)
        btn_180.clicked.connect(lambda: self.set_fisheye_camera_position(3.14159))
        center_column.addWidget(btn_180)
        
        button_row.addLayout(center_column)
        
        # Rotate right button
        rotate_right_btn = QPushButton("45° →")
        rotate_right_btn.setStyleSheet("""
            QPushButton {
                background-color: #424242;
                color: white;
                font-size: 14px;
                font-weight: bold;
                padding: 20px 15px;
                border-radius: 5px;
                border: 2px solid #616161;
            }
            QPushButton:hover {
                background-color: #616161;
            }
            QPushButton:pressed {
                background-color: #757575;
            }
        """)
        rotate_right_btn.clicked.connect(lambda: self.rotate_fisheye_camera(-0.785398))  # -45 degrees
        button_row.addWidget(rotate_right_btn)
        
        layout.addLayout(button_row)
        
        group.setLayout(layout)
        return group

    
    def create_hardware_group(self):
        """Create hardware control buttons (real mode only)"""
        group = QGroupBox("Hardware")
        group.setAutoFillBackground(True)
        group.setStyleSheet("QGroupBox { background-color: #2d2d2d; }")
        layout = QVBoxLayout()
        
        # CAN Interface button
        can_btn = QPushButton("Start CAN Interface")
        can_btn.setStyleSheet(Styles.standard_button(size=13))
        can_btn.clicked.connect(self.start_can_interface)
        layout.addWidget(can_btn)
        
        # Hardware Launch button
        hardware_btn = QPushButton("Launch Hardware")
        hardware_btn.setStyleSheet(Styles.standard_button(size=13))
        hardware_btn.clicked.connect(self.launch_hardware)
        layout.addWidget(hardware_btn)
        
        group.setLayout(layout)
        return group
    
    def start_can_interface(self):
        """Start CAN interface using canable_start.sh script"""
        try:
            script_path = f'{self.robot_workspace}/src/lunabot_ros/scripts/canable_start.sh'
            
            if self.is_remote:
                # Execute on remote robot via SSH
                self.node.get_logger().info(f'Running CAN interface on {self.robot_user}@{self.robot_host}')
                cmd = ['ssh', f'{self.robot_user}@{self.robot_host}', f'bash {script_path}']
            else:
                # Execute locally
                self.node.get_logger().info(f'Running CAN interface script: {script_path}')
                cmd = ['bash', os.path.expanduser(script_path)]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=10
            )
            
            # Log the output
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
                # Launch on remote robot via SSH
                self.node.get_logger().info(f'Launching hardware on {self.robot_user}@{self.robot_host}')
                
                # Hardware launch command
                hw_cmd = (
                    f"ssh {self.robot_user}@{self.robot_host} "
                    f"'source {self.robot_workspace}/install/setup.bash && "
                    f"ros2 launch lunabot_bringup hardware_launch.py'"
                )
                self.launch_processes['hardware'] = subprocess.Popen(
                    hw_cmd,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                
                # Actions launch command
                actions_cmd = (
                    f"ssh {self.robot_user}@{self.robot_host} "
                    f"'source {self.robot_workspace}/install/setup.bash && "
                    f"ros2 launch lunabot_bringup actions_launch.py use_sim:=false'"
                )
                self.launch_processes['actions'] = subprocess.Popen(
                    actions_cmd,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
            else:
                # Launch locally
                self.launch_processes['hardware'] = subprocess.Popen(
                    ['ros2', 'launch', 'lunabot_bringup', 'hardware_launch.py'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                
                self.launch_processes['actions'] = subprocess.Popen(
                    ['ros2', 'launch', 'lunabot_bringup', 'actions_launch.py', 'use_sim:=false'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
            
            self.hardware_enabled = True
            self.node.get_logger().info('Hardware and action servers launched')
        except Exception as e:
            self.node.get_logger().error(f'Failed to launch hardware: {str(e)}')
    
    def create_emergency_group(self):
        """Create emergency controls"""
        group = QGroupBox("Emergency Controls")
        group.setAutoFillBackground(True)
        group.setStyleSheet("QGroupBox { background-color: #2d2d2d; }")
        layout = QVBoxLayout()
        
        emergency_btn = QPushButton("EMERGENCY STOP")
        emergency_btn.setStyleSheet(Styles.emergency_button())
        emergency_btn.clicked.connect(self.emergency_stop)
        
        layout.addWidget(emergency_btn)
        group.setLayout(layout)
        return group
    
    def emergency_stop(self):
        """Publish emergency stop command"""
        msg = Bool()
        msg.data = True
        self.emergency_stop_pub.publish(msg)
        self.node.get_logger().error('EMERGENCY STOP TRIGGERED!')
        self.status_label.setText("Robot: EMERGENCY STOP")
        self.status_label.setStyleSheet("color: #d32f2f; font-weight: bold;")
        
        # Cancel any active operations
        if self.excavation_goal_handle is not None:
            self.excavation_goal_handle.cancel_goal_async()
        if self.dumping_goal_handle is not None:
            self.dumping_goal_handle.cancel_goal_async()
        
        # Reset button states
        self.is_excavating = False
        self.is_dumping = False
        self.full_auto_active = False
        self.excavate_btn.setText("Excavate")
        self.excavate_btn.setEnabled(False)
        self.dump_btn.setText("Dump")
        self.dump_btn.setEnabled(False)
        self.auto_btn.setText("One Cycle Auto")
        self.auto_btn.setEnabled(False)
    
    def launch_system(self, system_name):
        """Launch a system in a new terminal (user closes terminal to stop)"""
        self.node.get_logger().info(f'Launching {system_name}...')
        try:
            # Determine launch location based on mode
            # In real mode: run on robot PC (if remote), otherwise local
            # In sim mode: always run local
            run_on_robot = self.is_real_mode and self.is_remote
            
            # Determine use_sim argument based on robot mode
            use_sim_arg = 'false' if self.is_real_mode else 'true'
            
            # Launch file mapping
            launch_file = f'{system_name}_launch.py'
            if system_name == 'nav2':
                launch_file = 'nav2_stack_launch.py'
            
            if run_on_robot:
                # Launch on remote robot PC via SSH
                self.node.get_logger().info(f'Launching {system_name} on {self.robot_user}@{self.robot_host}')
                
                ssh_cmd = (
                    f"ssh {self.robot_user}@{self.robot_host} "
                    f"'source {self.robot_workspace}/install/setup.bash && "
                    f"ros2 launch lunabot_bringup {launch_file} use_sim:={use_sim_arg}'"
                )
                
                # Launch in a new terminal window with SSH command
                subprocess.Popen(
                    ['gnome-terminal', '--', 'bash', '-c',
                     f'{ssh_cmd}; read -p "Press Enter to close..."'],
                )
            else:
                # Launch locally
                # Get workspace install path from environment
                ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
                if ament_prefix:
                    # Get the first install path (our workspace)
                    install_paths = ament_prefix.split(':')
                    # Find the lunabot workspace install directory
                    workspace_install = None
                    for path in install_paths:
                        if 'lunabot' in path and 'install' in path:
                            # Get the install directory (parent of package dirs)
                            workspace_install = path.split('/install/')[0] + '/install'
                            break
                    
                    if not workspace_install and install_paths:
                        # Fallback to first install path
                        workspace_install = install_paths[0].split('/install/')[0] + '/install'
                    
                    setup_cmd = f'source {workspace_install}/setup.bash'
                else:
                    # Last resort: try to source from current ROS environment
                    setup_cmd = 'source $(ros2 pkg prefix lunabot_bringup)/../setup.bash 2>/dev/null || source install/setup.bash'
                
                # Launch in a new terminal window
                subprocess.Popen(
                    ['gnome-terminal', '--', 'bash', '-c',
                     f'{setup_cmd} && ros2 launch lunabot_bringup {launch_file} use_sim:={use_sim_arg}; read -p "Press Enter to close..."'],
                )
            
            location = f"on {self.robot_user}@{self.robot_host}" if run_on_robot else "locally"
            self.node.get_logger().info(f'{system_name} launched {location} (close terminal to stop)')
        except Exception as e:
            self.node.get_logger().error(f'Failed to launch {system_name}: {e}')
    
    def launch_rviz(self):
        """Launch RViz2 with robot visualization config"""
        self.node.get_logger().info('Launching RViz2...')
        try:
            # Get workspace install path
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
            
            # Get RViz config path
            config_path = '$(ros2 pkg prefix lunabot_config)/share/lunabot_config/rviz/robot_view.rviz'
            
            # Launch RViz2 in a new terminal
            subprocess.Popen(
                ['gnome-terminal', '--', 'bash', '-c',
                 f'{setup_cmd} && rviz2 -d {config_path}; read -p "Press Enter to close..."'],
            )
            
            self.node.get_logger().info('RViz2 launched (close terminal to stop)')
        except Exception as e:
            self.node.get_logger().error(f'Failed to launch RViz2: {e}')
    
    def send_excavate_goal(self):
        """Send excavate action goal or cancel if already running"""
        # If already excavating, cancel it
        if self.is_excavating:
            self.node.get_logger().info('Cancelling excavation...')
            if self.excavation_goal_handle is not None:
                cancel_future = self.excavation_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.excavate_cancel_callback)
            else:
                # Goal hasn't been accepted yet, just reset state
                self.is_excavating = False
                self.excavate_btn.setText("Excavate")
                self.operation_status_label.setText("Status: Cancelled")
                self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
            return
        
        self.node.get_logger().info('Sending excavate goal...')
        self.is_excavating = True
        self.excavate_btn.setText("Cancel Excavation")
        self.operation_status_label.setText("Status: Excavating...")
        self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
        
        goal_msg = Excavation.Goal()
        
        send_goal_future = self.excavation_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.excavate_goal_response_callback)
    
    def excavate_goal_response_callback(self, future):
        """Handle excavation goal response"""
        goal_handle = future.result()
        
        # Check if user cancelled before goal was accepted
        if not self.is_excavating:
            self.node.get_logger().info('Goal was cancelled before acceptance, cancelling now')
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        
        if not goal_handle.accepted:
            self.node.get_logger().error('Excavate goal rejected')
            self.is_excavating = False
            self.excavation_goal_handle = None
            self.excavate_btn.setText("Excavate")
            self.operation_status_label.setText("Status: Goal rejected")
            self.operation_status_label.setStyleSheet("color: #d32f2f; background-color: transparent;")
            return
        
        self.node.get_logger().info('Excavate goal accepted')
        self.excavation_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.excavate_result_callback)
    
    def excavate_result_callback(self, future):
        """Handle excavation result"""
        result = future.result().result
        self.is_excavating = False
        self.excavation_goal_handle = None
        self.excavate_btn.setText("Excavate")
        
        if result.success:
            self.node.get_logger().info('Excavation completed successfully')
            self.operation_status_label.setText("Status: Excavation completed")
            self.operation_status_label.setStyleSheet("color: #66bb6a; background-color: transparent;")
        else:
            self.node.get_logger().error('Excavation failed')
            self.operation_status_label.setText("Status: Excavation failed")
            self.operation_status_label.setStyleSheet("color: #d32f2f; background-color: transparent;")
    
    def excavate_cancel_callback(self, future):
        """Handle excavation cancellation response"""
        self.node.get_logger().info('Excavation cancelled')
        self.is_excavating = False
        self.excavation_goal_handle = None
        self.excavate_btn.setText("Excavate")
        self.operation_status_label.setText("Status: Excavation cancelled")
        self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
    
    def send_dump_goal(self):
        """Send dump action goal or cancel if already running"""
        # If already dumping, cancel it
        if self.is_dumping:
            self.node.get_logger().info('Cancelling dumping...')
            if self.dumping_goal_handle is not None:
                cancel_future = self.dumping_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.dump_cancel_callback)
            else:
                # Goal hasn't been accepted yet, just reset state
                self.is_dumping = False
                self.dump_btn.setText("Dump")
                self.operation_status_label.setText("Status: Cancelled")
                self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
            return
        
        self.node.get_logger().info('Sending dump goal...')
        self.is_dumping = True
        self.dump_btn.setText("Cancel Dump")
        self.operation_status_label.setText("Status: Dumping...")
        self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
        
        goal_msg = Dumping.Goal()
        
        send_goal_future = self.dumping_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.dump_goal_response_callback)
    
    def dump_goal_response_callback(self, future):
        """Handle dump goal response"""
        goal_handle = future.result()
        
        # Check if user cancelled before goal was accepted
        if not self.is_dumping:
            self.node.get_logger().info('Goal was cancelled before acceptance, cancelling now')
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        
        if not goal_handle.accepted:
            self.node.get_logger().error('Dump goal rejected')
            self.is_dumping = False
            self.dumping_goal_handle = None
            self.dump_btn.setText("Dump")
            self.operation_status_label.setText("Status: Goal rejected")
            self.operation_status_label.setStyleSheet("color: #d32f2f; background-color: transparent;")
            return
        
        self.node.get_logger().info('Dump goal accepted')
        self.dumping_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.dump_result_callback)
    
    def dump_result_callback(self, future):
        """Handle dump result"""
        result = future.result().result
        self.is_dumping = False
        self.dumping_goal_handle = None
        self.dump_btn.setText("Dump")
        
        if result.success:
            self.node.get_logger().info(f'Dumping completed: {result.message}')
            self.operation_status_label.setText(f"Status: {result.message}")
            self.operation_status_label.setStyleSheet("color: #66bb6a; background-color: transparent;")
        else:
            self.node.get_logger().error(f'Dumping failed: {result.message}')
            self.operation_status_label.setText(f"Status: Failed - {result.message}")
            self.operation_status_label.setStyleSheet("color: #d32f2f; background-color: transparent;")
    
    def dump_cancel_callback(self, future):
        """Handle dump cancellation response"""
        self.node.get_logger().info('Dumping cancelled')
        self.is_dumping = False
        self.dumping_goal_handle = None
        self.dump_btn.setText("Dump")
        self.operation_status_label.setText("Status: Dumping cancelled")
        self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
    
    def send_home_goal(self):
        """Send homing action goal or cancel if already running"""
        # If already homing, cancel it
        if self.is_homing:
            self.node.get_logger().info('Cancelling homing...')
            if self.homing_goal_handle is not None:
                cancel_future = self.homing_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.home_cancel_callback)
            else:
                # Goal hasn't been accepted yet, just reset state
                self.is_homing = False
                self.home_btn.setText("Home Actuators")
                self.operation_status_label.setText("Status: Cancelled")
                self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
            return
        
        self.node.get_logger().info('Sending home goal...')
        self.is_homing = True
        self.home_btn.setText("Cancel Homing")
        self.operation_status_label.setText("Status: Homing actuators...")
        self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
        
        goal_msg = Homing.Goal()
        
        send_goal_future = self.homing_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.home_goal_response_callback)
    
    def home_goal_response_callback(self, future):
        """Handle home goal response"""
        goal_handle = future.result()
        
        # Check if user cancelled before goal was accepted
        if not self.is_homing:
            self.node.get_logger().info('Goal was cancelled before acceptance, cancelling now')
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        
        if not goal_handle.accepted:
            self.node.get_logger().error('Home goal rejected')
            self.is_homing = False
            self.homing_goal_handle = None
            self.home_btn.setText("Home Actuators")
            self.operation_status_label.setText("Status: Goal rejected")
            self.operation_status_label.setStyleSheet("color: #d32f2f; background-color: transparent;")
            return
        
        self.node.get_logger().info('Home goal accepted')
        self.homing_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.home_result_callback)
    
    def home_result_callback(self, future):
        """Handle home result"""
        result = future.result().result
        self.is_homing = False
        self.homing_goal_handle = None
        self.home_btn.setText("Home Actuators")
        
        if result.success:
            self.node.get_logger().info(f'Homing completed: {result.message}')
            self.operation_status_label.setText(f"Status: {result.message}")
            self.operation_status_label.setStyleSheet("color: #66bb6a; background-color: transparent;")
        else:
            self.node.get_logger().error(f'Homing failed: {result.message}')
            self.operation_status_label.setText(f"Status: Failed - {result.message}")
            self.operation_status_label.setStyleSheet("color: #d32f2f; background-color: transparent;")
    
    def home_cancel_callback(self, future):
        """Handle home cancellation response"""
        self.node.get_logger().info('Homing cancelled')
        self.is_homing = False
        self.homing_goal_handle = None
        self.home_btn.setText("Home Actuators")
        self.operation_status_label.setText("Status: Homing cancelled")
        self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
    
    def send_full_auto_goal(self):
        """Start one autonomous cycle by launching navigation client"""
        # If one cycle auto is running, cancel it
        if self.full_auto_active:
            self.node.get_logger().info('Stopping one cycle auto...')
            
            # Kill the navigation client process if it exists
            if hasattr(self, 'navigation_client_process') and self.navigation_client_process:
                try:
                    self.navigation_client_process.terminate()
                    self.navigation_client_process = None
                except Exception as e:
                    self.node.get_logger().error(f'Failed to stop navigation client: {e}')
            
            self.full_auto_active = False
            self.auto_btn.setText("One Cycle Auto")
            self.operation_status_label.setText("Status: One cycle auto stopped")
            self.operation_status_label.setStyleSheet("color: #ffa726; background-color: transparent;")
            return
        
        if self.is_excavating or self.is_dumping or self.is_homing:
            self.node.get_logger().warning('Operation already in progress!')
            return
        
        self.node.get_logger().info('Starting one cycle auto (navigation client)...')
        self.full_auto_active = True
        self.auto_btn.setText("Stop One Cycle Auto")
        self.operation_status_label.setText("Status: One Cycle Auto Active...")
        self.operation_status_label.setStyleSheet("color: #2196f3; background-color: transparent;")
        
        try:
            # Get workspace install path
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
            
            # Determine use_sim and use_localization parameters
            use_sim_arg = 'false' if self.is_real_mode else 'true'
            use_localization_arg = 'false'  # Default to false, can make this configurable
            
            # Launch navigation client in a new terminal
            self.navigation_client_process = subprocess.Popen(
                ['gnome-terminal', '--', 'bash', '-c',
                 f'{setup_cmd} && ros2 run lunabot_nav navigation_client --ros-args -p use_sim_time:={use_sim_arg} -p use_localization:={use_localization_arg}; read -p "Press Enter to close..."'],
            )
            
            self.node.get_logger().info('Navigation client launched (one cycle auto)')
        except Exception as e:
            self.node.get_logger().error(f'Failed to launch navigation client: {e}')
            self.full_auto_active = False
            self.auto_btn.setText("One Cycle Auto")
            self.operation_status_label.setText("Status: Failed to start one cycle auto")
            self.operation_status_label.setStyleSheet("color: #d32f2f; background-color: transparent;")
    
    def rotate_fisheye_camera(self, delta_radians):
        """Rotate fisheye camera by delta angle"""
        new_position = self.fisheye_camera_position + delta_radians
        # Wrap to [-pi, pi]
        while new_position > 3.14159:
            new_position -= 6.28318
        while new_position < -3.14159:
            new_position += 6.28318
        self.set_fisheye_camera_position(new_position)
    
    def set_fisheye_camera_position(self, position_radians):
        """Set fisheye camera to absolute position"""
        self.fisheye_camera_position = position_radians
        
        # Publish position command to camera controller
        msg = Float64MultiArray()
        msg.data = [position_radians]
        self.camera_cmd_pub.publish(msg)
        
        # Update display
        degrees = position_radians * 180.0 / 3.14159
        self.camera_pos_label.setText(f"Position: {degrees:.0f}°")
        
        self.node.get_logger().info(f'Fisheye camera position set to {degrees:.1f}°')
    
    def spin_ros(self):
        """Spin ROS node to process callbacks"""
        if not self._shutdown and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0)
            except Exception as e:
                # Silently handle shutdown-related errors
                if not self._shutdown:
                    self.node.get_logger().debug(f'Spin error: {e}')
    
    def update_ui(self):
        """Update UI elements with latest data"""
        # Update bandwidth display
        self.bandwidth_total_label.setText(f"{self.bandwidth_total:.2f} Mbps")
        self.bandwidth_rx_label.setText(f"{self.bandwidth_rx:.2f}")
        self.bandwidth_tx_label.setText(f"{self.bandwidth_tx:.2f}")
        # Update bandwidth progress bar (max 4 Mbps per competition rules)
        bandwidth_percent = min(100, int((self.bandwidth_total / 4.0) * 100))
        self.bandwidth_progress.setValue(bandwidth_percent)
        
        # Change progress bar color based on bandwidth usage
        if bandwidth_percent >= 90:  # Over 3.6 Mbps - critical
            self.bandwidth_progress.setStyleSheet("QProgressBar::chunk { background-color: #e53935; }")
        elif bandwidth_percent >= 75:  # Over 3.0 Mbps - warning
            self.bandwidth_progress.setStyleSheet("QProgressBar::chunk { background-color: #fb8c00; }")
        else:  # Under 3.0 Mbps - normal
            self.bandwidth_progress.setStyleSheet("QProgressBar::chunk { background-color: #66bb6a; }")
        
        # Update velocity display
        self.linear_vel_label.setText(f"{self.linear_velocity:.2f}")
        self.angular_vel_label.setText(f"{self.angular_velocity:.2f}")
        
        # Update position display
        self.position_x_label.setText(f"{self.position_x:.2f}")
        self.position_y_label.setText(f"{self.position_y:.2f}")
        
        # Update bucket angle display (negated since rotation is inverted)
        bucket_angle_deg = -self.bucket_position * 180.0 / 3.14159  # Convert radians to degrees
        self.bucket_angle_label.setText(f"{bucket_angle_deg:.1f}°")
        
        # Publish teleop velocity if any key is active (sim mode only)
        if not self.is_real_mode and any(self.teleop_keys.values()):
            self.publish_teleop_velocity()
        
        # Update camera feeds
        if CV_AVAILABLE:
            # Update swappable camera (front or rear)
            if self.swappable_camera_showing_front:
                self.update_camera_display(self.swappable_camera_label, self.front_camera_image)
            else:
                self.update_camera_display(self.swappable_camera_label, self.rear_camera_image)
            # Update fisheye camera
            self.update_camera_display(self.fisheye_camera_label, self.fisheye_camera_image)
    
    def update_camera_display(self, label, cv_image):
        """Update a camera display label with OpenCV image"""
        if cv_image is None:
            return
        
        # Resize image to fit label
        height, width = cv_image.shape[:2]
        label_width = label.width()
        label_height = label.height()
        
        # Calculate scaling factor
        scale = min(label_width / width, label_height / height)
        new_width = int(width * scale)
        new_height = int(height * scale)
        
        # Resize and convert to QPixmap
        resized = cv2.resize(cv_image, (new_width, new_height))
        rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        label.setPixmap(pixmap)
    
    def keyPressEvent(self, event):
        """Handle key press events"""
        # F11 to toggle fullscreen
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
            event.accept()
            return
        # ESC to exit fullscreen
        elif event.key() == Qt.Key_Escape and self.isFullScreen():
            self.showNormal()
            event.accept()
            return
        # Teleop keys (sim mode only)
        elif not self.is_real_mode and not event.isAutoRepeat():
            handled = False
            if event.key() == Qt.Key_W:
                self.teleop_key_press('w')
                handled = True
            elif event.key() == Qt.Key_A:
                self.teleop_key_press('a')
                handled = True
            elif event.key() == Qt.Key_S:
                self.teleop_key_press('s')
                handled = True
            elif event.key() == Qt.Key_D:
                self.teleop_key_press('d')
                handled = True
            elif event.key() == Qt.Key_Up:
                self.teleop_key_press('up')
                handled = True
            elif event.key() == Qt.Key_Down:
                self.teleop_key_press('down')
                handled = True
            elif event.key() == Qt.Key_Q:
                self.adjust_speed('linear', 1.1)
                handled = True
            elif event.key() == Qt.Key_Z:
                self.adjust_speed('linear', 0.9)
                handled = True
            elif event.key() == Qt.Key_E:
                self.adjust_speed('angular', 1.1)
                handled = True
            elif event.key() == Qt.Key_C:
                self.adjust_speed('angular', 0.9)
                handled = True
            
            if handled:
                event.accept()
                return
            else:
                super().keyPressEvent(event)
        else:
            super().keyPressEvent(event)
    
    def keyReleaseEvent(self, event):
        """Handle key release events"""
        # Teleop keys (sim mode only)
        if not self.is_real_mode and not event.isAutoRepeat():
            handled = False
            if event.key() == Qt.Key_W:
                self.teleop_key_release('w')
                handled = True
            elif event.key() == Qt.Key_A:
                self.teleop_key_release('a')
                handled = True
            elif event.key() == Qt.Key_S:
                self.teleop_key_release('s')
                handled = True
            elif event.key() == Qt.Key_D:
                self.teleop_key_release('d')
                handled = True
            elif event.key() == Qt.Key_Up:
                self.teleop_key_release('up')
                handled = True
            elif event.key() == Qt.Key_Down:
                self.teleop_key_release('down')
                handled = True
            
            if handled:
                event.accept()
                return
            else:
                super().keyReleaseEvent(event)
        else:
            super().keyReleaseEvent(event)
    
    def closeEvent(self, event):
        """Handle window close event"""
        self._shutdown = True
        
        # Stop timers first
        self.ros_timer.stop()
        self.ui_timer.stop()
        
        # Give time for any pending callbacks to complete
        QApplication.processEvents()
        
        # Note: System launch terminals remain open - user closes them manually
        
        # Clean up ROS resources
        try:
            if self.node:
                self.node.destroy_node()
        except Exception:
            pass
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        
        event.accept()


def main(args=None):
    """Main entry point"""
    app = QApplication(sys.argv)
    
    # Set dark theme
    app.setStyle('Fusion')
    
    # Dark color palette
    dark_palette = QPalette()
    dark_palette.setColor(QPalette.Window, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Base, QColor(35, 35, 35))
    dark_palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ToolTipBase, QColor(25, 25, 25))
    dark_palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Text, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Button, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
    dark_palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
    dark_palette.setColor(QPalette.Link, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    
    # Apply dark palette
    app.setPalette(dark_palette)
    
    # Setup signal handling for clean shutdown
    def signal_handler(sig, frame):
        app.quit()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Allow Ctrl+C to work properly
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)
    
    window = LunabotGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
