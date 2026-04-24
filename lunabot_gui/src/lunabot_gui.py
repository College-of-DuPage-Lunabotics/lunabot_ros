#!/usr/bin/env python3
import signal
import sys

import rclpy
from rclpy.node import Node

import ui_widgets
from gui_styles import Colors, MAIN_STYLESHEET, Styles
from ros_interface import RobotInterface

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QIcon, QImage, QPalette, QPixmap, QFont
from PyQt5.QtWidgets import (QApplication, QGroupBox, QHBoxLayout, QLabel,
                              QMainWindow, QTextEdit, QPushButton, QSizePolicy, QVBoxLayout,
                              QWidget)


try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


# --- GUI Layout Constants ---
WINDOW_DEFAULT_WIDTH   = 1600
WINDOW_DEFAULT_HEIGHT  = 900
WINDOW_MIN_WIDTH       = 1000
WINDOW_MIN_HEIGHT      = 650
SIDEBAR_MIN_WIDTH      = 300
SIDEBAR_COLLAPSED_WIDTH = 30
CAMERA_MIN_WIDTH       = 320
CAMERA_MIN_HEIGHT      = 240

# --- Teleop Constants ---
TELEOP_LINEAR_SPEED_DEFAULT  = 0.35   # m/s
TELEOP_ANGULAR_SPEED_DEFAULT = 0.25   # rad/s
TELEOP_MAX_LINEAR_SPEED      = 0.75   # m/s
TELEOP_MAX_ANGULAR_SPEED     = 0.75   # rad/s
TELEOP_SPEED_INCREMENT       = 1.1
TELEOP_SPEED_DECREMENT       = 0.9
TELEOP_BUCKET_SPEED          = 0.05   # rad per update
TELEOP_BUCKET_LIMIT_MIN      = -1.57  # rad (down)
TELEOP_BUCKET_LIMIT_MAX      = 0.1    # rad (up)

# --- Timer Intervals ---
ROS_SPIN_INTERVAL_MS = 10      # 100 Hz for ROS callbacks
UI_UPDATE_INTERVAL_MS = 100    # 10 Hz for UI stats
CAMERA_UPDATE_INTERVAL_MS = 66 # 15 Hz for camera displays

# --- Bandwidth Thresholds ---
BANDWIDTH_MAX_MBPS           = 4.0
BANDWIDTH_WARN_THRESHOLD     = 0.75   # fraction of max
BANDWIDTH_CRITICAL_THRESHOLD = 0.90   # fraction of max


class LunabotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Need to init rclpy temporarily to read parameter
        if not rclpy.ok():
            rclpy.init()
        temp_node = Node('temp_param_reader')
        temp_node.declare_parameter('mode', True)  # True = sim, False = real
        mode_val = temp_node.get_parameter('mode').value
        temp_node.destroy_node()
        
        self.robot = RobotInterface(mode_param=mode_val)
        
        self.robot.on_robot_state_update = self.handle_robot_state_update
        self.robot.on_control_state_update = self.handle_control_state_update
        self.robot.on_log = self.append_log
        
        self.swappable_camera_showing_front = True
        self.sidebar_collapsed = False
        self.fisheye_camera_position = 0.0
        self.fisheye_showing_deposit = True  # True = Deposit View (0 deg ), False = Bucket View (180 deg)
        self.full_auto_active = False
        self.emergency_stopped = False
        
        # Teleop state (sim mode only)
        self.teleop_keys = {
            'w': False, 'a': False, 's': False, 'd': False,
            'up': False, 'down': False
        }
        self.linear_speed = TELEOP_LINEAR_SPEED_DEFAULT
        self.angular_speed = TELEOP_ANGULAR_SPEED_DEFAULT
        self.max_linear_speed = TELEOP_MAX_LINEAR_SPEED
        self.max_angular_speed = TELEOP_MAX_ANGULAR_SPEED
        
        # Camera frame tracking
        self.last_camera_frame_ids = {
            'front': None,
            'rear': None,
            'fisheye': None
        }
        
        self.init_ui()

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(ROS_SPIN_INTERVAL_MS)

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(UI_UPDATE_INTERVAL_MS)
        
        # Separate timer for camera updates (less frequent)
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.update_cameras)
        self.camera_timer.start(CAMERA_UPDATE_INTERVAL_MS)
    
    def init_ui(self):
        self.setWindowTitle('Lunabot Control Panel')
        
        import os
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg_dir = get_package_share_directory('lunabot_gui')
            icon_path = os.path.join(pkg_dir, 'resource', 'lunabot_icon.png')
            if os.path.exists(icon_path):
                self.setWindowIcon(QIcon(icon_path))
        except Exception as e:
            print(f"Could not load window icon: {e}")
        
        screen = QApplication.primaryScreen().geometry()
        window_width = min(WINDOW_DEFAULT_WIDTH, int(screen.width() * 0.9))
        window_height = min(WINDOW_DEFAULT_HEIGHT, int(screen.height() * 0.9))
        x = (screen.width() - window_width) // 2
        y = (screen.height() - window_height) // 2
        self.setGeometry(x, y, window_width, window_height)
        
        self.setMinimumSize(WINDOW_MIN_WIDTH, WINDOW_MIN_HEIGHT)
        
        self.setStyleSheet(MAIN_STYLESHEET)
        
        main_widget = QWidget()
        main_widget.setObjectName("centralWidget")
        main_widget.setStyleSheet("background-color: #1a1a1a;")
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(6)
        main_widget.setLayout(main_layout)
        
        content_widget = self.create_content_area()
        main_layout.addWidget(content_widget, 10)
        
        sidebar_container = self.create_sidebar()
        sidebar_container.setMinimumWidth(SIDEBAR_MIN_WIDTH)
        main_layout.addWidget(sidebar_container, 0)
    
    def create_content_area(self):
        content_widget = QWidget()
        content_layout = QVBoxLayout()
        content_layout.setContentsMargins(4, 0, 0, 4)
        content_layout.setSpacing(5)
        content_widget.setLayout(content_layout)
        
        top_row = self.create_camera_displays()
        content_layout.addWidget(top_row, 10)

        bottom_row = self.create_bottom_row()
        content_layout.addWidget(bottom_row, 0)
        
        return content_widget
    
    def create_camera_displays(self):
        top_row = QWidget()
        top_row_layout = QHBoxLayout()
        top_row_layout.setContentsMargins(0, 0, 0, 0)
        top_row_layout.setSpacing(10)
        top_row.setLayout(top_row_layout)
        
        swappable_container = QWidget()
        swappable_layout = QVBoxLayout()
        swappable_layout.setContentsMargins(0, 0, 0, 0)
        swappable_container.setLayout(swappable_layout)
        
        self.realsense_toggle_btn = QPushButton("RealSense: OFF")
        self.realsense_toggle_btn.setStyleSheet(f"""
            QPushButton {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #3a3a3a, stop:1 #323232);
                color: {Colors.TEXT_DIM};
                border: 1px solid #4a4a4a;
                border-radius: 4px;
                font-size: 13px;
                font-weight: bold;
                padding: 6px 8px;
            }}
            QPushButton:hover {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #444444, stop:1 #3c3c3c);
                border: 1px solid #545454;
            }}
            QPushButton:pressed {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2e2e2e, stop:1 #262626);
                border: 1px solid #3a3a3a;
            }}
        """)
        self.realsense_toggle_btn.setMaximumHeight(32)
        self.realsense_toggle_btn.clicked.connect(self.toggle_realsense_cameras)
        swappable_layout.addWidget(self.realsense_toggle_btn)
        
        self.swappable_camera_group = QGroupBox("Front Camera")
        self.swappable_camera_group.setAutoFillBackground(True)
        self.swappable_camera_group.setStyleSheet(f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2a2a2a, stop:1 #252525); padding-top: 20px; }}")
        swappable_camera_layout = QVBoxLayout()
        swappable_camera_layout.setContentsMargins(4, 4, 4, 4)
        swappable_camera_layout.setSpacing(0)
        self.swappable_camera_label = QLabel("No camera feed")
        self.swappable_camera_label.setAlignment(Qt.AlignCenter)
        self.swappable_camera_label.setMinimumSize(CAMERA_MIN_WIDTH, CAMERA_MIN_HEIGHT)
        self.swappable_camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.swappable_camera_label.setStyleSheet(Styles.camera_label())
        swappable_camera_layout.addWidget(self.swappable_camera_label)
        self.swappable_camera_group.setLayout(swappable_camera_layout)
        swappable_layout.addWidget(self.swappable_camera_group)
        
        self.swap_camera_btn = QPushButton("Switch to Rear Camera")
        self.swap_camera_btn.setStyleSheet(f"""
            QPushButton {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #252525, stop:1 #1d1d1d);
                color: {Colors.TEXT_MAIN};
                border: 1px solid #353535;
                border-radius: 4px;
                font-size: 13px;
                font-weight: bold;
                padding: 6px 8px;
            }}
            QPushButton:hover {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #303030, stop:1 #282828);
                border: 1px solid #404040;
            }}
            QPushButton:pressed {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #1a1a1a, stop:1 #121212);
                border: 1px solid #252525;
            }}
        """)
        self.swap_camera_btn.setMaximumHeight(32)
        self.swap_camera_btn.clicked.connect(self.swap_camera)
        swappable_layout.addWidget(self.swap_camera_btn)
        
        top_row_layout.addWidget(swappable_container, 1)
        
        # Fisheye container with rotation controls
        fisheye_container = QWidget()
        fisheye_container_layout = QVBoxLayout()
        fisheye_container_layout.setContentsMargins(0, 0, 0, 0)
        fisheye_container_layout.setSpacing(4)
        fisheye_container.setLayout(fisheye_container_layout)
        
        fisheye_group = QGroupBox("Fisheye Camera")
        fisheye_group.setAutoFillBackground(True)
        fisheye_group.setStyleSheet(f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2a2a2a, stop:1 #252525); padding-top: 20px; }}")
        fisheye_layout = QVBoxLayout()
        fisheye_layout.setContentsMargins(4, 4, 4, 4)
        fisheye_layout.setSpacing(0)
        self.fisheye_camera_label = QLabel("No camera feed")
        self.fisheye_camera_label.setAlignment(Qt.AlignCenter)
        self.fisheye_camera_label.setMinimumSize(CAMERA_MIN_WIDTH, CAMERA_MIN_HEIGHT)
        self.fisheye_camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.fisheye_camera_label.setStyleSheet(Styles.camera_label())
        fisheye_layout.addWidget(self.fisheye_camera_label)
        fisheye_group.setLayout(fisheye_layout)
        
        fisheye_container_layout.addWidget(fisheye_group)
        
        # Fisheye swap button (Bucket View / Deposit View)
        self.swap_fisheye_btn = QPushButton("Switch to Bucket View")
        self.swap_fisheye_btn.setStyleSheet(f"""
            QPushButton {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #252525, stop:1 #1d1d1d);
                color: {Colors.TEXT_MAIN};
                border: 1px solid #353535;
                border-radius: 4px;
                font-size: 13px;
                font-weight: bold;
                padding: 6px 8px;
            }}
            QPushButton:hover {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #303030, stop:1 #282828);
                border: 1px solid #404040;
            }}
            QPushButton:pressed {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #1a1a1a, stop:1 #121212);
                border: 1px solid #252525;
            }}
        """)
        self.swap_fisheye_btn.setMaximumHeight(32)
        self.swap_fisheye_btn.clicked.connect(self.swap_fisheye_view)
        fisheye_container_layout.addWidget(self.swap_fisheye_btn)
        top_row_layout.addWidget(fisheye_container, 1)
        
        return top_row
    
    def create_bottom_row(self):
        bottom_row = QWidget()
        bottom_row_layout = QHBoxLayout()
        bottom_row_layout.setContentsMargins(0, 0, 0, 0)
        bottom_row_layout.setSpacing(3)
        bottom_row.setLayout(bottom_row_layout)
        
        # Left column: Mode/Bandwidth, Telemetry, Bucket State
        left_column = QWidget()
        left_column_layout = QVBoxLayout()
        left_column_layout.setContentsMargins(0, 0, 0, 0)
        left_column_layout.setSpacing(3)
        left_column.setLayout(left_column_layout)
        
        # Row 1: Mode + Bandwidth
        mode_bandwidth_container = QWidget()
        mode_bandwidth_layout = QHBoxLayout()
        mode_bandwidth_layout.setContentsMargins(0, 0, 0, 0)
        mode_bandwidth_layout.setSpacing(3)
        mode_bandwidth_container.setLayout(mode_bandwidth_layout)
        
        mode_group = ui_widgets.create_mode_group(self)
        if not self.robot.is_real_mode:
            mode_group.setEnabled(False)
        mode_bandwidth_layout.addWidget(mode_group, 2)
        
        bandwidth_group = ui_widgets.create_bandwidth_group(self)
        mode_bandwidth_layout.addWidget(bandwidth_group, 3)
        
        left_column_layout.addWidget(mode_bandwidth_container)
        
        # Row 2: Telemetry (without Status and Bucket State)
        telemetry_group = ui_widgets.create_condensed_telemetry_group(self)
        left_column_layout.addWidget(telemetry_group)
        
        # Row 3: Bucket State slider (full width)
        bucket_state_group = ui_widgets.create_bucket_state_widget(self)
        left_column_layout.addWidget(bucket_state_group)
        
        left_column_layout.addStretch()
        
        bottom_row_layout.addWidget(left_column, 1)
        
        # Right column: Terminal
        terminal_widget = self.create_terminal_output()
        bottom_row_layout.addWidget(terminal_widget, 1)
        
        return bottom_row
    
    def create_terminal_output(self):
        terminal_group = QGroupBox("Terminal Output")
        terminal_group.setAutoFillBackground(True)
        terminal_group.setStyleSheet(f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1d1d1d); padding-top: 16px; }}")
        terminal_layout = QVBoxLayout()
        terminal_layout.setContentsMargins(3, 6, 3, 2)
        terminal_layout.setSpacing(2)
        terminal_group.setLayout(terminal_layout)
        
        self.terminal_text = QTextEdit()
        self.terminal_text.setReadOnly(True)
        self.terminal_text.document().setMaximumBlockCount(500)  # Limit to 500 lines
        terminal_font = QFont("Monospace", 9)
        self.terminal_text.setFont(terminal_font)
        self.terminal_text.setStyleSheet(f"""
            QTextEdit {{
                background-color: #0d0d0d;
                color: {Colors.TEXT_MAIN};
                border: 1px solid #2a2a2a;
                border-radius: 2px;
            }}
            QScrollBar:vertical {{
                background: #0d0d0d;
                width: 10px;
                border: none;
                border-radius: 0px;
            }}
            QScrollBar::handle:vertical {{
                background: #2a2a2a;
                min-height: 20px;
                border-radius: 4px;
                margin: 2px;
            }}
            QScrollBar::handle:vertical:hover {{
                background: #3a3a3a;
            }}
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical {{
                height: 0px;
            }}
            QScrollBar::add-page:vertical,
            QScrollBar::sub-page:vertical {{
                background: none;
            }}
        """)
        terminal_layout.addWidget(self.terminal_text)
        
        return terminal_group
    
    def create_sidebar(self):
        self.sidebar_container = QWidget()
        sidebar_container_layout = QHBoxLayout()
        sidebar_container_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_container_layout.setSpacing(0)
        self.sidebar_container.setLayout(sidebar_container_layout)
        
        self.sidebar_widget = QWidget()
        self.sidebar_widget.setStyleSheet("background-color: #1a1a1a;")
        self.sidebar_widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        self.sidebar_widget.setMinimumWidth(300)
        sidebar_layout = QVBoxLayout()
        sidebar_layout.setContentsMargins(4, 4, 4, 4)
        sidebar_layout.setSpacing(6)
        self.sidebar_widget.setLayout(sidebar_layout)
        
        sidebar_layout.addStretch(1)

        hardware_group = ui_widgets.create_hardware_group(self)
        sidebar_layout.addWidget(hardware_group)
        if not self.robot.is_real_mode:
            hardware_group.setEnabled(False)
        
        system_group = ui_widgets.create_launch_group(self)
        sidebar_layout.addWidget(system_group)
        
        action_group = ui_widgets.create_action_control_group(self)
        sidebar_layout.addWidget(action_group)
        
        # Only show keyboard teleop in sim mode
        if not self.robot.is_real_mode:
            teleop_group = ui_widgets.create_teleop_control_group(self)
            teleop_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.MinimumExpanding)
            sidebar_layout.addWidget(teleop_group, 0)  # stretch factor 0 = can shrink
        
        sidebar_container_layout.addWidget(self.sidebar_widget)
        
        # Controls toggle button (stays visible when sidebar collapses)
        edge_tab_container = QWidget()
        edge_tab_container.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        edge_tab_container.setFixedWidth(30)
        edge_tab_layout = QVBoxLayout()
        edge_tab_layout.setContentsMargins(0, 0, 0, 0)
        edge_tab_layout.setSpacing(0)
        edge_tab_container.setLayout(edge_tab_layout)
        
        edge_tab_layout.addStretch(1)

        self.edge_tab = QPushButton("◀\n\nC\nO\nN\nT\nR\nO\nL\nS")
        self.edge_tab.setStyleSheet(f"""
            QPushButton {{
                background-color: {Colors.BG_MAIN};
                color: {Colors.STATUS_SUCCESS};
                border: none;
                border-left: 2px solid {Colors.STATUS_SUCCESS};
                font-size: 12px;
                font-weight: bold;
                padding: 10px 5px;
                letter-spacing: 2px;
            }}
        """)
        self.edge_tab.setFixedWidth(30)
        self.edge_tab.setFixedHeight(200)  # Fixed height for the button
        self.edge_tab.clicked.connect(self.toggle_sidebar)
        edge_tab_layout.addWidget(self.edge_tab)
        
        edge_tab_layout.addStretch(1)
        
        sidebar_container_layout.addWidget(edge_tab_container)
        
        return self.sidebar_container
    
    def swap_camera(self):
        self.swappable_camera_showing_front = not self.swappable_camera_showing_front
        if self.swappable_camera_showing_front:
            self.swappable_camera_group.setTitle("Front Camera")
            self.swap_camera_btn.setText("Switch to Rear Camera")
        else:
            self.swappable_camera_group.setTitle("Rear Camera")
            self.swap_camera_btn.setText("Switch to Front Camera")
    
    def swap_fisheye_view(self):
        """Toggle between Deposit View (0 deg) and Bucket View (180 deg)"""
        self.fisheye_showing_deposit = not self.fisheye_showing_deposit
        if self.fisheye_showing_deposit:
            # Deposit View - 0 deg
            self.set_fisheye_position(0)
            self.swap_fisheye_btn.setText("Switch to Bucket View")
        else:
            # Bucket View - 180 deg
            self.set_fisheye_position(180)
            self.swap_fisheye_btn.setText("Switch to Deposit View")
    
    def set_fisheye_position(self, degrees):
        """Set fisheye camera to specific position (0 or 180 degrees)"""
        import math
        position_radians = math.radians(degrees)
        self.robot.publish_camera_position(position_radians)
        self.robot.node.get_logger().info(f'Fisheye camera position set to {degrees} deg')
    
    def toggle_sidebar(self):
        self.sidebar_collapsed = not self.sidebar_collapsed
        if self.sidebar_collapsed:
            self.sidebar_widget.hide()
            self.sidebar_container.setMinimumWidth(SIDEBAR_COLLAPSED_WIDTH)
            self.sidebar_container.setMaximumWidth(SIDEBAR_COLLAPSED_WIDTH)
            self.edge_tab.setText("▶\n\nC\nO\nN\nT\nR\nO\nL\nS")  # Point right to open
        else:
            self.sidebar_widget.show()
            self.sidebar_container.setMinimumWidth(SIDEBAR_MIN_WIDTH)
            self.sidebar_container.setMaximumWidth(16777215)
            self.edge_tab.setText("◀\n\nC\nO\nN\nT\nR\nO\nL\nS")  # Point left to close
    
    def toggle_mode(self):
        self.robot.publish_mode_switch()

    def start_can_interface(self):
        self.robot.start_can_interface()
    
    def launch_hardware(self):
        if self.robot.launch_processes.get('hardware') is not None:
            self.robot.stop_system('hardware')
        else:
            self.robot.launch_hardware()
    
    def launch_system(self, system_name):
        if self.robot.launch_processes.get(system_name) is not None:
            self.robot.stop_system(system_name)
        else:
            self.robot.launch_system(system_name)
    
    def launch_rviz(self):
        self.robot.launch_rviz()

    def send_excavate_goal(self):
        if self.robot.is_excavating:
            self.robot.node.get_logger().info('Canceling excavation')
            self.robot.cancel_excavate_goal(self.excavate_cancel_callback)
        else:
            self.robot.is_excavating = True
            self.excavate_btn.setText("Cancel Excavation")
            self.operation_status_label.setText("Status: Excavating...")
            self.operation_status_label.setStyleSheet(Styles.status_label('warning') + " font-weight: bold;")
            self.robot.send_excavate_goal(
                self.excavate_goal_response_callback,
                self.excavate_result_callback,
                self.excavate_cancel_callback
            )
    
    def excavate_goal_response_callback(self, goal_handle):
        if not goal_handle.accepted:
            self.robot.node.get_logger().error('Excavate goal rejected')
            self.robot.is_excavating = False
            self.excavate_btn.setText("Excavate")
            self.operation_status_label.setText("Status: Goal rejected")
            self.operation_status_label.setStyleSheet(Styles.status_label('error') + " font-weight: bold;")
        else:
            self.robot.node.get_logger().info('Excavate goal accepted')
    
    def excavate_result_callback(self, future):
        result = future.result().result
        self.robot.is_excavating = False
        self.excavate_btn.setText("Excavate")
        
        if result.success:
            self.robot.node.get_logger().info('Excavation completed successfully')
            self.operation_status_label.setText("Status: Excavation completed")
            self.operation_status_label.setStyleSheet(Styles.status_label('active') + " font-weight: bold;")
        else:
            self.robot.node.get_logger().error('Excavation failed')
            self.operation_status_label.setText("Status: Excavation failed")
            self.operation_status_label.setStyleSheet(Styles.status_label('error') + " font-weight: bold;")
    
    def excavate_cancel_callback(self, future):
        self.robot.node.get_logger().info('Excavation canceled')
        self.robot.is_excavating = False
        self.excavate_btn.setText("Excavate")
        self.operation_status_label.setText("Status: Excavation canceled")
        self.operation_status_label.setStyleSheet(Styles.status_label('warning') + " font-weight: bold;")
    
    def send_deposit_goal(self):
        if self.robot.is_depositing:
            self.robot.node.get_logger().info('Canceling deposit')
            self.robot.cancel_deposit_goal(self.deposit_cancel_callback)
        else:
            self.robot.is_depositing = True
            self.deposit_btn.setText("Cancel Deposit")
            self.operation_status_label.setText("Status: Depositing...")
            self.operation_status_label.setStyleSheet(Styles.status_label('warning') + " font-weight: bold;")
            self.robot.send_deposit_goal(
                self.deposit_goal_response_callback,
                self.deposit_result_callback,
                self.deposit_cancel_callback
            )
    
    def deposit_goal_response_callback(self, goal_handle):
        if not goal_handle.accepted:
            self.robot.node.get_logger().error('Deposit goal rejected')
            self.robot.is_depositing = False
            self.deposit_btn.setText("Deposit")
            self.operation_status_label.setText("Status: Goal rejected")
            self.operation_status_label.setStyleSheet(Styles.status_label('error') + " font-weight: bold;")
        else:
            self.robot.node.get_logger().info('Deposit goal accepted')
    
    def deposit_result_callback(self, future):
        result = future.result().result
        self.robot.is_depositing = False
        self.deposit_btn.setText("Deposit")
        
        if result.success:
            self.robot.node.get_logger().info(f'Depositing completed: {result.message}')
            self.operation_status_label.setText(f"Status: {result.message}")
            self.operation_status_label.setStyleSheet(Styles.status_label('active') + " font-weight: bold;")
        else:
            self.robot.node.get_logger().error(f'Depositing failed: {result.message}')
            self.operation_status_label.setText(f"Status: Failed - {result.message}")
            self.operation_status_label.setStyleSheet(Styles.status_label('error') + " font-weight: bold;")
    
    def deposit_cancel_callback(self, future):
        self.robot.node.get_logger().info('Depositing canceled')
        self.robot.is_depositing = False
        self.deposit_btn.setText("Deposit")
        self.operation_status_label.setText("Status: Depositing canceled")
        self.operation_status_label.setStyleSheet(Styles.status_label('warning') + " font-weight: bold;")
    
    def send_home_goal(self):
        if self.robot.is_homing:
            self.robot.node.get_logger().info('Canceling homing')
            self.robot.cancel_home_goal(self.home_cancel_callback)
        else:
            self.robot.is_homing = True
            self.operation_status_label.setText("Status: Homing...")
            self.operation_status_label.setStyleSheet(Styles.status_label('warning') + " font-weight: bold;")
            self.robot.send_home_goal(
                self.home_goal_response_callback,
                self.home_result_callback,
                self.home_cancel_callback
            )
    
    def home_goal_response_callback(self, goal_handle):
        if not goal_handle.accepted:
            self.robot.node.get_logger().error('Home goal rejected')
            self.robot.is_homing = False
            self.operation_status_label.setText("Status: Goal rejected")
            self.operation_status_label.setStyleSheet(Styles.status_label('error') + " font-weight: bold;")
        else:
            self.robot.node.get_logger().info('Home goal accepted')
    
    def home_result_callback(self, future):
        result = future.result().result
        self.robot.is_homing = False
        
        if result.success:
            self.robot.node.get_logger().info(f'Homing completed: {result.message}')
            self.operation_status_label.setText(f"Status: {result.message}")
            self.operation_status_label.setStyleSheet(Styles.status_label('active') + " font-weight: bold;")
        else:
            self.robot.node.get_logger().error(f'Homing failed: {result.message}')
            self.operation_status_label.setText(f"Status: Failed - {result.message}")
            self.operation_status_label.setStyleSheet(Styles.status_label('error') + " font-weight: bold;")
    
    def home_cancel_callback(self, future):
        self.robot.node.get_logger().info('Homing canceled')
        self.robot.is_homing = False
        self.operation_status_label.setText("Status: Homing canceled")
        self.operation_status_label.setStyleSheet(Styles.status_label('warning') + " font-weight: bold;")
    
    def emergency_stop(self):
        if not self.emergency_stopped:
            self.robot.node.get_logger().error('EMERGENCY STOP TRIGGERED!')
            self.robot.publish_emergency_stop()
            self.operation_status_label.setText("Status: EMERGENCY STOP")
            self.operation_status_label.setStyleSheet(Styles.status_label('error') + " font-weight: bold;")

            if self.robot.is_excavating:
                self.robot.cancel_excavate_goal(lambda f: None)
            if self.robot.is_depositing:
                self.robot.cancel_deposit_goal(lambda f: None)
            if self.robot.is_homing:
                self.robot.cancel_home_goal(lambda f: None)
            if self.full_auto_active:
                self.robot.stop_navigation_client()
                self.full_auto_active = False
                self.auto_btn.setText("One Cycle Auto")
            
            self.emergency_stopped = True
            self.emergency_stop_btn.setText("Re-enable Robot")
            self.emergency_stop_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: #f57c00;
                    color: white;
                    border: none;
                    border-radius: 5px;
                    font-size: 14px;
                    font-weight: bold;
                    padding: 10px 8px;
                }}
                QPushButton:hover {{
                    background-color: #e65100;
                }}
                QPushButton:pressed {{
                    background-color: #d84315;
                }}
            """)
        else:
            self.robot.node.get_logger().info('Re-enabling robot from emergency stop')
            self.robot.publish_re_enable()
            self.operation_status_label.setText("Status: Robot re-enabled")
            self.operation_status_label.setStyleSheet(Styles.status_label('warning') + " font-weight: bold;")

            self.emergency_stopped = False
            self.emergency_stop_btn.setText("Emergency Stop")
            self.emergency_stop_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: #d32f2f;
                    color: white;
                    border: none;
                    border-radius: 5px;
                    font-size: 14px;
                    font-weight: bold;
                    padding: 10px 8px;
                }}
                QPushButton:hover {{
                    background-color: #b71c1c;
                }}
                QPushButton:pressed {{
                    background-color: #9a0007;
                }}
            """)
    
    def send_full_auto_goal(self):
        if self.full_auto_active:
            self.robot.node.get_logger().info('Stopping one cycle auto')
            self.robot.stop_navigation_client()
            self.full_auto_active = False
            self.auto_btn.setText("One Cycle Auto")
            self.operation_status_label.setText("Status: One cycle auto stopped")
            self.operation_status_label.setStyleSheet(Styles.status_label('warning') + " font-weight: bold;")
            return
        
        if self.robot.is_excavating or self.robot.is_depositing or self.robot.is_homing:
            self.robot.node.get_logger().warning('Operation already in progress')
            return
        
        self.robot.node.get_logger().info('Starting one cycle auto')
        self.full_auto_active = True
        self.auto_btn.setText("Stop One Cycle Auto")
        self.operation_status_label.setText("Status: One Cycle Auto Active...")
        self.operation_status_label.setStyleSheet(Styles.status_label('info') + " font-weight: bold;")
        
        if not self.robot.launch_navigation_client():
            self.full_auto_active = False
            self.auto_btn.setText("One Cycle Auto")
            self.operation_status_label.setText("Status: Failed to start")
            self.operation_status_label.setStyleSheet(Styles.status_label('error') + " font-weight: bold;")
    
    def rotate_fisheye_camera(self, delta_radians):
        new_position = self.fisheye_camera_position + delta_radians
        while new_position > math.pi:
            new_position -= 2 * math.pi
        while new_position < -math.pi:
            new_position += 2 * math.pi
        self.set_fisheye_camera_position(new_position)
    
    def set_fisheye_camera_position(self, position_radians):
        self.fisheye_camera_position = position_radians
        self.robot.publish_camera_position(position_radians)
        
        degrees = math.degrees(position_radians)
        self.camera_pos_label.setText(f"Position: {degrees:.0f} deg")
        self.robot.node.get_logger().info(f'Fisheye camera position set to {degrees:.1f} deg')
    
    def toggle_realsense_cameras(self):
        """Toggle RealSense camera streaming on/off"""
        enabled = self.robot.toggle_realsense_cameras()
        
        if enabled:
            self.realsense_toggle_btn.setText("RealSense: ON")
            self.realsense_toggle_btn.setStyleSheet(f"""
                QPushButton {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2e4e3e, stop:1 #264636);
                    color: {Colors.STATUS_SUCCESS};
                    border: 1px solid #3e5e4e;
                    border-radius: 4px;
                    font-size: 13px;
                    font-weight: bold;
                    padding: 6px 8px;
                }}
                QPushButton:hover {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #385848, stop:1 #305040);
                    border: 1px solid #486858;
                }}
                QPushButton:pressed {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #224232, stop:1 #1a3a2a);
                    border: 1px solid #2e4e3e;
                }}
            """)
        else:
            self.realsense_toggle_btn.setText("RealSense: OFF")
            self.realsense_toggle_btn.setStyleSheet(f"""
                QPushButton {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #3a3a3a, stop:1 #323232);
                    color: {Colors.TEXT_DIM};
                    border: 1px solid #4a4a4a;
                    border-radius: 4px;
                    font-size: 13px;
                    font-weight: bold;
                    padding: 6px 8px;
                }}
                QPushButton:hover {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #444444, stop:1 #3c3c3c);
                    border: 1px solid #545454;
                }}
                QPushButton:pressed {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2e2e2e, stop:1 #262626);
                    border: 1px solid #3a3a3a;
                }}
            """)
    
    def restart_can(self):
        """Restart CAN interface"""
        success = self.robot.restart_can_interface()
        
        if success and hasattr(self, 'can_restart_btn'):
            self.can_restart_btn.setStyleSheet(f"""
                QPushButton {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2e3e2e, stop:1 #263626);
                    color: {Colors.STATUS_SUCCESS};
                    border: 1px solid #3e4e3e;
                    border-radius: 4px;
                    font-size: 13px;
                    font-weight: bold;
                    padding: 6px 8px;
                }}
                QPushButton:hover {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #384838, stop:1 #305030);
                    border: 1px solid #486048;
                }}
                QPushButton:pressed {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #223222, stop:1 #1a2a1a);
                    border: 1px solid #2e3e2e;
                }}
            """)
            QTimer.singleShot(2000, lambda: self.can_restart_btn.setStyleSheet(f"""
                QPushButton {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2e2e2e, stop:1 #262626);
                    color: {Colors.TEXT_MAIN};
                    border: 1px solid #3e3e3e;
                    border-radius: 4px;
                    font-size: 13px;
                    font-weight: bold;
                    padding: 6px 8px;
                }}
                QPushButton:hover {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #383838, stop:1 #303030);
                    border: 1px solid #484848;
                }}
                QPushButton:pressed {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1a1a1a);
                    border: 1px solid #2e2e2e;
                }}
            """))
    
    def adjust_speed(self, speed_type, multiplier):
        if speed_type == 'linear':
            self.linear_speed *= multiplier
            self.linear_speed = min(self.max_linear_speed, max(0.0, self.linear_speed))
        elif speed_type == 'angular':
            self.angular_speed *= multiplier
            self.angular_speed = min(self.max_angular_speed, max(0.0, self.angular_speed))
        
        speed_html = f'<div style="text-align: right; line-height: 1.2;"><span style="color: #aaa; font-size: 13px;">Lin: {self.linear_speed:.2f} m/s | Ang: {self.angular_speed:.2f} rad/s</span><br><span style="color: #888; font-size: 11px; font-style: italic;">Q/Z: Linear Speed | E/C: Angular Speed</span></div>'
        self.speed_label.setText(speed_html)
        self.robot.node.get_logger().info(f"Speed adjusted - Linear: {self.linear_speed:.2f}, Angular: {self.angular_speed:.2f}")
    
    def teleop_key_press(self, key):
        if self.robot.is_real_mode:
            return
        self.teleop_keys[key] = True
        if key in self.teleop_buttons:
            self.teleop_buttons[key].setStyleSheet(self.teleop_active_style)
        self.publish_teleop_velocity()
    
    def teleop_key_release(self, key):
        if self.robot.is_real_mode:
            return
        self.teleop_keys[key] = False
        if key in self.teleop_buttons:
            self.teleop_buttons[key].setStyleSheet(self.teleop_button_style)
        self.publish_teleop_velocity()
    
    def publish_teleop_velocity(self):
        linear_x = 0.0
        angular_z = 0.0
        
        if self.teleop_keys['w']:
            linear_x = self.linear_speed
        elif self.teleop_keys['s']:
            linear_x = -self.linear_speed
        
        if self.teleop_keys['a']:
            angular_z = self.angular_speed
        elif self.teleop_keys['d']:
            angular_z = -self.angular_speed
        
        self.robot.publish_velocity(linear_x, angular_z)
        
        if self.teleop_keys['up']:
            new_position = max(self.robot.bucket_position - TELEOP_BUCKET_SPEED, TELEOP_BUCKET_LIMIT_MIN)
            self.robot.publish_bucket_position(new_position)
            self.robot.bucket_position = new_position
        elif self.teleop_keys['down']:
            new_position = min(self.robot.bucket_position + TELEOP_BUCKET_SPEED, TELEOP_BUCKET_LIMIT_MAX)
            self.robot.publish_bucket_position(new_position)
            self.robot.bucket_position = new_position
    
    # ROS and UI Update Callbacks
    def handle_robot_state_update(self):
        if hasattr(self, 'mode_label') and self.robot.is_real_mode:
            is_manual = (self.robot.robot_mode == "MANUAL")
            display_text = "Manual" if is_manual else "AUTO"
            self.mode_label.setText(display_text)
            
            if is_manual:
                self.mode_label.setStyleSheet(f"color: {Colors.STATUS_SUCCESS}; font-weight: bold; font-size: 14px; background-color: transparent;")
                self.mode_switch_btn.setText("Switch to Auto")
            else:
                self.mode_label.setStyleSheet(f"color: {Colors.STATUS_ERROR}; font-weight: bold; font-size: 14px; background-color: transparent;")
                self.mode_switch_btn.setText("Switch to Manual")
        
        if hasattr(self, 'pointlio_btn'):
            self.pointlio_btn.setText("Stop Point-LIO" if self.robot.launch_processes.get('pointlio') else "Point-LIO")

        if hasattr(self, 'mapping_btn'):
            self.mapping_btn.setText("Stop RTAB-Map" if self.robot.launch_processes.get('mapping') else "RTAB-Map")

        if hasattr(self, 'nav2_btn'):
            self.nav2_btn.setText("Stop Navigation2" if self.robot.launch_processes.get('nav2') else "Navigation2")
        
        if hasattr(self, 'hardware_btn'):
            self.hardware_btn.setText("Stop Hardware" if self.robot.launch_processes.get('hardware') else "Launch Hardware")
        
        if self.robot.robot_disabled:
            self.status_label.setText("Robot: DISABLED")
            self.status_label.setStyleSheet(Styles.status_label('error') + " font-weight: bold;")
            self.excavate_btn.setEnabled(False)
            self.deposit_btn.setEnabled(False)
            self.auto_btn.setEnabled(False)
        else:
            self.status_label.setText("Active")
            self.status_label.setStyleSheet(Styles.status_label('active') + " font-weight: bold;")
            if not (self.robot.is_excavating or self.robot.is_depositing or self.robot.is_navigating):
                self.excavate_btn.setEnabled(True)
                self.deposit_btn.setEnabled(True)
                self.auto_btn.setEnabled(True)
    
    def handle_control_state_update(self, msg):
        if msg.is_excavating:
            self.excavate_btn.setText("Cancel Excavation")
            self.operation_status_label.setText("Status: Excavating")
            self.operation_status_label.setStyleSheet(Styles.status_label('warning') + " font-weight: bold;")
        else:
            self.excavate_btn.setText("Excavate")
        
        if msg.is_depositing:
            self.deposit_btn.setText("Cancel Deposit")
            self.operation_status_label.setText("Status: Depositing")
            self.operation_status_label.setStyleSheet(Styles.status_label('warning') + " font-weight: bold;")
        else:
            self.deposit_btn.setText("Deposit")
        
        if msg.is_navigating:
            self.operation_status_label.setText("Status: Navigating")
            self.operation_status_label.setStyleSheet(Styles.status_label('info') + " font-weight: bold;")
        elif msg.status_message and not (msg.is_excavating or msg.is_depositing):
            self.operation_status_label.setText(f"Status: {msg.status_message}")
            self.operation_status_label.setStyleSheet(Styles.status_label('idle') + " font-weight: bold;")
        elif not (msg.is_excavating or msg.is_depositing or msg.is_navigating):
            self.operation_status_label.setText("Status: Idle")
            self.operation_status_label.setStyleSheet(Styles.status_label('idle') + " font-weight: bold;")
    
    def spin_ros(self):
        self.robot.spin_once()
    
    def update_ui(self):
        self.bandwidth_total_label.setText(f"{self.robot.bandwidth_avg_total:.2f} Mbps")
        self.bandwidth_total_current_label.setText(f"{self.robot.bandwidth_total:.2f} Mbps")
        self.bandwidth_rx_current_label.setText(f"{self.robot.bandwidth_rx:.2f} Mbps")
        self.bandwidth_tx_current_label.setText(f"{self.robot.bandwidth_tx:.2f} Mbps")

        bandwidth_percent = min(100, int((self.robot.bandwidth_avg_total / BANDWIDTH_MAX_MBPS) * 100))
        self.bandwidth_progress.setValue(bandwidth_percent)

        if bandwidth_percent >= int(BANDWIDTH_CRITICAL_THRESHOLD * 100):
            self.bandwidth_progress.setStyleSheet(f"QProgressBar::chunk {{ background-color: {Colors.STATUS_ERROR}; }}")
        elif bandwidth_percent >= int(BANDWIDTH_WARN_THRESHOLD * 100):
            self.bandwidth_progress.setStyleSheet(f"QProgressBar::chunk {{ background-color: {Colors.STATUS_WARNING}; }}")
        else:
            self.bandwidth_progress.setStyleSheet(f"QProgressBar::chunk {{ background-color: {Colors.STATUS_SUCCESS}; }}")
        
        self.linear_vel_label.setText(f"{self.robot.linear_velocity:.2f}")
        self.angular_vel_label.setText(f"{self.robot.angular_velocity:.2f}")

        self.position_x_label.setText(f"{self.robot.position_x:.2f}")
        self.position_y_label.setText(f"{self.robot.position_y:.2f}")
        self.position_z_label.setText(f"{self.robot.position_z:.2f}")
        self.orientation_roll_label.setText(f"{self.robot.orientation_roll:.2f}")
        self.orientation_pitch_label.setText(f"{self.robot.orientation_pitch:.2f}")
        self.orientation_yaw_label.setText(f"{self.robot.orientation_yaw:.2f}")

        self.bucket_angle_label.setText(f"{self.robot.bucket_position:.2f} rad")

        if self.robot.vibration_duty_cycle > 0.01:
            self.vibration_state_label.setText(f"ON ({self.robot.vibration_duty_cycle:.2f})")
            self.vibration_state_label.setStyleSheet(f"background-color: transparent; color: {Colors.STATUS_SUCCESS};")
        else:
            self.vibration_state_label.setText("OFF")
            self.vibration_state_label.setStyleSheet(f"background-color: transparent; color: {Colors.STATUS_ERROR};")
        
        self.power_voltage_label.setText(f"{self.robot.power_voltage:.2f}")
        self.power_current_label.setText(f"{self.robot.power_current:.2f}")
        self.power_watts_label.setText(f"{self.robot.power_watts:.2f}")
        self.power_energy_label.setText(f"{self.robot.power_energy_wh:.2f}")
        self.power_temp_label.setText(f"{self.robot.power_temp:.1f}")
        
        if not self.robot.is_real_mode and any(self.teleop_keys.values()):
            self.publish_teleop_velocity()

        self.update_bucket_leds()
    
    def update_cameras(self):
        """Separate camera update loop - runs less frequently than stats"""
        if not CV_AVAILABLE:
            return
        
        if self.swappable_camera_showing_front:
            self.update_camera_display(self.swappable_camera_label, self.robot.front_camera_image, 'front')
        else:
            self.update_camera_display(self.swappable_camera_label, self.robot.rear_camera_image, 'rear')
        
        self.update_camera_display(self.fisheye_camera_label, self.robot.fisheye_camera_image, 'fisheye')
    
    def update_bucket_leds(self):
        pos = self.robot.bucket_position
        self.bucket_slider.set_position(pos)

    def update_camera_display(self, label, cv_image, camera_name):
        """Update camera display with frame tracking to avoid reprocessing same frame"""
        if cv_image is None:
            return
        
        frame_id = id(cv_image)
        if self.last_camera_frame_ids.get(camera_name) == frame_id:
            return
        self.last_camera_frame_ids[camera_name] = frame_id
        
        height, width = cv_image.shape[:2]
        label_width = label.width()
        label_height = label.height()
        
        scale = min(label_width / width, label_height / height)
        new_width = int(width * scale)
        new_height = int(height * scale)
        
        resized = cv2.resize(cv_image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        label.setPixmap(pixmap)
    
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
            event.accept()
            return
        elif event.key() == Qt.Key_Escape and self.isFullScreen():
            self.showNormal()
            event.accept()
            return
        elif not self.robot.is_real_mode and not event.isAutoRepeat():
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
                self.adjust_speed('linear', TELEOP_SPEED_INCREMENT)
                handled = True
            elif event.key() == Qt.Key_Z:
                self.adjust_speed('linear', TELEOP_SPEED_DECREMENT)
                handled = True
            elif event.key() == Qt.Key_E:
                self.adjust_speed('angular', TELEOP_SPEED_INCREMENT)
                handled = True
            elif event.key() == Qt.Key_C:
                self.adjust_speed('angular', TELEOP_SPEED_DECREMENT)
                handled = True
            
            if handled:
                event.accept()
                return
        
        super().keyPressEvent(event)
    
    def keyReleaseEvent(self, event):
        if not self.robot.is_real_mode and not event.isAutoRepeat():
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
        
        super().keyReleaseEvent(event)
    
    def _do_shutdown(self):
        self.ros_timer.stop()
        self.ui_timer.stop()
        self.camera_timer.stop()
        QApplication.processEvents()
        self.robot.shutdown()
    
    def append_log(self, message):
        """Append a log message to the terminal output with ANSI color support"""
        try:
            # Color map for log levels
            level_colors = {
                'DEBUG': '#757575',   # Gray
                'INFO': '#e0e0e0',    # Light gray/white
                'WARN': '#ffca28',    # Yellow
                'ERROR': '#ef5350',   # Red
                'FATAL': '#ff5252',   # Bright red
            }
            
            # Extract log level from message format: [LEVEL] node_name: message
            import re
            level_match = re.match(r'\[(\w+)\]', message)
            
            if level_match:
                level = level_match.group(1)
                color = level_colors.get(level, '#e0e0e0')  # Default to white
                
                # Convert ANSI codes in the message, then wrap entire line in level color
                html_message = self.ansi_to_html(message)
                html_message = f'<span style="color:{color}">{html_message}</span>'
            else:
                # No level found, just convert ANSI codes
                html_message = self.ansi_to_html(message)
            
            self.terminal_text.append(html_message)
            # Auto-scroll to bottom
            self.terminal_text.verticalScrollBar().setValue(
                self.terminal_text.verticalScrollBar().maximum()
            )
        except Exception as e:
            # Fallback: append plain text if conversion fails
            print(f"Error converting log message: {e}")
            self.terminal_text.append(message.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;'))

    
    def ansi_to_html(self, text):
        """Convert ANSI color codes to HTML"""
        import re
        
        # ANSI color map to HTML colors
        ansi_colors = {
            '30': '#000000',  # Black
            '31': '#ef5350',  # Red
            '32': '#66bb6a',  # Green
            '33': '#ffca28',  # Yellow
            '34': '#42a5f5',  # Blue
            '35': '#ab47bc',  # Magenta
            '36': '#26c6da',  # Cyan
            '37': '#e0e0e0',  # White
            '90': '#757575',  # Bright Black (Gray)
            '91': '#ff5252',  # Bright Red
            '92': '#69f0ae',  # Bright Green
            '93': '#ffd54f',  # Bright Yellow
            '94': '#448aff',  # Bright Blue
            '95': '#e040fb',  # Bright Magenta
            '96': '#18ffff',  # Bright Cyan
            '97': '#ffffff',  # Bright White
        }
        
        # First escape HTML characters
        result = text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
        
        # Track open spans
        open_spans = 0
        
        # Replace ANSI color codes with HTML spans
        def replace_color(match):
            nonlocal open_spans
            codes = match.group(1).split(';')
            
            # Check for reset
            if '0' in codes or codes == ['']:
                if open_spans > 0:
                    open_spans -= 1
                    return '</span>'
                return ''
            
            # Build style attributes
            styles = []
            for code in codes:
                if code == '1':  # Bold
                    styles.append('font-weight:bold')
                elif code in ansi_colors:
                    styles.append(f'color:{ansi_colors[code]}')
            
            if styles:
                open_spans += 1
                return f'<span style="{";".join(styles)}">'
            return ''
        
        result = re.sub(r'\x1b\[([0-9;]*)m', replace_color, result)
        
        # Close any unclosed spans
        while open_spans > 0:
            result += '</span>'
            open_spans -= 1
        
        return result

    def closeEvent(self, event):
        self._do_shutdown()
        event.accept()


def main(args=None):
    app = QApplication(sys.argv)
    
    # Set application icon (for dock/taskbar)
    import os
    from ament_index_python.packages import get_package_share_directory
    try:
        pkg_dir = get_package_share_directory('lunabot_gui')
        icon_path = os.path.join(pkg_dir, 'resource', 'lunabot_icon.png')
        if os.path.exists(icon_path):
            app.setWindowIcon(QIcon(icon_path))
    except Exception as e:
        print(f"Could not load application icon: {e}")
    
    app.setStyle('Fusion')
    
    dark_palette = QPalette()
    dark_palette.setColor(QPalette.Window,          QColor(*Colors.PALETTE_WINDOW))
    dark_palette.setColor(QPalette.WindowText,      QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Base,            QColor(*Colors.PALETTE_BASE))
    dark_palette.setColor(QPalette.AlternateBase,   QColor(*Colors.PALETTE_WINDOW))
    dark_palette.setColor(QPalette.ToolTipBase,     QColor(*Colors.PALETTE_TOOLTIP_BASE))
    dark_palette.setColor(QPalette.ToolTipText,     QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Text,            QColor(255, 255, 255))
    dark_palette.setColor(QPalette.Button,          QColor(*Colors.PALETTE_WINDOW))
    dark_palette.setColor(QPalette.ButtonText,      QColor(255, 255, 255))
    dark_palette.setColor(QPalette.BrightText,      QColor(255, 0, 0))
    dark_palette.setColor(QPalette.Link,            QColor(*Colors.PALETTE_LINK))
    dark_palette.setColor(QPalette.Highlight,       QColor(*Colors.PALETTE_HIGHLIGHT))
    dark_palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    
    app.setPalette(dark_palette)
    
    def signal_handler(sig, frame):
        app.quit()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)
    
    window = LunabotGUI()
    app.aboutToQuit.connect(window._do_shutdown)
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
