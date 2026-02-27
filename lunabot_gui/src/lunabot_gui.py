#!/usr/bin/env python3
import sys
import signal
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QLabel, QPushButton, QGroupBox, QSizePolicy)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap, QPalette, QColor, QIcon

# Import refactored modules
from ros_interface import RobotInterface
import ui_widgets
from gui_styles import Colors, Styles, MAIN_STYLESHEET

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class LunabotGUI(QMainWindow):
    """Main GUI window for Lunabot control and monitoring"""
    
    def __init__(self):
        super().__init__()
        
        # Get mode parameter from ROS first
        # Need to init rclpy temporarily to read parameter
        if not rclpy.ok():
            rclpy.init()
        temp_node = Node('temp_param_reader')
        temp_node.declare_parameter('mode', True)
        mode_val = temp_node.get_parameter('mode').value
        temp_node.destroy_node()
        
        # Create ROS interface with mode
        self.robot = RobotInterface(mode_param=mode_val)
        
        # Setup ROS callbacks to update UI
        self.robot.on_robot_state_update = self.handle_robot_state_update
        self.robot.on_control_state_update = self.handle_control_state_update
        
        # UI state
        self.swappable_camera_showing_front = True
        self.sidebar_collapsed = False
        self.fisheye_camera_position = 0.0
        self.full_auto_active = False
        self.emergency_stopped = False
        
        # Teleop state (sim mode only)
        self.teleop_keys = {
            'w': False, 'a': False, 's': False, 'd': False,
            'up': False, 'down': False
        }
        self.linear_speed = 0.35
        self.angular_speed = 0.25
        self.max_linear_speed = 0.75
        self.max_angular_speed = 0.75
        
        # Setup UI
        self.init_ui()
        
        # Setup timers
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)  # 100 Hz
        
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(100)  # 10 Hz
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('Lunabot Control Panel')
        
        # Set window icon
        import os
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg_dir = get_package_share_directory('lunabot_gui')
            icon_path = os.path.join(pkg_dir, 'resource', 'lunabot_icon.png')
            if os.path.exists(icon_path):
                self.setWindowIcon(QIcon(icon_path))
        except Exception as e:
            print(f"Could not load window icon: {e}")
        
        # Get screen size and adjust window accordingly
        screen = QApplication.primaryScreen().geometry()
        # Use 90% of screen size, or target size (1600x900), whichever is smaller
        window_width = min(1600, int(screen.width() * 0.9))
        window_height = min(900, int(screen.height() * 0.9))
        # Center the window
        x = (screen.width() - window_width) // 2
        y = (screen.height() - window_height) // 2
        self.setGeometry(x, y, window_width, window_height)
        
        # Set reasonable size constraints (allows resizing)
        # Minimum size ensures text remains readable
        self.setMinimumSize(1000, 650)
        
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
        
        # Main content area
        content_widget = self.create_content_area()
        main_layout.addWidget(content_widget, 10)
        
        # Right sidebar (fixed width, no stretch)
        sidebar_container = self.create_sidebar()
        sidebar_container.setMinimumWidth(250)
        main_layout.addWidget(sidebar_container, 0)
    
    def create_content_area(self):
        """Create main content area with cameras and telemetry"""
        content_widget = QWidget()
        content_layout = QVBoxLayout()
        content_layout.setContentsMargins(4, 0, 0, 4)
        content_layout.setSpacing(5)
        content_widget.setLayout(content_layout)
        
        # Top row: Cameras (maximize space)
        top_row = self.create_camera_displays()
        content_layout.addWidget(top_row, 10)
        
        # Bottom row: Status, Mode, Telemetry, Camera Controls (minimize space)
        bottom_row = self.create_bottom_row()
        content_layout.addWidget(bottom_row, 0)
        
        return content_widget
    
    def create_camera_displays(self):
        """Create camera display widgets"""
        top_row = QWidget()
        top_row_layout = QHBoxLayout()
        top_row_layout.setContentsMargins(0, 0, 0, 0)
        top_row_layout.setSpacing(10)
        top_row.setLayout(top_row_layout)
        
        # Swappable camera (Front/Rear)
        swappable_container = QWidget()
        swappable_layout = QVBoxLayout()
        swappable_layout.setContentsMargins(0, 0, 0, 0)
        swappable_container.setLayout(swappable_layout)
        
        self.swappable_camera_group = QGroupBox("Front Camera")
        self.swappable_camera_group.setAutoFillBackground(True)
        self.swappable_camera_group.setStyleSheet(f"QGroupBox {{ background-color: {Colors.BG_BOX}; }}")
        swappable_camera_layout = QVBoxLayout()
        self.swappable_camera_label = QLabel("No camera feed")
        self.swappable_camera_label.setAlignment(Qt.AlignCenter)
        self.swappable_camera_label.setMinimumSize(320, 240)
        self.swappable_camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.swappable_camera_label.setStyleSheet(Styles.camera_label())
        swappable_camera_layout.addWidget(self.swappable_camera_label)
        self.swappable_camera_group.setLayout(swappable_camera_layout)
        swappable_layout.addWidget(self.swappable_camera_group)
        
        self.swap_camera_btn = QPushButton("⇄  Rear Camera")
        self.swap_camera_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {Colors.BG_MAIN};
                color: {Colors.STATUS_SUCCESS};
                border: none;
                border-top: 2px solid {Colors.STATUS_SUCCESS};
                font-size: 13px;
                font-weight: bold;
                padding: 5px;
            }}
            QPushButton:hover {{
                background-color: #3a3a3a;
                color: {Colors.STATUS_SUCCESS};
            }}
        """)
        self.swap_camera_btn.setMaximumHeight(30)
        self.swap_camera_btn.clicked.connect(self.swap_camera)
        swappable_layout.addWidget(self.swap_camera_btn)
        
        top_row_layout.addWidget(swappable_container, 1)
        
        # Fisheye camera
        fisheye_group = QGroupBox("Fisheye Camera")
        fisheye_group.setAutoFillBackground(True)
        fisheye_group.setStyleSheet(f"QGroupBox {{ background-color: {Colors.BG_BOX}; }}")
        fisheye_layout = QVBoxLayout()
        self.fisheye_camera_label = QLabel("No camera feed")
        self.fisheye_camera_label.setAlignment(Qt.AlignCenter)
        self.fisheye_camera_label.setMinimumSize(320, 240)
        self.fisheye_camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.fisheye_camera_label.setStyleSheet(Styles.camera_label())
        fisheye_layout.addWidget(self.fisheye_camera_label)
        fisheye_group.setLayout(fisheye_layout)
        top_row_layout.addWidget(fisheye_group, 1)
        
        return top_row
    
    def create_bottom_row(self):
        """Create bottom row with status, telemetry, and controls"""
        bottom_row = QWidget()
        bottom_row_layout = QHBoxLayout()
        bottom_row_layout.setContentsMargins(0, 0, 0, 0)
        bottom_row_layout.setSpacing(10)
        bottom_row_layout.setAlignment(Qt.AlignBottom)
        bottom_row.setLayout(bottom_row_layout)
        
        # Left column: Status + Mode + Telemetry
        left_column = QWidget()
        left_column_layout = QVBoxLayout()
        left_column_layout.setContentsMargins(0, 0, 0, 0)
        left_column_layout.setSpacing(3)
        left_column.setLayout(left_column_layout)
        
        # Status and Mode side by side
        status_mode_container = QWidget()
        status_mode_layout = QHBoxLayout()
        status_mode_layout.setContentsMargins(0, 0, 0, 0)
        status_mode_layout.setSpacing(3)
        status_mode_container.setLayout(status_mode_layout)
        
        status_group = ui_widgets.create_status_group(self)
        status_mode_layout.addWidget(status_group, 1)
        
        mode_group = ui_widgets.create_mode_group(self)
        status_mode_layout.addWidget(mode_group, 1)
        if not self.robot.is_real_mode:
            mode_group.setEnabled(False)
        
        left_column_layout.addWidget(status_mode_container)
        
        # Network
        network_group = ui_widgets.create_network_group(self)
        self.network_apply_btn.clicked.connect(self.apply_network_config)
        left_column_layout.addWidget(network_group)
        
        # Telemetry
        telemetry_group = ui_widgets.create_condensed_telemetry_group(self)
        left_column_layout.addWidget(telemetry_group)
        
        bottom_row_layout.addWidget(left_column, 2)
        
        # Right: Camera controls
        camera_group = ui_widgets.create_camera_control_group(self)
        bottom_row_layout.addWidget(camera_group, 1)
        
        return bottom_row
    
    def create_sidebar(self):
        """Create right sidebar with controls"""
        self.sidebar_container = QWidget()
        sidebar_container_layout = QHBoxLayout()
        sidebar_container_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_container_layout.setSpacing(0)
        self.sidebar_container.setLayout(sidebar_container_layout)
        
        # Sidebar content
        self.sidebar_widget = QWidget()
        self.sidebar_widget.setStyleSheet("background-color: #1a1a1a;")
        self.sidebar_widget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        self.sidebar_widget.setMinimumWidth(240)  # Prevent controls from compressing
        sidebar_layout = QVBoxLayout()
        sidebar_layout.setContentsMargins(4, 4, 4, 4)
        sidebar_layout.setSpacing(6)  # Reduced spacing to give more room to buttons
        self.sidebar_widget.setLayout(sidebar_layout)
        
        # Add stretch at top to push controls toward bottom (can compress if needed)
        sidebar_layout.addStretch(1)
        
        # Add control groups
        hardware_group = ui_widgets.create_hardware_group(self)
        sidebar_layout.addWidget(hardware_group)
        if not self.robot.is_real_mode:
            hardware_group.setEnabled(False)
        
        system_group = ui_widgets.create_launch_group(self)
        sidebar_layout.addWidget(system_group)
        
        action_group = ui_widgets.create_action_control_group(self)
        sidebar_layout.addWidget(action_group)
        
        teleop_group = ui_widgets.create_teleop_control_group(self)
        teleop_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.MinimumExpanding)
        sidebar_layout.addWidget(teleop_group, 0)  # stretch factor 0 = can shrink
        if self.robot.is_real_mode:
            teleop_group.setEnabled(False)
        
        sidebar_container_layout.addWidget(self.sidebar_widget)
        
        # Edge tab container - positions toggle button at bottom
        edge_tab_container = QWidget()
        edge_tab_container.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        edge_tab_container.setMaximumWidth(30)
        edge_tab_layout = QVBoxLayout()
        edge_tab_layout.setContentsMargins(0, 0, 0, 0)
        edge_tab_layout.setSpacing(0)
        edge_tab_container.setLayout(edge_tab_layout)
        
        # Add stretch at top to push button to bottom
        edge_tab_layout.addStretch(1)
        
        # Edge tab button
        self.edge_tab = QPushButton("▶\n\nC\nO\nN\nT\nR\nO\nL\nS")
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
            QPushButton:hover {{
                background-color: #3a3a3a;
                color: {Colors.STATUS_SUCCESS};
                border-left: 2px solid {Colors.STATUS_SUCCESS};
            }}
        """)
        self.edge_tab.setMaximumWidth(30)
        self.edge_tab.setFixedHeight(200)  # Fixed height for the button
        self.edge_tab.clicked.connect(self.toggle_sidebar)
        edge_tab_layout.addWidget(self.edge_tab)
        
        sidebar_container_layout.addWidget(edge_tab_container)
        
        return self.sidebar_container
    
    # UI Event Handlers
    def swap_camera(self):
        """Toggle between front and rear camera"""
        self.swappable_camera_showing_front = not self.swappable_camera_showing_front
        if self.swappable_camera_showing_front:
            self.swappable_camera_group.setTitle("Front Camera")
            self.swap_camera_btn.setText("⇄  Rear Camera")
        else:
            self.swappable_camera_group.setTitle("Rear Camera")
            self.swap_camera_btn.setText("⇄  Front Camera")
    
    def apply_network_config(self):
        """Apply network configuration changes"""
        self.robot.robot_host = self.robot_host_edit.text()
        self.robot.robot_user = self.robot_user_edit.text()
        # In real mode, remote execution is always enabled
        self.robot.is_remote = self.robot.is_real_mode
        
        self.robot.node.get_logger().info(f'Network config updated: {self.robot.robot_user}@{self.robot.robot_host}:{self.robot.robot_workspace}')
    
    def toggle_sidebar(self):
        """Toggle sidebar visibility"""
        self.sidebar_collapsed = not self.sidebar_collapsed
        if self.sidebar_collapsed:
            self.sidebar_widget.hide()
            self.edge_tab.setText("◀\n\nC\nO\nN\nT\nR\nO\nL\nS")
            self.sidebar_container.setMinimumWidth(30)
            self.sidebar_container.setMaximumWidth(30)
        else:
            self.sidebar_widget.show()
            self.edge_tab.setText("▶\n\nC\nO\nN\nT\nR\nO\nL\nS")
            self.sidebar_container.setMinimumWidth(250)
            self.sidebar_container.setMaximumWidth(16777215)
    
    def toggle_mode(self):
        """Toggle between manual and auto mode"""
        self.robot.publish_mode_switch()
    
    # Hardware and System Controls
    def start_can_interface(self):
        """Start CAN interface"""
        self.robot.start_can_interface()
    
    def launch_hardware(self):
        """Toggle launch/stop for hardware"""
        if self.robot.launch_processes.get('hardware') is not None:
            self.robot.stop_system('hardware')
        else:
            self.robot.launch_hardware()
    
    def launch_system(self, system_name):
        """Toggle launch/stop for a system (PointLIO, mapping, Nav2, localization)"""
        if self.robot.launch_processes.get(system_name) is not None:
            self.robot.stop_system(system_name)
        else:
            self.robot.launch_system(system_name)
    
    def launch_rviz(self):
        """Launch RViz2"""
        self.robot.launch_rviz()
    
    # Action Controls
    def send_excavate_goal(self):
        """Send excavation action goal or cancel"""
        if self.robot.is_excavating:
            self.robot.node.get_logger().info('Cancelling excavation')
            self.robot.cancel_excavate_goal(self.excavate_cancel_callback)
        else:
            self.robot.is_excavating = True
            self.excavate_btn.setText("Cancel Excavation")
            self.operation_status_label.setText("Status: Excavating...")
            self.operation_status_label.setStyleSheet("color: #ffa726; font-weight: bold; background-color: transparent;")
            self.robot.send_excavate_goal(
                self.excavate_goal_response_callback,
                self.excavate_result_callback,
                self.excavate_cancel_callback
            )
    
    def excavate_goal_response_callback(self, goal_handle):
        """Handle excavation goal response"""
        if not goal_handle.accepted:
            self.robot.node.get_logger().error('Excavate goal rejected')
            self.robot.is_excavating = False
            self.excavate_btn.setText("Excavate")
            self.operation_status_label.setText("Status: Goal rejected")
            self.operation_status_label.setStyleSheet("color: #d32f2f; font-weight: bold; background-color: transparent;")
        else:
            self.robot.node.get_logger().info('Excavate goal accepted')
    
    def excavate_result_callback(self, future):
        """Handle excavation result"""
        result = future.result().result
        self.robot.is_excavating = False
        self.excavate_btn.setText("Excavate")
        
        if result.success:
            self.robot.node.get_logger().info('Excavation completed successfully')
            self.operation_status_label.setText("Status: Excavation completed")
            self.operation_status_label.setStyleSheet("color: #66bb6a; font-weight: bold; background-color: transparent;")
        else:
            self.robot.node.get_logger().error('Excavation failed')
            self.operation_status_label.setText("Status: Excavation failed")
            self.operation_status_label.setStyleSheet("color: #d32f2f; font-weight: bold; background-color: transparent;")
    
    def excavate_cancel_callback(self, future):
        """Handle excavation cancellation"""
        self.robot.node.get_logger().info('Excavation cancelled')
        self.robot.is_excavating = False
        self.excavate_btn.setText("Excavate")
        self.operation_status_label.setText("Status: Excavation cancelled")
        self.operation_status_label.setStyleSheet("color: #ffa726; font-weight: bold; background-color: transparent;")
    
    def send_deposit_goal(self):
        """Send depositing action goal or cancel"""
        if self.robot.is_depositing:
            self.robot.node.get_logger().info('Cancelling deposit')
            self.robot.cancel_deposit_goal(self.deposit_cancel_callback)
        else:
            self.robot.is_depositing = True
            self.deposit_btn.setText("Cancel Deposit")
            self.operation_status_label.setText("Status: Depositing...")
            self.operation_status_label.setStyleSheet("color: #ffa726; font-weight: bold; background-color: transparent;")
            self.robot.send_deposit_goal(
                self.deposit_goal_response_callback,
                self.deposit_result_callback,
                self.deposit_cancel_callback
            )
    
    def deposit_goal_response_callback(self, goal_handle):
        """Handle deposit goal response"""
        if not goal_handle.accepted:
            self.robot.node.get_logger().error('Deposit goal rejected')
            self.robot.is_depositing = False
            self.deposit_btn.setText("Deposit")
            self.operation_status_label.setText("Status: Goal rejected")
            self.operation_status_label.setStyleSheet("color: #d32f2f; font-weight: bold; background-color: transparent;")
        else:
            self.robot.node.get_logger().info('Deposit goal accepted')
    
    def deposit_result_callback(self, future):
        """Handle deposit result"""
        result = future.result().result
        self.robot.is_depositing = False
        self.deposit_btn.setText("Deposit")
        
        if result.success:
            self.robot.node.get_logger().info(f'Depositing completed: {result.message}')
            self.operation_status_label.setText(f"Status: {result.message}")
            self.operation_status_label.setStyleSheet("color: #66bb6a; font-weight: bold; background-color: transparent;")
        else:
            self.robot.node.get_logger().error(f'Depositing failed: {result.message}')
            self.operation_status_label.setText(f"Status: Failed - {result.message}")
            self.operation_status_label.setStyleSheet("color: #d32f2f; font-weight: bold; background-color: transparent;")
    
    def deposit_cancel_callback(self, future):
        """Handle deposit cancellation"""
        self.robot.node.get_logger().info('Depositing cancelled')
        self.robot.is_depositing = False
        self.deposit_btn.setText("Deposit")
        self.operation_status_label.setText("Status: Depositing cancelled")
        self.operation_status_label.setStyleSheet("color: #ffa726; font-weight: bold; background-color: transparent;")
    
    def send_home_goal(self):
        """Send homing action goal or cancel"""
        if self.robot.is_homing:
            self.robot.node.get_logger().info('Cancelling homing')
            self.robot.cancel_home_goal(self.home_cancel_callback)
        else:
            self.robot.is_homing = True
            self.home_btn.setText("Cancel Homing")
            self.operation_status_label.setText("Status: Homing...")
            self.operation_status_label.setStyleSheet("color: #ffa726; font-weight: bold; background-color: transparent;")
            self.robot.send_home_goal(
                self.home_goal_response_callback,
                self.home_result_callback,
                self.home_cancel_callback
            )
    
    def home_goal_response_callback(self, goal_handle):
        """Handle home goal response"""
        if not goal_handle.accepted:
            self.robot.node.get_logger().error('Home goal rejected')
            self.robot.is_homing = False
            self.home_btn.setText("Home Actuators")
            self.operation_status_label.setText("Status: Goal rejected")
            self.operation_status_label.setStyleSheet("color: #d32f2f; font-weight: bold; background-color: transparent;")
        else:
            self.robot.node.get_logger().info('Home goal accepted')
    
    def home_result_callback(self, future):
        """Handle home result"""
        result = future.result().result
        self.robot.is_homing = False
        self.home_btn.setText("Home Actuators")
        
        if result.success:
            self.robot.node.get_logger().info(f'Homing completed: {result.message}')
            self.operation_status_label.setText(f"Status: {result.message}")
            self.operation_status_label.setStyleSheet("color: #66bb6a; font-weight: bold; background-color: transparent;")
        else:
            self.robot.node.get_logger().error(f'Homing failed: {result.message}')
            self.operation_status_label.setText(f"Status: Failed - {result.message}")
            self.operation_status_label.setStyleSheet("color: #d32f2f; font-weight: bold; background-color: transparent;")
    
    def home_cancel_callback(self, future):
        """Handle home cancellation"""
        self.robot.node.get_logger().info('Homing cancelled')
        self.robot.is_homing = False
        self.home_btn.setText("Home Actuators")
        self.operation_status_label.setText("Status: Homing cancelled")
        self.operation_status_label.setStyleSheet("color: #ffa726; font-weight: bold; background-color: transparent;")
    
    def emergency_stop(self):
        """Toggle emergency stop state"""
        if not self.emergency_stopped:
            # Trigger emergency stop
            self.robot.node.get_logger().error('EMERGENCY STOP TRIGGERED FROM GUI!')
            self.robot.publish_emergency_stop()
            self.operation_status_label.setText("Status: EMERGENCY STOP")
            self.operation_status_label.setStyleSheet("color: #d32f2f; font-weight: bold; background-color: transparent;")
            
            # Cancel any active operations
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
            
            # Update button to re-enable mode
            self.emergency_stopped = True
            self.emergency_stop_btn.setText("Re-enable Robot")
            self.emergency_stop_btn.setStyleSheet(Styles.orange_button())
        else:
            # Re-enable robot
            self.robot.node.get_logger().info('Re-enabling robot from emergency stop')
            self.robot.publish_re_enable()
            self.operation_status_label.setText("Status: Robot re-enabled")
            self.operation_status_label.setStyleSheet("color: #ffa726; font-weight: bold; background-color: transparent;")
            
            # Update button back to emergency stop mode
            self.emergency_stopped = False
            self.emergency_stop_btn.setText("Emergency Stop")
            self.emergency_stop_btn.setStyleSheet(Styles.red_button())
    
    def send_full_auto_goal(self):
        """Start one autonomous cycle"""
        if self.full_auto_active:
            self.robot.node.get_logger().info('Stopping one cycle auto')
            self.robot.stop_navigation_client()
            self.full_auto_active = False
            self.auto_btn.setText("One Cycle Auto")
            self.operation_status_label.setText("Status: One cycle auto stopped")
            self.operation_status_label.setStyleSheet("color: #ffa726; font-weight: bold; background-color: transparent;")
            return
        
        if self.robot.is_excavating or self.robot.is_depositing or self.robot.is_homing:
            self.robot.node.get_logger().warning('Operation already in progress')
            return
        
        self.robot.node.get_logger().info('Starting one cycle auto')
        self.full_auto_active = True
        self.auto_btn.setText("Stop One Cycle Auto")
        self.operation_status_label.setText("Status: One Cycle Auto Active...")
        self.operation_status_label.setStyleSheet("color: #2196f3; font-weight: bold; background-color: transparent;")
        
        if not self.robot.launch_navigation_client():
            self.full_auto_active = False
            self.auto_btn.setText("One Cycle Auto")
            self.operation_status_label.setText("Status: Failed to start")
            self.operation_status_label.setStyleSheet("color: #d32f2f; font-weight: bold; background-color: transparent;")
    
    # Camera Controls
    def rotate_fisheye_camera(self, delta_radians):
        """Rotate fisheye camera by delta angle"""
        new_position = self.fisheye_camera_position + delta_radians
        while new_position > 3.14159:
            new_position -= 6.28318
        while new_position < -3.14159:
            new_position += 6.28318
        self.set_fisheye_camera_position(new_position)
    
    def set_fisheye_camera_position(self, position_radians):
        """Set fisheye camera to absolute position"""
        self.fisheye_camera_position = position_radians
        self.robot.publish_camera_position(position_radians)
        
        degrees = position_radians * 180.0 / 3.14159
        self.camera_pos_label.setText(f"Position: {degrees:.0f}°")
        self.robot.node.get_logger().info(f'Fisheye camera position set to {degrees:.1f}°')
    
    # Teleop Controls (Sim Mode Only)
    def adjust_speed(self, speed_type, multiplier):
        """Adjust linear or angular speed"""
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
        """Handle teleop key press"""
        if self.robot.is_real_mode:
            return
        self.teleop_keys[key] = True
        if key in self.teleop_buttons:
            self.teleop_buttons[key].setStyleSheet(self.teleop_active_style)
        self.publish_teleop_velocity()
    
    def teleop_key_release(self, key):
        """Handle teleop key release"""
        if self.robot.is_real_mode:
            return
        self.teleop_keys[key] = False
        if key in self.teleop_buttons:
            self.teleop_buttons[key].setStyleSheet(self.teleop_button_style)
        self.publish_teleop_velocity()
    
    def publish_teleop_velocity(self):
        """Publish velocity command based on active teleop keys"""
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
        
        # Handle bucket control
        bucket_speed = 0.05
        if self.teleop_keys['up']:
            new_position = max(self.robot.bucket_position - bucket_speed, -1.57)
            self.robot.publish_bucket_position(new_position)
            self.robot.bucket_position = new_position
        elif self.teleop_keys['down']:
            new_position = min(self.robot.bucket_position + bucket_speed, 0.1)
            self.robot.publish_bucket_position(new_position)
            self.robot.bucket_position = new_position
    
    # ROS and UI Update Callbacks
    def handle_robot_state_update(self):
        """Handle robot state update from ROS interface"""
        # Update mode display
        if hasattr(self, 'mode_label') and self.robot.is_real_mode:
            is_manual = (self.robot.robot_mode == "MANUAL")
            display_text = "Manual" if is_manual else "AUTO"
            self.mode_label.setText(display_text)
            
            if is_manual:
                self.mode_label.setStyleSheet("color: #66bb6a; font-weight: bold; font-size: 14px; background-color: transparent;")
                self.mode_switch_btn.setText("Switch to Auto")
            else:
                self.mode_label.setStyleSheet("color: #d32f2f; font-weight: bold; font-size: 14px; background-color: transparent;")
                self.mode_switch_btn.setText("Switch to Manual")
        
        # Update launch button text based on running state
        if hasattr(self, 'pointlio_btn'):
            self.pointlio_btn.setText("Stop PointLIO" if self.robot.launch_processes.get('pointlio') else "PointLIO")
        
        if hasattr(self, 'mapping_btn'):
            self.mapping_btn.setText("Stop Mapping" if self.robot.launch_processes.get('mapping') else "Mapping")
        
        if hasattr(self, 'nav2_btn'):
            self.nav2_btn.setText("Stop Nav2" if self.robot.launch_processes.get('nav2') else "Nav2")
        
        if hasattr(self, 'localize_btn'):
            self.localize_btn.setText("Stop Localization" if self.robot.launch_processes.get('localization') else "Localize")
        
        if hasattr(self, 'hardware_btn'):
            self.hardware_btn.setText("Stop Hardware" if self.robot.launch_processes.get('hardware') else "Launch Hardware")
        
        # Update robot status
        if self.robot.robot_disabled:
            self.status_label.setText("Robot: DISABLED")
            self.status_label.setStyleSheet("color: #d32f2f; font-weight: bold; background-color: transparent;")
            self.excavate_btn.setEnabled(False)
            self.deposit_btn.setEnabled(False)
            self.auto_btn.setEnabled(False)
        else:
            self.status_label.setText("Active")
            self.status_label.setStyleSheet("color: #66bb6a; font-weight: bold; background-color: transparent;")
            if not (self.robot.is_excavating or self.robot.is_depositing or self.robot.is_navigating):
                self.excavate_btn.setEnabled(True)
                self.deposit_btn.setEnabled(True)
                self.auto_btn.setEnabled(True)
    
    def handle_control_state_update(self, msg):
        """Handle control state message"""
        # Update operation states and buttons
        if msg.is_excavating:
            self.excavate_btn.setText("Cancel Excavation")
            self.operation_status_label.setText("Status: Excavating")
            self.operation_status_label.setStyleSheet("color: #ffa726; font-weight: bold; background-color: transparent;")
        else:
            self.excavate_btn.setText("Excavate")
        
        if msg.is_depositing:
            self.deposit_btn.setText("Cancel Deposit")
            self.operation_status_label.setText("Status: Depositing")
            self.operation_status_label.setStyleSheet("color: #ffa726; font-weight: bold; background-color: transparent;")
        else:
            self.deposit_btn.setText("Deposit")
        
        if msg.is_navigating:
            self.operation_status_label.setText("Status: Navigating")
            self.operation_status_label.setStyleSheet("color: #2196f3; font-weight: bold; background-color: transparent;")
        elif msg.status_message and not (msg.is_excavating or msg.is_depositing):
            self.operation_status_label.setText(f"Status: {msg.status_message}")
            self.operation_status_label.setStyleSheet("color: #aaa; font-weight: bold; background-color: transparent;")
        elif not (msg.is_excavating or msg.is_depositing or msg.is_navigating):
            self.operation_status_label.setText("Status: Idle")
            self.operation_status_label.setStyleSheet("color: #aaa; font-weight: bold; background-color: transparent;")
    
    def spin_ros(self):
        """Spin ROS node to process callbacks"""
        self.robot.spin_once()
    
    def update_ui(self):
        """Update UI elements with latest data"""
        # Show total average bandwidth (since startup)
        self.bandwidth_total_label.setText(f"{self.robot.bandwidth_avg_total:.2f} Mbps")
        
        # Show current instantaneous bandwidth
        self.bandwidth_total_current_label.setText(f"{self.robot.bandwidth_total:.2f} Mbps")
        self.bandwidth_rx_current_label.setText(f"{self.robot.bandwidth_rx:.2f} Mbps")
        self.bandwidth_tx_current_label.setText(f"{self.robot.bandwidth_tx:.2f} Mbps")
        
        # Use total average for progress bar
        bandwidth_percent = min(100, int((self.robot.bandwidth_avg_total / 4.0) * 100))
        self.bandwidth_progress.setValue(bandwidth_percent)
        
        if bandwidth_percent >= 90:
            self.bandwidth_progress.setStyleSheet("QProgressBar::chunk { background-color: #e53935; }")
        elif bandwidth_percent >= 75:
            self.bandwidth_progress.setStyleSheet("QProgressBar::chunk { background-color: #fb8c00; }")
        else:
            self.bandwidth_progress.setStyleSheet("QProgressBar::chunk { background-color: #66bb6a; }")
        
        # Update velocity
        self.linear_vel_label.setText(f"{self.robot.linear_velocity:.2f}")
        self.angular_vel_label.setText(f"{self.robot.angular_velocity:.2f}")
        
        # Update position
        self.position_x_label.setText(f"{self.robot.position_x:.2f}")
        self.position_y_label.setText(f"{self.robot.position_y:.2f}")
        
        # Update bucket angle
        bucket_angle_deg = -self.robot.bucket_position * 180.0 / 3.14159
        self.bucket_angle_label.setText(f"{bucket_angle_deg:.1f}°")
        
        # Update vibration state based on duty cycle
        if self.robot.vibration_duty_cycle > 0.01:
            self.vibration_state_label.setText(f"ON ({self.robot.vibration_duty_cycle:.2f})")
            self.vibration_state_label.setStyleSheet("background-color: transparent; color: #66bb6a;")  # Green for ON
        else:
            self.vibration_state_label.setText("OFF")
            self.vibration_state_label.setStyleSheet("background-color: transparent; color: #d32f2f;")  # Red for OFF
        
        # Update power monitoring
        self.power_voltage_label.setText(f"{self.robot.power_voltage:.2f}")
        self.power_current_label.setText(f"{self.robot.power_current:.2f}")
        self.power_watts_label.setText(f"{self.robot.power_watts:.2f}")
        self.power_energy_label.setText(f"{self.robot.power_energy_kwh:.4f}")
        
        # Publish teleop velocity if any key is active (sim mode only)
        if not self.robot.is_real_mode and any(self.teleop_keys.values()):
            self.publish_teleop_velocity()
        
        # Update camera feeds
        if CV_AVAILABLE:
            if self.swappable_camera_showing_front:
                self.update_camera_display(self.swappable_camera_label, self.robot.front_camera_image)
            else:
                self.update_camera_display(self.swappable_camera_label, self.robot.rear_camera_image)
            self.update_camera_display(self.fisheye_camera_label, self.robot.fisheye_camera_image)
    
    def update_camera_display(self, label, cv_image):
        """Update a camera display label with OpenCV image"""
        if cv_image is None:
            return
        
        height, width = cv_image.shape[:2]
        label_width = label.width()
        label_height = label.height()
        
        scale = min(label_width / width, label_height / height)
        new_width = int(width * scale)
        new_height = int(height * scale)
        
        resized = cv2.resize(cv_image, (new_width, new_height))
        rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        label.setPixmap(pixmap)
    
    # Qt Event Handlers
    def keyPressEvent(self, event):
        """Handle key press events"""
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
        
        super().keyPressEvent(event)
    
    def keyReleaseEvent(self, event):
        """Handle key release events"""
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
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.ros_timer.stop()
        self.ui_timer.stop()
        QApplication.processEvents()
        
        self.robot.shutdown()
        event.accept()


def main(args=None):
    """Main entry point"""
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
    
    # Set dark theme
    app.setStyle('Fusion')
    
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
    
    app.setPalette(dark_palette)
    
    # Signal handling
    def signal_handler(sig, frame):
        app.quit()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)
    
    window = LunabotGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
