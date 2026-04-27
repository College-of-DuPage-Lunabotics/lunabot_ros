#!/usr/bin/env python3
import math

from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QColor, QFont, QPainter
from PyQt5.QtWidgets import (QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                              QLineEdit, QProgressBar, QPushButton, QSizePolicy,
                              QVBoxLayout, QWidget)

from gui_styles import Colors, Styles

# Font size constants (pt) for Monospace labels
_FONT_SM = 9    # dense telemetry values
_FONT_MD = 10   # subbox data values / status labels


def create_condensed_telemetry_group(app):
    group = QGroupBox("Telemetry")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1d1d1d); padding-top: 16px; }}")
    main_layout = QHBoxLayout()
    main_layout.setContentsMargins(3, 6, 3, 2)
    main_layout.setSpacing(8)
    group.setLayout(main_layout)
    
    power_group = QGroupBox("Power")
    power_group.setAutoFillBackground(True)
    power_group.setStyleSheet(Styles.subbox())
    power_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    power_layout = QVBoxLayout()
    power_layout.setSpacing(1)
    power_layout.setContentsMargins(3, 4, 3, 2)
    
    power_grid = QGridLayout()
    power_grid.setSpacing(1)
    power_grid.setHorizontalSpacing(2)
    power_grid.setVerticalSpacing(1)
    power_grid.setColumnStretch(1, 0)
    power_grid.setColumnStretch(4, 0)
    
    voltage_label = QLabel("V:")
    voltage_label.setFont(QFont("Monospace", _FONT_SM))
    voltage_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(voltage_label, 0, 0)
    app.power_voltage_label = QLabel("0.00")
    app.power_voltage_label.setFont(QFont("Monospace", _FONT_SM))
    app.power_voltage_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(app.power_voltage_label, 0, 1)
    voltage_unit_label = QLabel("V")
    voltage_unit_label.setFont(QFont("Monospace", _FONT_SM))
    voltage_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(voltage_unit_label, 0, 2)
    
    current_label = QLabel("I:")
    current_label.setFont(QFont("Monospace", _FONT_SM))
    current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(current_label, 1, 0)
    app.power_current_label = QLabel("0.00")
    app.power_current_label.setFont(QFont("Monospace", _FONT_SM))
    app.power_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(app.power_current_label, 1, 1)
    current_unit_label = QLabel("A")
    current_unit_label.setFont(QFont("Monospace", _FONT_SM))
    current_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(current_unit_label, 1, 2)
    
    power_grid.setColumnMinimumWidth(3, 8)

    power_label = QLabel("P:")
    power_label.setFont(QFont("Monospace", _FONT_SM))
    power_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(power_label, 0, 4)
    app.power_watts_label = QLabel("0.00")
    app.power_watts_label.setFont(QFont("Monospace", _FONT_SM))
    app.power_watts_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(app.power_watts_label, 0, 5)
    power_unit_label = QLabel("W")
    power_unit_label.setFont(QFont("Monospace", _FONT_SM))
    power_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(power_unit_label, 0, 6)
    
    energy_label = QLabel("E:")
    energy_label.setFont(QFont("Monospace", _FONT_SM))
    energy_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(energy_label, 1, 4)
    app.power_energy_label = QLabel("0.00")
    app.power_energy_label.setFont(QFont("Monospace", _FONT_SM))
    app.power_energy_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(app.power_energy_label, 1, 5)
    energy_unit_label = QLabel("Wh")
    energy_unit_label.setFont(QFont("Monospace", _FONT_SM))
    energy_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(energy_unit_label, 1, 6)
    
    temp_label = QLabel("T:")
    temp_label.setFont(QFont("Monospace", _FONT_SM))
    temp_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(temp_label, 2, 0)
    app.power_temp_label = QLabel("0.0")
    app.power_temp_label.setFont(QFont("Monospace", _FONT_SM))
    app.power_temp_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(app.power_temp_label, 2, 1)
    temp_unit_label = QLabel("°C")
    temp_unit_label.setFont(QFont("Monospace", _FONT_SM))
    temp_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(temp_unit_label, 2, 2)
    
    power_container = QHBoxLayout()
    power_container.addLayout(power_grid)
    power_container.addStretch()
    power_layout.addLayout(power_container)
    
    power_group.setLayout(power_layout)
    main_layout.addWidget(power_group, 0)
    
    velocity_group = QGroupBox("Velocity")
    velocity_group.setAutoFillBackground(True)
    velocity_group.setStyleSheet(Styles.subbox())
    velocity_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    velocity_layout = QVBoxLayout()
    velocity_layout.setSpacing(1)
    velocity_layout.setContentsMargins(3, 4, 3, 2)

    vel_layout = QGridLayout()
    vel_layout.setSpacing(1)
    vel_layout.setHorizontalSpacing(2)
    vel_layout.setVerticalSpacing(1)
    vel_layout.setColumnStretch(1, 0)
    
    linear_label = QLabel("Lin:")
    linear_label.setFont(QFont("Monospace", _FONT_SM))
    linear_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(linear_label, 0, 0)
    app.linear_vel_label = QLabel("0.00")
    app.linear_vel_label.setFont(QFont("Monospace", _FONT_SM))
    app.linear_vel_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(app.linear_vel_label, 0, 1)
    
    linear_unit_label = QLabel("m/s")
    linear_unit_label.setFont(QFont("Monospace", _FONT_SM))
    linear_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(linear_unit_label, 0, 2)
    
    angular_label = QLabel("Ang:")
    angular_label.setFont(QFont("Monospace", _FONT_SM))
    angular_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(angular_label, 1, 0)
    app.angular_vel_label = QLabel("0.00")
    app.angular_vel_label.setFont(QFont("Monospace", _FONT_SM))
    app.angular_vel_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(app.angular_vel_label, 1, 1)
    
    angular_unit_label = QLabel("rad/s")
    angular_unit_label.setFont(QFont("Monospace", _FONT_SM))
    angular_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(angular_unit_label, 1, 2)
    
    vel_container = QHBoxLayout()
    vel_container.addLayout(vel_layout)
    vel_container.addStretch()
    velocity_layout.addLayout(vel_container)
    velocity_layout.addStretch()
    
    velocity_group.setLayout(velocity_layout)
    main_layout.addWidget(velocity_group, 0)
    
    position_group = QGroupBox("Pose")
    position_group.setAutoFillBackground(True)
    position_group.setStyleSheet(Styles.subbox())
    position_group.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Preferred)
    position_layout = QVBoxLayout()
    position_layout.setSpacing(1)
    position_layout.setContentsMargins(3, 4, 3, 2)

    pos_layout = QGridLayout()
    pos_layout.setSpacing(1)
    pos_layout.setHorizontalSpacing(2)
    pos_layout.setVerticalSpacing(1)
    pos_layout.setColumnStretch(1, 0)
    pos_layout.setColumnStretch(4, 0)
    
    x_label = QLabel("X:")
    x_label.setFont(QFont("Monospace", _FONT_SM))
    x_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(x_label, 0, 0)
    app.position_x_label = QLabel("0.00")
    app.position_x_label.setFont(QFont("Monospace", _FONT_SM))
    app.position_x_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(app.position_x_label, 0, 1)
    x_unit_label = QLabel("m")
    x_unit_label.setFont(QFont("Monospace", _FONT_SM))
    x_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(x_unit_label, 0, 2)
    
    y_label = QLabel("Y:")
    y_label.setFont(QFont("Monospace", _FONT_SM))
    y_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(y_label, 1, 0)
    app.position_y_label = QLabel("0.00")
    app.position_y_label.setFont(QFont("Monospace", _FONT_SM))
    app.position_y_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(app.position_y_label, 1, 1)
    y_unit_label = QLabel("m")
    y_unit_label.setFont(QFont("Monospace", _FONT_SM))
    y_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(y_unit_label, 1, 2)
    
    z_label = QLabel("Z:")
    z_label.setFont(QFont("Monospace", _FONT_SM))
    z_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(z_label, 2, 0)
    app.position_z_label = QLabel("0.00")
    app.position_z_label.setFont(QFont("Monospace", _FONT_SM))
    app.position_z_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(app.position_z_label, 2, 1)
    z_unit_label = QLabel("m")
    z_unit_label.setFont(QFont("Monospace", _FONT_SM))
    z_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(z_unit_label, 2, 2)
    
    pos_layout.setColumnMinimumWidth(3, 8)
    
    roll_label = QLabel("R:")
    roll_label.setFont(QFont("Monospace", _FONT_SM))
    roll_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(roll_label, 0, 4)
    app.orientation_roll_label = QLabel("0.00")
    app.orientation_roll_label.setFont(QFont("Monospace", _FONT_SM))
    app.orientation_roll_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(app.orientation_roll_label, 0, 5)
    roll_unit_label = QLabel("rad")
    roll_unit_label.setFont(QFont("Monospace", _FONT_SM))
    roll_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(roll_unit_label, 0, 6)
    
    pitch_label = QLabel("P:")
    pitch_label.setFont(QFont("Monospace", _FONT_SM))
    pitch_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(pitch_label, 1, 4)
    app.orientation_pitch_label = QLabel("0.00")
    app.orientation_pitch_label.setFont(QFont("Monospace", _FONT_SM))
    app.orientation_pitch_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(app.orientation_pitch_label, 1, 5)
    pitch_unit_label = QLabel("rad")
    pitch_unit_label.setFont(QFont("Monospace", _FONT_SM))
    pitch_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(pitch_unit_label, 1, 6)
    
    yaw_label = QLabel("Y:")
    yaw_label.setFont(QFont("Monospace", _FONT_SM))
    yaw_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(yaw_label, 2, 4)
    app.orientation_yaw_label = QLabel("0.00")
    app.orientation_yaw_label.setFont(QFont("Monospace", _FONT_SM))
    app.orientation_yaw_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(app.orientation_yaw_label, 2, 5)
    yaw_unit_label = QLabel("rad")
    yaw_unit_label.setFont(QFont("Monospace", _FONT_SM))
    yaw_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(yaw_unit_label, 2, 6)
    
    pos_container = QHBoxLayout()
    pos_container.addLayout(pos_layout)
    pos_container.addStretch()
    position_layout.addLayout(pos_container)
    position_layout.addStretch()
    
    position_group.setLayout(position_layout)
    main_layout.addWidget(position_group, 0)
    
    vibration_group = QGroupBox("Vibration State")
    vibration_group.setAutoFillBackground(True)
    vibration_group.setStyleSheet(Styles.subbox())
    vibration_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    vibration_layout = QVBoxLayout()
    vibration_layout.setSpacing(1)
    vibration_layout.setContentsMargins(3, 4, 3, 2)

    vibration_value_container = QHBoxLayout()
    app.vibration_state_label = QLabel("OFF")
    app.vibration_state_label.setFont(QFont("Monospace", _FONT_SM))
    app.vibration_state_label.setStyleSheet(f"background-color: transparent; color: {Colors.STATUS_ERROR};")
    vibration_value_container.addWidget(app.vibration_state_label)
    vibration_value_container.addStretch()
    vibration_layout.addLayout(vibration_value_container)
    vibration_layout.addStretch()
    
    vibration_group.setLayout(vibration_layout)
    main_layout.addWidget(vibration_group, 0)
    
    bucket_angle_group = QGroupBox("Bucket Angle")
    bucket_angle_group.setAutoFillBackground(True)
    bucket_angle_group.setStyleSheet(Styles.subbox())
    bucket_angle_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    bucket_angle_layout = QVBoxLayout()
    bucket_angle_layout.setSpacing(1)
    bucket_angle_layout.setContentsMargins(3, 4, 3, 2)

    bucket_value_container = QHBoxLayout()
    app.bucket_angle_label = QLabel("0.00 rad")
    app.bucket_angle_label.setFont(QFont("Monospace", _FONT_SM))
    app.bucket_angle_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    bucket_value_container.addWidget(app.bucket_angle_label)
    bucket_value_container.addStretch()
    bucket_angle_layout.addLayout(bucket_value_container)
    bucket_angle_layout.addStretch()
    
    bucket_angle_group.setLayout(bucket_angle_layout)
    main_layout.addWidget(bucket_angle_group, 0)
    
    main_layout.addStretch()
    
    return group


def create_bandwidth_group(app):
    """Create bandwidth widget with stats and progress bar"""
    group = QGroupBox("Bandwidth")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1d1d1d); padding-top: 16px; }}")
    group.setMaximumHeight(100)
    layout = QVBoxLayout()
    layout.setContentsMargins(4, 6, 4, 2)
    layout.setSpacing(1)
    group.setLayout(layout)
    
    values_container = QHBoxLayout()
    values_container.setSpacing(6)

    total_avg_label = QLabel("Avg:")
    total_avg_label.setFont(QFont("Monospace", _FONT_SM))
    total_avg_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(total_avg_label)
    app.bandwidth_total_label = QLabel("0.00 Mbps")
    app.bandwidth_total_label.setFont(QFont("Monospace", _FONT_SM))
    app.bandwidth_total_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(app.bandwidth_total_label)
    
    values_container.addSpacing(8)

    total_current_label = QLabel("Total:")
    total_current_label.setFont(QFont("Monospace", _FONT_SM))
    total_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(total_current_label)
    app.bandwidth_total_current_label = QLabel("0.00 Mbps")
    app.bandwidth_total_current_label.setFont(QFont("Monospace", _FONT_SM))
    app.bandwidth_total_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(app.bandwidth_total_current_label)
    
    values_container.addSpacing(8)

    rx_current_label = QLabel("RX:")
    rx_current_label.setFont(QFont("Monospace", _FONT_SM))
    rx_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(rx_current_label)
    app.bandwidth_rx_current_label = QLabel("0.00 Mbps")
    app.bandwidth_rx_current_label.setFont(QFont("Monospace", _FONT_SM))
    app.bandwidth_rx_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(app.bandwidth_rx_current_label)
    
    values_container.addSpacing(8)

    tx_current_label = QLabel("TX:")
    tx_current_label.setFont(QFont("Monospace", _FONT_SM))
    tx_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(tx_current_label)
    app.bandwidth_tx_current_label = QLabel("0.00 Mbps")
    app.bandwidth_tx_current_label.setFont(QFont("Monospace", _FONT_SM))
    app.bandwidth_tx_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(app.bandwidth_tx_current_label)
    
    values_container.addStretch()
    layout.addLayout(values_container)
    
    layout.addSpacing(2)
    
    app.bandwidth_progress = QProgressBar()
    app.bandwidth_progress.setMaximum(100)
    app.bandwidth_progress.setValue(0)
    app.bandwidth_progress.setTextVisible(False)
    app.bandwidth_progress.setMaximumHeight(8)
    layout.addWidget(app.bandwidth_progress)
    
    return group


def create_bucket_state_widget(app):
    """Create standalone bucket state slider widget"""
    bucket_state_group = QGroupBox("Bucket State")
    bucket_state_group.setAutoFillBackground(True)
    bucket_state_group.setStyleSheet(f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1d1d1d); padding-top: 16px; }}")
    bucket_state_layout = QVBoxLayout()
    bucket_state_layout.setSpacing(6)
    bucket_state_layout.setContentsMargins(3, 18, 3, 2)

    # Progress bar style slider
    app.bucket_slider = BucketSliderWidget()
    app.bucket_slider.setMaximumHeight(8)
    bucket_state_layout.addWidget(app.bucket_slider)
    
    # Zone labels below slider (proportions automatically match slider zones)
    zone_labels_row = QHBoxLayout()
    zone_labels_row.setSpacing(0)
    zone_labels_row.setContentsMargins(0, 4, 0, 0)
    
    # Get zone proportions from slider widget
    deposit_width, travel_width, excavate_width = BucketSliderWidget.get_zone_proportions()
    
    deposit_lbl = QLabel("Deposit")
    deposit_lbl.setAlignment(Qt.AlignCenter)
    deposit_lbl.setStyleSheet(f"color: {Colors.TEXT_MAIN}; font-size: 12px; font-family: Monospace; background: transparent;")
    zone_labels_row.addWidget(deposit_lbl, deposit_width)
    
    travel_lbl = QLabel("Travel")
    travel_lbl.setAlignment(Qt.AlignCenter)
    travel_lbl.setStyleSheet(f"color: {Colors.TEXT_MAIN}; font-size: 12px; font-family: Monospace; background: transparent;")
    zone_labels_row.addWidget(travel_lbl, travel_width)
    
    excavate_lbl = QLabel("Excavate")
    excavate_lbl.setAlignment(Qt.AlignCenter)
    excavate_lbl.setStyleSheet(f"color: {Colors.TEXT_MAIN}; font-size: 12px; font-family: Monospace; background: transparent;")
    zone_labels_row.addWidget(excavate_lbl, excavate_width)
    
    bucket_state_layout.addLayout(zone_labels_row)
    
    bucket_state_group.setLayout(bucket_state_layout)
    return bucket_state_group


def create_mode_group(app):
    group = QGroupBox("Robot Mode")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1d1d1d); }}")
    group.setMaximumHeight(100)
    group.setMinimumWidth(220)
    layout = QHBoxLayout()
    layout.setSpacing(3)
    layout.setContentsMargins(4, 3, 4, 3)
    
    # In sim mode, show "Simulation" in grey. In real mode, show Manual/Auto with colors
    if not app.robot.is_real_mode:
        mode_text = "Simulation"
        mode_color = Colors.TEXT_DARKER
        button_text = "Switch to Auto"
    else:
        is_manual = (app.robot.robot_mode == "MANUAL")
        
        if is_manual:
            mode_text = "Manual"
            mode_color = Colors.STATUS_SUCCESS
            button_text = "Switch to Auto"
        else:
            mode_text = "Auto"
            mode_color = Colors.STATUS_ERROR
            button_text = "Switch to Manual"
    
    app.mode_label = QLabel(mode_text)
    app.mode_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
    app.mode_label.setStyleSheet(f"color: {mode_color}; font-weight: bold; font-size: 13px; background-color: transparent;")
    layout.addWidget(app.mode_label, 1)
    
    mode_switch_btn = QPushButton(button_text)
    mode_switch_btn.setStyleSheet(f"""
        QPushButton {{
            background-color: {Colors.BTN_INACTIVE};
            color: white;
            font-size: 11px;
            font-weight: bold;
            padding: 4px 10px;
            border: none;
            border-radius: 2px;
        }}
        QPushButton:hover {{
            background-color: {Colors.BTN_HOVER};
        }}
        QPushButton:pressed {{
            background-color: {Colors.BTN_PRESSED};
        }}
        QPushButton:disabled {{
            background-color: {Colors.BTN_DISABLED};
            color: {Colors.TEXT_GRAY};
            border: none;
            border-radius: 2px;
        }}
    """)
    mode_switch_btn.clicked.connect(app.toggle_mode)
    layout.addWidget(mode_switch_btn)
    app.mode_switch_btn = mode_switch_btn
    
    group.setLayout(layout)
    return group

def create_teleop_control_group(app):
    group = QGroupBox("Keyboard Teleop")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #202020, stop:1 #1a1a1a); }}")
    layout = QVBoxLayout()
    layout.setSpacing(4)
    layout.setContentsMargins(4, 4, 4, 4)
    
    # Top row: Bucket controls (left) and Speed info (right)
    top_row = QHBoxLayout()
    
    # Left side: Bucket controls
    bucket_section = QVBoxLayout()
    bucket_section.setSpacing(3)
    
    arrow_label = QLabel("Bucket")
    arrow_label.setStyleSheet(f"color: {Colors.TEXT_SECONDARY}; font-size: 11px; background-color: transparent;")
    arrow_label.setAlignment(Qt.AlignCenter)
    bucket_section.addWidget(arrow_label)
    
    button_style = f"""
        QPushButton {{
            background-color: {Colors.BTN_INACTIVE};
            color: white;
            border: none;
            border-radius: 2px;
            padding: 6px;
            font-size: 13px;
            font-weight: bold;
            min-width: 40px;
            min-height: 40px;
        }}
        QPushButton:hover {{
            background-color: {Colors.BTN_HOVER};
        }}
        QPushButton:pressed {{
            background-color: {Colors.BTN_PRESSED};
        }}
        QPushButton:disabled {{
            background-color: {Colors.BTN_DISABLED};
            color: {Colors.TEXT_GRAY};
            border: none;
            border-radius: 2px;
        }}
    """
    active_button_style = f"""
        QPushButton {{
            background-color: {Colors.BTN_ACTIVE};
            color: white;
            border: none;
            border-radius: 2px;
            padding: 6px;
            font-size: 13px;
            font-weight: bold;
            min-width: 40px;
            min-height: 40px;
        }}
    """
    
    arrow_button_layout = QHBoxLayout()
    arrow_button_layout.setSpacing(4)
    app.btn_up = QPushButton("↑")
    app.btn_down = QPushButton("↓")
    
    for btn in [app.btn_up, app.btn_down]:
        btn.setStyleSheet(button_style)
        btn.setFocusPolicy(Qt.NoFocus)
    
    app.btn_up.pressed.connect(lambda: app.teleop_key_press('up'))
    app.btn_up.released.connect(lambda: app.teleop_key_release('up'))
    app.btn_down.pressed.connect(lambda: app.teleop_key_press('down'))
    app.btn_down.released.connect(lambda: app.teleop_key_release('down'))
    
    arrow_button_layout.addWidget(app.btn_up)
    arrow_button_layout.addWidget(app.btn_down)
    bucket_section.addLayout(arrow_button_layout)
    
    top_row.addLayout(bucket_section)
    top_row.addStretch()
    
    # Right side: Speed info
    speed_section = QVBoxLayout()
    speed_section.setSpacing(0)
    
    speed_html = f'<div style="text-align: right; line-height: 1.2;"><span style="color: {Colors.TEXT_DIM}; font-size: 13px;">Lin: {app.linear_speed:.2f} m/s | Ang: {app.angular_speed:.2f} rad/s</span><br><span style="color: {Colors.TEXT_DARKER}; font-size: 11px; font-style: italic;">Q/Z: Linear Speed | E/C: Angular Speed</span></div>'
    app.speed_label = QLabel(speed_html)
    app.speed_label.setStyleSheet("background-color: transparent; padding: 0px; margin: 0px;")
    app.speed_label.setAlignment(Qt.AlignRight)
    speed_section.addWidget(app.speed_label)
    
    top_row.addLayout(speed_section)
    layout.addLayout(top_row)
    
    wasd_label = QLabel("Movement")
    wasd_label.setStyleSheet(f"color: {Colors.TEXT_SECONDARY}; font-size: 11px; margin-top: 2px; background-color: transparent;")
    wasd_label.setAlignment(Qt.AlignCenter)
    layout.addWidget(wasd_label)
    
    wasd_grid = QGridLayout()
    wasd_grid.setSpacing(12)  # Increased spacing for small screens
    
    app.btn_w = QPushButton("W")
    app.btn_a = QPushButton("A")
    app.btn_s = QPushButton("S")
    app.btn_d = QPushButton("D")
    
    for btn in [app.btn_w, app.btn_a, app.btn_s, app.btn_d]:
        btn.setStyleSheet(button_style)
        btn.setFocusPolicy(Qt.NoFocus)
    
    app.btn_w.pressed.connect(lambda: app.teleop_key_press('w'))
    app.btn_w.released.connect(lambda: app.teleop_key_release('w'))
    app.btn_a.pressed.connect(lambda: app.teleop_key_press('a'))
    app.btn_a.released.connect(lambda: app.teleop_key_release('a'))
    app.btn_s.pressed.connect(lambda: app.teleop_key_press('s'))
    app.btn_s.released.connect(lambda: app.teleop_key_release('s'))
    app.btn_d.pressed.connect(lambda: app.teleop_key_press('d'))
    app.btn_d.released.connect(lambda: app.teleop_key_release('d'))
    
    wasd_grid.addWidget(app.btn_w, 0, 1)
    wasd_grid.addWidget(app.btn_a, 1, 0)
    wasd_grid.addWidget(app.btn_s, 1, 1)
    wasd_grid.addWidget(app.btn_d, 1, 2)
    
    layout.addLayout(wasd_grid)
    
    app.teleop_buttons = {
        'w': app.btn_w, 'a': app.btn_a, 's': app.btn_s, 'd': app.btn_d,
        'up': app.btn_up, 'down': app.btn_down
    }
    app.teleop_button_style = button_style
    app.teleop_active_style = active_button_style
    
    group.setLayout(layout)
    return group


def create_launch_group(app):
    group = QGroupBox("Launch")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #202020, stop:1 #1a1a1a); padding-top: 24px; }}")
    layout = QVBoxLayout()
    layout.setSpacing(6)
    layout.setContentsMargins(4, 8, 4, 4)
    layout.setContentsMargins(4, 8, 4, 4)
    
    app.pointlio_btn = QPushButton("Point-LIO")
    app.pointlio_btn.setStyleSheet(f"""
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
    """)
    app.pointlio_btn.setMaximumHeight(32)
    app.pointlio_btn.clicked.connect(lambda: app.launch_system('pointlio'))
    layout.addWidget(app.pointlio_btn)

    app.mapping_btn = QPushButton("RTAB-Map")
    app.mapping_btn.setStyleSheet(f"""
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
    """)
    app.mapping_btn.setMaximumHeight(32)
    app.mapping_btn.clicked.connect(lambda: app.launch_system('mapping'))
    layout.addWidget(app.mapping_btn)

    app.nav2_btn = QPushButton("Navigation2")
    app.nav2_btn.setStyleSheet(f"""
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
    """)
    app.nav2_btn.setMaximumHeight(32)
    app.nav2_btn.clicked.connect(lambda: app.launch_system('nav2'))
    layout.addWidget(app.nav2_btn)
    
    rviz_btn = QPushButton("RViz2")
    rviz_btn.setStyleSheet(f"""
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
    """)
    rviz_btn.setMaximumHeight(32)
    rviz_btn.clicked.connect(app.launch_rviz)
    layout.addWidget(rviz_btn)
    
    group.setLayout(layout)
    return group


def create_action_control_group(app):
    group = QGroupBox("Actions")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #202020, stop:1 #1a1a1a); padding-top: 24px; }}")
    layout = QVBoxLayout()
    layout.setSpacing(6)
    layout.setContentsMargins(4, 8, 4, 4)
    
    app.excavate_btn = QPushButton("Excavate")
    app.excavate_btn.setStyleSheet(f"""
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
    """)
    app.excavate_btn.setMaximumHeight(32)
    app.excavate_btn.clicked.connect(app.send_excavate_goal)
    layout.addWidget(app.excavate_btn)
    
    app.deposit_btn = QPushButton("Deposit")
    app.deposit_btn.setStyleSheet(f"""
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
    """)
    app.deposit_btn.setMaximumHeight(32)
    app.deposit_btn.clicked.connect(app.send_deposit_goal)
    layout.addWidget(app.deposit_btn)
    
    app.auto_btn = QPushButton("One Cycle Auto")
    app.auto_btn.setStyleSheet(f"""
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
    """)
    app.auto_btn.setMaximumHeight(32)
    app.auto_btn.clicked.connect(app.send_full_auto_goal)
    layout.addWidget(app.auto_btn)
    
    app.emergency_stop_btn = QPushButton("Emergency Stop")
    app.emergency_stop_btn.setStyleSheet(f"""
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
    app.emergency_stop_btn.setMaximumHeight(50)
    app.emergency_stop_btn.clicked.connect(app.emergency_stop)
    app.emergency_stop_btn.setEnabled(app.robot.is_real_mode)
    layout.addWidget(app.emergency_stop_btn)
    
    app.operation_status_label = QLabel("Status: Idle")
    app.operation_status_label.setWordWrap(True)
    app.operation_status_label.setFont(QFont("Monospace", _FONT_MD))
    app.operation_status_label.setStyleSheet(f"color: {Colors.TEXT_MAIN}; font-weight: bold; background-color: transparent; padding: 3px 0px;")
    app.operation_status_label.setMinimumHeight(20)
    app.operation_status_label.setMaximumHeight(60)  # Limit status label height
    layout.addWidget(app.operation_status_label)
    
    group.setLayout(layout)
    return group


def create_camera_control_group(app):
    group = QGroupBox("Fisheye Camera Rotation")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"""
        QGroupBox {{ 
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2a2a2a, stop:1 #252525); 
            margin: 0px;
            font-weight: bold;
            padding-top: 8px;
            padding-left: 4px;
            padding-right: 4px;
            padding-bottom: 4px;
            color: {Colors.TEXT_MAIN};
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            subcontrol-position: top left;
            padding: 0 5px;
            top: 5px;
            left: 2px;
            color: {Colors.TEXT_MAIN};
        }}
    """)
    layout = QVBoxLayout()
    layout.setContentsMargins(10, 0, 10, 10)
    layout.setSpacing(0)
    
    app.camera_pos_label = QLabel("Position: 0°")
    app.camera_pos_label.setStyleSheet(f"color: {Colors.TEXT_DIM}; font-size: 14px; font-weight: bold; background-color: transparent; margin-top: -8px;")
    app.camera_pos_label.setAlignment(Qt.AlignCenter)
    app.camera_pos_label.setMaximumHeight(20)
    layout.addWidget(app.camera_pos_label)
    
    layout.addSpacing(2)

    button_row = QHBoxLayout()
    button_row.setSpacing(8)

    btn_0 = QPushButton("0°")
    btn_0.setStyleSheet(Styles.camera_button(padding="14px 10px"))
    btn_0.setFixedWidth(80)
    btn_0.clicked.connect(lambda: app.set_fisheye_camera_position(0.0))
    button_row.addWidget(btn_0, 0)

    rotation_label = QLabel("Fisheye Rotation")
    rotation_label.setStyleSheet(f"""
        color: #ffffff; 
        font-size: 16px; 
        font-weight: bold; 
        background: #404040;
        padding: 10px;
        border-radius: 4px;
    """)
    rotation_label.setAlignment(Qt.AlignCenter)
    rotation_label.setMinimumWidth(150)
    rotation_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    button_row.addWidget(rotation_label, 1)

    btn_180 = QPushButton("180°")
    btn_180.setStyleSheet(Styles.camera_button(padding="14px 10px"))
    btn_180.setFixedWidth(80)
    btn_180.clicked.connect(lambda: app.set_fisheye_camera_position(math.pi))
    button_row.addWidget(btn_180, 0)

    layout.addLayout(button_row)
    
    group.setLayout(layout)
    return group


class BucketSliderWidget(QWidget):
    """Horizontal 3-zone progress bar with position indicator, styled like bandwidth monitor."""

    # Zone boundaries (adjust these to change zone proportions)
    _MAX_POS = 0.0         # 0 rad (deposit)
    _MIN_POS = 1.5708      # π/2 rad (90 deg, excavation)
    _MID1    = 0.5236      # π/6 rad (30 deg, deposit ready threshold)
    _MID2    = 1.309       # 5π/12 rad (75 deg, excavation ready threshold)

    @classmethod
    def get_zone_proportions(cls):
        """Calculate zone width proportions for label alignment.
        Returns (deposit_width, travel_width, excavate_width) as integers."""
        total = cls._MIN_POS - cls._MAX_POS
        deposit_width = int(((cls._MID1 - cls._MAX_POS) / total) * 1000)
        travel_width = int(((cls._MID2 - cls._MID1) / total) * 1000)
        excavate_width = int(((cls._MIN_POS - cls._MID2) / total) * 1000)
        return deposit_width, travel_width, excavate_width

    def __init__(self, parent=None):
        super().__init__(parent)
        self._position = 0.7854  # default to center of range (travel position)
        self.setMinimumSize(100, 8)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_position(self, pos):
        self._position = max(self._MAX_POS, min(self._MIN_POS, pos))
        self.update()

    def paintEvent(self, event):
        from PyQt5.QtGui import QPen, QLinearGradient
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        total = self._MIN_POS - self._MAX_POS

        def _x(pos):
            # Map position to x coordinate (left=0 rad, right=max rad)
            return int((pos - self._MAX_POS) / total * w)

        x1 = _x(self._MID1)
        x2 = _x(self._MID2)

        r = 3  # corner radius matching progress bar

        # Background gradient - dark black
        from PyQt5.QtGui import QPainterPath
        bg_gradient = QLinearGradient(0, 0, 0, h)
        bg_gradient.setColorAt(0, QColor("#0d0d0d"))
        bg_gradient.setColorAt(1, QColor("#0a0a0a"))
        
        bar_rect = QRectF(0, 0, w, h)
        clip_path = QPainterPath()
        clip_path.addRoundedRect(bar_rect, r, r)
        painter.setClipPath(clip_path)
        
        painter.fillRect(bar_rect, bg_gradient)

        # Draw colored zone gradients
        def draw_zone_gradient(x_start, x_end, color_top, color_bottom):
            zone_gradient = QLinearGradient(0, 0, 0, h)
            zone_gradient.setColorAt(0, QColor(color_top))
            zone_gradient.setColorAt(1, QColor(color_bottom))
            painter.fillRect(QRectF(x_start, 0, x_end - x_start, h), zone_gradient)

        # Deposit zone (yellow/orange gradient)
        draw_zone_gradient(0, x1, "#ffa726", "#f57c00")
        
        # Travel zone (green gradient)
        draw_zone_gradient(x1, x2, "#66bb6a", "#43a047")
        
        # Excavate zone (red gradient)
        draw_zone_gradient(x2, w, "#ef5350", "#d32f2f")

        painter.setClipping(False)

        # Border matching QProgressBar
        painter.setPen(QPen(QColor("#3e3e3e"), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(QRectF(0.5, 0.5, w - 1, h - 1), r, r)

        # Position indicator line - taller and more prominent
        ix = _x(self._position)
        ind_w = 4
        ind_x = max(ind_w // 2, min(w - ind_w // 2, ix))
        
        # Draw darker outline for contrast
        painter.setPen(QPen(QColor("#000000"), ind_w + 2))
        painter.drawLine(ind_x, -3, ind_x, h + 3)
        
        # Draw bright white indicator line on top
        painter.setPen(QPen(QColor("#ffffff"), ind_w))
        painter.drawLine(ind_x, -3, ind_x, h + 3)




def create_hardware_group(app):
    group = QGroupBox("Hardware")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #202020, stop:1 #1a1a1a); padding-top: 24px; }}")
    layout = QVBoxLayout()
    layout.setSpacing(6)
    layout.setContentsMargins(4, 8, 4, 4)
    
    button_style = f"""
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
        QPushButton:disabled {{
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #1e1e1e, stop:1 #1a1a1a);
            color: #505050;
            border: 1px solid #2e2e2e;
        }}
    """
    
    app.can_btn = QPushButton("Start CAN")
    app.can_btn.setStyleSheet(button_style)
    app.can_btn.setMaximumHeight(32)
    app.can_btn.setEnabled(False)  # Greyed out by default
    app.can_btn.clicked.connect(app.start_can_interface)
    layout.addWidget(app.can_btn)
    
    app.can_restart_btn = QPushButton("Restart CAN")
    app.can_restart_btn.setStyleSheet(button_style)
    app.can_restart_btn.setMaximumHeight(32)
    app.can_restart_btn.setEnabled(False)  # Greyed out by default
    app.can_restart_btn.clicked.connect(app.restart_can)
    layout.addWidget(app.can_restart_btn)
    
    app.hardware_btn = QPushButton("Launch Hardware")
    app.hardware_btn.setStyleSheet(button_style)
    app.hardware_btn.setMaximumHeight(32)
    app.hardware_btn.setEnabled(False)  # Greyed out by default
    app.hardware_btn.clicked.connect(app.launch_hardware)
    layout.addWidget(app.hardware_btn)
    
    group.setLayout(layout)
    return group

