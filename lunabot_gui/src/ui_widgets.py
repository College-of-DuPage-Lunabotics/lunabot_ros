#!/usr/bin/env python3
from PyQt5.QtWidgets import (QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, 
                              QPushButton, QGridLayout, QProgressBar, QSizePolicy, QLineEdit)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from gui_styles import Colors, Styles


def create_condensed_telemetry_group(app):
    """Create condensed telemetry display in one row"""
    group = QGroupBox("Telemetry")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background-color: {Colors.BG_BOX}; padding-top: 16px; }}")
    main_layout = QHBoxLayout()
    main_layout.setContentsMargins(3, 6, 3, 2)
    main_layout.setSpacing(4)
    group.setLayout(main_layout)
    
    # Power Section
    power_group = QGroupBox("Power")
    power_group.setAutoFillBackground(True)
    power_group.setStyleSheet(Styles.subbox())
    power_layout = QVBoxLayout()
    power_layout.setSpacing(1)
    power_layout.setContentsMargins(3, 2, 3, 2)
    
    power_grid = QGridLayout()
    power_grid.setSpacing(1)
    power_grid.setHorizontalSpacing(2)
    power_grid.setColumnStretch(1, 0)
    power_grid.setColumnStretch(4, 0)
    
    # Left column: Voltage and Current
    voltage_label = QLabel("V:")
    voltage_label.setFont(QFont("Monospace", 9))
    voltage_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(voltage_label, 0, 0)
    app.power_voltage_label = QLabel("0.00")
    app.power_voltage_label.setFont(QFont("Monospace", 9))
    app.power_voltage_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(app.power_voltage_label, 0, 1)
    voltage_unit_label = QLabel("V")
    voltage_unit_label.setFont(QFont("Monospace", 9))
    voltage_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(voltage_unit_label, 0, 2)
    
    current_label = QLabel("I:")
    current_label.setFont(QFont("Monospace", 9))
    current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(current_label, 1, 0)
    app.power_current_label = QLabel("0.00")
    app.power_current_label.setFont(QFont("Monospace", 9))
    app.power_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(app.power_current_label, 1, 1)
    current_unit_label = QLabel("A")
    current_unit_label.setFont(QFont("Monospace", 9))
    current_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(current_unit_label, 1, 2)
    
    # Add spacing column
    power_grid.setColumnMinimumWidth(3, 8)
    
    # Right column: Power and Energy
    power_label = QLabel("P:")
    power_label.setFont(QFont("Monospace", 9))
    power_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(power_label, 0, 4)
    app.power_watts_label = QLabel("0.00")
    app.power_watts_label.setFont(QFont("Monospace", 9))
    app.power_watts_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(app.power_watts_label, 0, 5)
    power_unit_label = QLabel("W")
    power_unit_label.setFont(QFont("Monospace", 9))
    power_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(power_unit_label, 0, 6)
    
    energy_label = QLabel("E:")
    energy_label.setFont(QFont("Monospace", 9))
    energy_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(energy_label, 1, 4)
    app.power_energy_label = QLabel("0.0000")
    app.power_energy_label.setFont(QFont("Monospace", 9))
    app.power_energy_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(app.power_energy_label, 1, 5)
    energy_unit_label = QLabel("kWh")
    energy_unit_label.setFont(QFont("Monospace", 9))
    energy_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    power_grid.addWidget(energy_unit_label, 1, 6)
    
    power_container = QHBoxLayout()
    power_container.addLayout(power_grid)
    power_container.addStretch()
    power_layout.addLayout(power_container)
    
    power_group.setLayout(power_layout)
    main_layout.addWidget(power_group, 1)
    
    # Velocity Section
    velocity_group = QGroupBox("Velocity")
    velocity_group.setAutoFillBackground(True)
    velocity_group.setStyleSheet(Styles.subbox())
    velocity_layout = QVBoxLayout()
    velocity_layout.setSpacing(1)
    velocity_layout.setContentsMargins(3, 2, 3, 2)
    
    vel_layout = QGridLayout()
    vel_layout.setSpacing(1)
    vel_layout.setHorizontalSpacing(2)
    vel_layout.setColumnStretch(1, 0)
    
    linear_label = QLabel("Lin:")
    linear_label.setFont(QFont("Monospace", 9))
    linear_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(linear_label, 0, 0)
    app.linear_vel_label = QLabel("0.00")
    app.linear_vel_label.setFont(QFont("Monospace", 9))
    app.linear_vel_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(app.linear_vel_label, 0, 1)
    
    linear_unit_label = QLabel("m/s")
    linear_unit_label.setFont(QFont("Monospace", 9))
    linear_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(linear_unit_label, 0, 2)
    
    angular_label = QLabel("Ang:")
    angular_label.setFont(QFont("Monospace", 9))
    angular_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(angular_label, 1, 0)
    app.angular_vel_label = QLabel("0.00")
    app.angular_vel_label.setFont(QFont("Monospace", 9))
    app.angular_vel_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(app.angular_vel_label, 1, 1)
    
    angular_unit_label = QLabel("rad/s")
    angular_unit_label.setFont(QFont("Monospace", 9))
    angular_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(angular_unit_label, 1, 2)
    
    vel_container = QHBoxLayout()
    vel_container.addLayout(vel_layout)
    vel_container.addStretch()
    velocity_layout.addLayout(vel_container)
    
    velocity_group.setLayout(velocity_layout)
    main_layout.addWidget(velocity_group, 1)
    
    # Position Section
    position_group = QGroupBox("Position")
    position_group.setAutoFillBackground(True)
    position_group.setStyleSheet(Styles.subbox())
    position_layout = QVBoxLayout()
    position_layout.setSpacing(1)
    position_layout.setContentsMargins(3, 2, 3, 2)
    
    pos_layout = QGridLayout()
    pos_layout.setSpacing(1)
    pos_layout.setHorizontalSpacing(2)
    pos_layout.setColumnStretch(1, 0)
    
    x_label = QLabel("X:")
    x_label.setFont(QFont("Monospace", 9))
    x_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(x_label, 0, 0)
    app.position_x_label = QLabel("0.00")
    app.position_x_label.setFont(QFont("Monospace", 9))
    app.position_x_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(app.position_x_label, 0, 1)
    
    x_unit_label = QLabel("m")
    x_unit_label.setFont(QFont("Monospace", 9))
    x_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(x_unit_label, 0, 2)
    
    y_label = QLabel("Y:")
    y_label.setFont(QFont("Monospace", 9))
    y_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(y_label, 1, 0)
    app.position_y_label = QLabel("0.00")
    app.position_y_label.setFont(QFont("Monospace", 9))
    app.position_y_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(app.position_y_label, 1, 1)
    
    y_unit_label = QLabel("m")
    y_unit_label.setFont(QFont("Monospace", 9))
    y_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(y_unit_label, 1, 2)
    
    pos_container = QHBoxLayout()
    pos_container.addLayout(pos_layout)
    pos_container.addStretch()
    position_layout.addLayout(pos_container)
    
    position_group.setLayout(position_layout)
    main_layout.addWidget(position_group, 1)
    
    # Bucket Angle Section
    bucket_group = QGroupBox("Bucket Angle")
    bucket_group.setAutoFillBackground(True)
    bucket_group.setStyleSheet(Styles.subbox())
    bucket_layout = QVBoxLayout()
    bucket_layout.setSpacing(1)
    bucket_layout.setContentsMargins(3, 2, 3, 2)
    
    bucket_value_container = QHBoxLayout()
    app.bucket_angle_label = QLabel("0.0°")
    app.bucket_angle_label.setFont(QFont("Monospace", 9))
    app.bucket_angle_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    bucket_value_container.addWidget(app.bucket_angle_label)
    bucket_value_container.addStretch()
    bucket_layout.addLayout(bucket_value_container)
    
    bucket_group.setLayout(bucket_layout)
    main_layout.addWidget(bucket_group, 1)
    
    # Vibration State Section
    vibration_group = QGroupBox("Vibration State")
    vibration_group.setAutoFillBackground(True)
    vibration_group.setStyleSheet(Styles.subbox())
    vibration_layout = QVBoxLayout()
    vibration_layout.setSpacing(1)
    vibration_layout.setContentsMargins(3, 2, 3, 2)
    
    vibration_value_container = QHBoxLayout()
    app.vibration_state_label = QLabel("OFF")
    app.vibration_state_label.setFont(QFont("Monospace", 9))
    app.vibration_state_label.setStyleSheet(f"background-color: transparent; color: {Colors.STATUS_ERROR};")  # Red for OFF
    vibration_value_container.addWidget(app.vibration_state_label)
    vibration_value_container.addStretch()
    vibration_layout.addLayout(vibration_value_container)
    
    vibration_group.setLayout(vibration_layout)
    main_layout.addWidget(vibration_group, 1)
    
    return group


def create_network_group(app):
    """Create network monitoring and configuration display"""
    group = QGroupBox("Network")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background-color: {Colors.BG_BOX}; padding-top: 16px; }}")
    main_layout = QHBoxLayout()
    main_layout.setContentsMargins(3, 6, 3, 2)
    main_layout.setSpacing(4)
    group.setLayout(main_layout)
    
    # Bandwidth Section (left)
    bandwidth_group = QGroupBox("Bandwidth")
    bandwidth_group.setAutoFillBackground(True)
    bandwidth_group.setStyleSheet(Styles.subbox())
    bandwidth_layout = QVBoxLayout()
    bandwidth_layout.setSpacing(1)
    bandwidth_layout.setContentsMargins(3, 2, 3, 2)
    
    # All bandwidth values on one line
    values_container = QHBoxLayout()
    values_container.setSpacing(6)
    
    # Total Avg
    total_avg_label = QLabel("Avg:")
    total_avg_label.setFont(QFont("Monospace", 9))
    total_avg_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(total_avg_label)
    app.bandwidth_total_label = QLabel("0.00 Mbps")
    app.bandwidth_total_label.setFont(QFont("Monospace", 9))
    app.bandwidth_total_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(app.bandwidth_total_label)
    
    values_container.addSpacing(8)
    
    # Total Current
    total_current_label = QLabel("Total:")
    total_current_label.setFont(QFont("Monospace", 9))
    total_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(total_current_label)
    app.bandwidth_total_current_label = QLabel("0.00 Mbps")
    app.bandwidth_total_current_label.setFont(QFont("Monospace", 9))
    app.bandwidth_total_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(app.bandwidth_total_current_label)
    
    values_container.addSpacing(8)
    
    # RX Current
    rx_current_label = QLabel("RX:")
    rx_current_label.setFont(QFont("Monospace", 9))
    rx_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(rx_current_label)
    app.bandwidth_rx_current_label = QLabel("0.00 Mbps")
    app.bandwidth_rx_current_label.setFont(QFont("Monospace", 9))
    app.bandwidth_rx_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(app.bandwidth_rx_current_label)
    
    values_container.addSpacing(8)
    
    # TX Current
    tx_current_label = QLabel("TX:")
    tx_current_label.setFont(QFont("Monospace", 9))
    tx_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(tx_current_label)
    app.bandwidth_tx_current_label = QLabel("0.00 Mbps")
    app.bandwidth_tx_current_label.setFont(QFont("Monospace", 9))
    app.bandwidth_tx_current_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(app.bandwidth_tx_current_label)
    
    values_container.addStretch()
    bandwidth_layout.addLayout(values_container)
    
    bandwidth_layout.addSpacing(2)
    
    # Progress bar for bandwidth
    app.bandwidth_progress = QProgressBar()
    app.bandwidth_progress.setMaximum(100)
    app.bandwidth_progress.setValue(0)
    app.bandwidth_progress.setTextVisible(False)
    app.bandwidth_progress.setMaximumHeight(8)
    bandwidth_layout.addWidget(app.bandwidth_progress)
    
    bandwidth_group.setLayout(bandwidth_layout)
    bandwidth_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
    main_layout.addWidget(bandwidth_group, 0)  # Don't stretch bandwidth
    
    # Network Config Section (right)
    config_group = QGroupBox("Network Config")
    config_group.setAutoFillBackground(True)
    config_group.setStyleSheet(Styles.subbox())
    config_layout = QHBoxLayout()
    config_layout.setSpacing(5)
    config_layout.setContentsMargins(3, 2, 3, 2)
    
    # Robot Host
    host_label = QLabel("Host:")
    host_label.setFont(QFont("Monospace", 9))
    host_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    config_layout.addWidget(host_label)
    
    app.robot_host_edit = QLineEdit(app.robot.robot_host)
    app.robot_host_edit.setFont(QFont("Monospace", 9))
    app.robot_host_edit.setStyleSheet(f"background-color: {Colors.BG_MAIN}; color: {Colors.TEXT_MAIN}; padding: 2px; border: 1px solid {Colors.BORDER_DARK};")
    app.robot_host_edit.setMaximumHeight(20)
    config_layout.addWidget(app.robot_host_edit)
    
    config_layout.addSpacing(6)
    
    # Robot User
    user_label = QLabel("User:")
    user_label.setFont(QFont("Monospace", 9))
    user_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    config_layout.addWidget(user_label)
    
    app.robot_user_edit = QLineEdit(app.robot.robot_user)
    app.robot_user_edit.setFont(QFont("Monospace", 9))
    app.robot_user_edit.setStyleSheet(f"background-color: {Colors.BG_MAIN}; color: {Colors.TEXT_MAIN}; padding: 2px; border: 1px solid {Colors.BORDER_DARK};")
    app.robot_user_edit.setMaximumHeight(20)
    config_layout.addWidget(app.robot_user_edit)
    
    config_layout.addSpacing(6)
    
    # Apply button
    app.network_apply_btn = QPushButton("Apply")
    app.network_apply_btn.setStyleSheet(f"""
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
    app.network_apply_btn.setMinimumWidth(70)
    app.network_apply_btn.setMaximumHeight(24)
    config_layout.addWidget(app.network_apply_btn)
    
    config_layout.addStretch()
    config_group.setLayout(config_layout)
    config_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    main_layout.addWidget(config_group, 1)  # Let config stretch more
    
    return group


def create_combined_telemetry_group(app):
    """Create telemetry display with sectioned layout"""
    group = QGroupBox("Telemetry")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background-color: {Colors.BG_BOX}; }}")
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
    bucket_group.setStyleSheet(Styles.subbox())
    bucket_layout = QVBoxLayout()
    bucket_layout.setSpacing(2)
    bucket_layout.setContentsMargins(6, 6, 6, 6)
    
    bucket_value_container = QHBoxLayout()
    app.bucket_angle_label = QLabel("0.0°")
    app.bucket_angle_label.setFont(QFont("Monospace", 10))
    app.bucket_angle_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    bucket_value_container.addWidget(app.bucket_angle_label)
    bucket_value_container.addStretch()
    bucket_layout.addLayout(bucket_value_container)
    
    bucket_group.setLayout(bucket_layout)
    col1_layout.addWidget(bucket_group, 1)
    
    # Vibration State Section
    vibration_group = QGroupBox("Vibration State")
    vibration_group.setAutoFillBackground(True)
    vibration_group.setStyleSheet(Styles.subbox())
    vibration_layout = QVBoxLayout()
    vibration_layout.setSpacing(2)
    vibration_layout.setContentsMargins(6, 6, 6, 6)
    
    vibration_value_container = QHBoxLayout()
    app.vibration_state_label = QLabel("OFF")
    app.vibration_state_label.setFont(QFont("Monospace", 10))
    app.vibration_state_label.setStyleSheet(f"background-color: transparent; color: {Colors.STATUS_ERROR};")  # Red for OFF
    vibration_value_container.addWidget(app.vibration_state_label)
    vibration_value_container.addStretch()
    vibration_layout.addLayout(vibration_value_container)
    
    vibration_group.setLayout(vibration_layout)
    col1_layout.addWidget(vibration_group, 1)
    
    # Bandwidth Section
    bandwidth_group = QGroupBox("Bandwidth")
    bandwidth_group.setAutoFillBackground(True)
    bandwidth_group.setStyleSheet(Styles.subbox())
    bandwidth_layout = QVBoxLayout()
    bandwidth_layout.setSpacing(2)
    bandwidth_layout.setContentsMargins(6, 6, 6, 6)
    
    # All bandwidth values on one line
    values_container = QHBoxLayout()
    values_container.setSpacing(10)
    
    # Total
    total_label = QLabel("Total:")
    total_label.setFont(QFont("Monospace", 10))
    total_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(total_label)
    app.bandwidth_total_label = QLabel("0.00 Mbps")
    app.bandwidth_total_label.setFont(QFont("Monospace", 10))
    app.bandwidth_total_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(app.bandwidth_total_label)
    
    values_container.addSpacing(10)
    
    # RX
    rx_label = QLabel("RX:")
    rx_label.setFont(QFont("Monospace", 10))
    rx_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(rx_label)
    app.bandwidth_rx_label = QLabel("0.00 Mbps")
    app.bandwidth_rx_label.setFont(QFont("Monospace", 10))
    app.bandwidth_rx_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(app.bandwidth_rx_label)
    
    values_container.addSpacing(10)
    
    # TX
    tx_label = QLabel("TX:")
    tx_label.setFont(QFont("Monospace", 10))
    tx_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(tx_label)
    app.bandwidth_tx_label = QLabel("0.00 Mbps")
    app.bandwidth_tx_label.setFont(QFont("Monospace", 10))
    app.bandwidth_tx_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    values_container.addWidget(app.bandwidth_tx_label)
    
    values_container.addStretch()
    bandwidth_layout.addLayout(values_container)
    
    bandwidth_layout.addSpacing(4)
    
    # Progress bar for bandwidth
    app.bandwidth_progress = QProgressBar()
    app.bandwidth_progress.setMaximum(100)
    app.bandwidth_progress.setValue(0)
    app.bandwidth_progress.setTextVisible(False)
    app.bandwidth_progress.setMaximumHeight(10)
    bandwidth_layout.addWidget(app.bandwidth_progress)
    
    bandwidth_group.setLayout(bandwidth_layout)
    col1_layout.addWidget(bandwidth_group, 1)
    
    main_layout.addLayout(col1_layout, 1)
    
    # Column 2: Velocity and Position
    col2_layout = QVBoxLayout()
    col2_layout.setSpacing(7)
    
    # Velocity Section
    velocity_group = QGroupBox("Velocity")
    velocity_group.setAutoFillBackground(True)
    velocity_group.setStyleSheet(Styles.subbox())
    velocity_layout = QVBoxLayout()
    velocity_layout.setSpacing(2)
    velocity_layout.setContentsMargins(6, 6, 6, 6)
    
    vel_layout = QGridLayout()
    vel_layout.setSpacing(2)
    vel_layout.setHorizontalSpacing(3)
    vel_layout.setColumnStretch(1, 0)
    
    linear_label = QLabel("Linear:")
    linear_label.setFont(QFont("Monospace", 10))
    linear_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(linear_label, 0, 0)
    app.linear_vel_label = QLabel("0.00")
    app.linear_vel_label.setFont(QFont("Monospace", 10))
    app.linear_vel_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(app.linear_vel_label, 0, 1)
    
    linear_unit_label = QLabel("m/s")
    linear_unit_label.setFont(QFont("Monospace", 10))
    linear_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(linear_unit_label, 0, 2)
    
    angular_label = QLabel("Angular:")
    angular_label.setFont(QFont("Monospace", 10))
    angular_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(angular_label, 1, 0)
    app.angular_vel_label = QLabel("0.00")
    app.angular_vel_label.setFont(QFont("Monospace", 10))
    app.angular_vel_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    vel_layout.addWidget(app.angular_vel_label, 1, 1)
    
    angular_unit_label = QLabel("rad/s")
    angular_unit_label.setFont(QFont("Monospace", 10))
    angular_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
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
    position_group.setStyleSheet(Styles.subbox())
    position_layout = QVBoxLayout()
    position_layout.setSpacing(2)
    position_layout.setContentsMargins(6, 6, 6, 6)
    
    pos_layout = QGridLayout()
    pos_layout.setSpacing(2)
    pos_layout.setHorizontalSpacing(3)
    pos_layout.setColumnStretch(1, 0)
    
    x_label = QLabel("X:")
    x_label.setFont(QFont("Monospace", 10))
    x_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(x_label, 0, 0)
    app.position_x_label = QLabel("0.00")
    app.position_x_label.setFont(QFont("Monospace", 10))
    app.position_x_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(app.position_x_label, 0, 1)
    
    x_unit_label = QLabel("m")
    x_unit_label.setFont(QFont("Monospace", 10))
    x_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(x_unit_label, 0, 2)
    
    y_label = QLabel("Y:")
    y_label.setFont(QFont("Monospace", 10))
    y_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(y_label, 1, 0)
    app.position_y_label = QLabel("0.00")
    app.position_y_label.setFont(QFont("Monospace", 10))
    app.position_y_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(app.position_y_label, 1, 1)
    
    y_unit_label = QLabel("m")
    y_unit_label.setFont(QFont("Monospace", 10))
    y_unit_label.setStyleSheet(f"background-color: transparent; color: {Colors.TEXT_MAIN};")
    pos_layout.addWidget(y_unit_label, 1, 2)
    
    pos_container = QHBoxLayout()
    pos_container.addLayout(pos_layout)
    pos_container.addStretch()
    position_layout.addLayout(pos_container)
    
    position_group.setLayout(position_layout)
    col2_layout.addWidget(position_group, 1)
    
    main_layout.addLayout(col2_layout, 1)
    
    return group


def create_status_group(app):
    """Create robot status display"""
    group = QGroupBox("Robot Status")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background-color: {Colors.BG_BOX}; }}")
    layout = QVBoxLayout()
    layout.setContentsMargins(4, 2, 4, 2)
    
    app.status_label = QLabel("Active")
    app.status_label.setStyleSheet(f"color: {Colors.STATUS_SUCCESS}; font-weight: bold; background-color: transparent; font-size: 13px;")
    
    layout.addWidget(app.status_label)
    group.setLayout(layout)
    return group


def create_mode_group(app):
    """Create mode display and switcher"""
    group = QGroupBox("Robot Mode")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background-color: {Colors.BG_BOX}; }}")
    layout = QHBoxLayout()
    layout.setSpacing(3)
    layout.setContentsMargins(4, 2, 4, 2)
    
    # In sim mode, show "Simulation" in grey. In real mode, show Manual/AUTO with colors
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
            mode_text = "AUTO"
            mode_color = Colors.STATUS_ERROR
            button_text = "Switch to Manual"
    
    app.mode_label = QLabel(mode_text)
    app.mode_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
    app.mode_label.setStyleSheet(f"color: {mode_color}; font-weight: bold; font-size: 13px; background-color: transparent;")
    layout.addWidget(app.mode_label, 1)
    
    # Mode switch button
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
    """Create keyboard teleop control buttons"""
    group = QGroupBox("Keyboard Teleop")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background-color: {Colors.BG_BOX}; }}")
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
    
    # Button styles
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
    
    # Arrow buttons for bucket control
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
    
    # WASD Movement Controls
    wasd_label = QLabel("Movement")
    wasd_label.setStyleSheet(f"color: {Colors.TEXT_SECONDARY}; font-size: 11px; margin-top: 2px; background-color: transparent;")
    wasd_label.setAlignment(Qt.AlignCenter)
    layout.addWidget(wasd_label)
    
    # WASD button grid
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
    
    # Store button references
    app.teleop_buttons = {
        'w': app.btn_w, 'a': app.btn_a, 's': app.btn_s, 'd': app.btn_d,
        'up': app.btn_up, 'down': app.btn_down
    }
    app.teleop_button_style = button_style
    app.teleop_active_style = active_button_style
    
    group.setLayout(layout)
    return group


def create_launch_group(app):
    """Create system launch controls"""
    group = QGroupBox("Launch")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background-color: {Colors.BG_BOX}; }}")
    layout = QVBoxLayout()
    layout.setSpacing(4)  # Spacing between buttons
    layout.setContentsMargins(4, 4, 4, 4)
    
    button_style = f"""
        QPushButton {{
            background-color: {Colors.BTN_INACTIVE};
            color: white;
            font-size: 12px;
            font-weight: bold;
            padding: 4px;
            border: none;
            border-radius: 2px;
            text-align: center;
        }}
        QPushButton:hover {{
            background-color: {Colors.BTN_HOVER};
        }}
        QPushButton:pressed {{
            background-color: {Colors.BTN_PRESSED};
        }}
    """
    
    app.pointlio_btn = QPushButton("PointLIO")
    app.pointlio_btn.setStyleSheet(button_style)
    app.pointlio_btn.setMinimumHeight(24)
    app.pointlio_btn.setMaximumHeight(26)
    app.pointlio_btn.clicked.connect(lambda: app.launch_system('pointlio'))
    layout.addWidget(app.pointlio_btn)
    
    app.mapping_btn = QPushButton("Mapping")
    app.mapping_btn.setStyleSheet(button_style)
    app.mapping_btn.setMinimumHeight(24)
    app.mapping_btn.setMaximumHeight(26)
    app.mapping_btn.clicked.connect(lambda: app.launch_system('mapping'))
    layout.addWidget(app.mapping_btn)
    
    app.nav2_btn = QPushButton("Nav2")
    app.nav2_btn.setStyleSheet(button_style)
    app.nav2_btn.setMinimumHeight(24)
    app.nav2_btn.setMaximumHeight(26)
    app.nav2_btn.clicked.connect(lambda: app.launch_system('nav2'))
    layout.addWidget(app.nav2_btn)
    
    rviz_btn = QPushButton("RViz2")
    rviz_btn.setStyleSheet(Styles.light_blue_button(size=12))
    rviz_btn.setMinimumHeight(24)
    rviz_btn.setMaximumHeight(26)
    rviz_btn.clicked.connect(app.launch_rviz)
    layout.addWidget(rviz_btn)
    
    group.setLayout(layout)
    return group


def create_action_control_group(app):
    """Create action control buttons"""
    group = QGroupBox("Actions")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background-color: {Colors.BG_BOX}; }}")
    layout = QVBoxLayout()
    layout.setSpacing(4)  # Spacing between buttons
    layout.setContentsMargins(4, 4, 4, 4)
    
    app.home_btn = QPushButton("Home Actuators")
    app.home_btn.setStyleSheet(Styles.orange_button(size=12))
    app.home_btn.setMinimumHeight(24)
    app.home_btn.setMaximumHeight(26)
    app.home_btn.clicked.connect(app.send_home_goal)
    layout.addWidget(app.home_btn)
    
    app.localize_btn = QPushButton("Localize")
    app.localize_btn.setStyleSheet(Styles.standard_button(size=12))
    app.localize_btn.setMinimumHeight(24)
    app.localize_btn.setMaximumHeight(26)
    app.localize_btn.clicked.connect(lambda: app.launch_system('localization'))
    layout.addWidget(app.localize_btn)
    
    app.excavate_btn = QPushButton("Excavate")
    app.excavate_btn.setMinimumHeight(24)
    app.excavate_btn.setMaximumHeight(26)
    app.excavate_btn.setStyleSheet(f"""
        QPushButton {{
            background-color: {Colors.BTN_INACTIVE};
            color: white;
            font-size: 12px;
            font-weight: bold;
            padding: 4px;
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
    app.excavate_btn.clicked.connect(app.send_excavate_goal)
    layout.addWidget(app.excavate_btn)
    
    app.deposit_btn = QPushButton("Deposit")
    app.deposit_btn.setMinimumHeight(24)
    app.deposit_btn.setMaximumHeight(26)
    app.deposit_btn.setStyleSheet(f"""
        QPushButton {{
            background-color: {Colors.BTN_INACTIVE};
            color: white;
            font-size: 12px;
            font-weight: bold;
            padding: 4px;
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
    app.deposit_btn.clicked.connect(app.send_deposit_goal)
    layout.addWidget(app.deposit_btn)
    
    app.auto_btn = QPushButton("One Cycle Auto")
    app.auto_btn.setStyleSheet(Styles.blue_button(size=12))
    app.auto_btn.setMinimumHeight(24)
    app.auto_btn.setMaximumHeight(26)
    app.auto_btn.clicked.connect(app.send_full_auto_goal)
    layout.addWidget(app.auto_btn)
    
    app.emergency_stop_btn = QPushButton("Emergency Stop")
    app.emergency_stop_btn.setStyleSheet(Styles.red_button(size=12))
    app.emergency_stop_btn.setMinimumHeight(24)
    app.emergency_stop_btn.setMaximumHeight(26)
    app.emergency_stop_btn.clicked.connect(app.emergency_stop)
    app.emergency_stop_btn.setEnabled(app.robot.is_real_mode)
    layout.addWidget(app.emergency_stop_btn)
    
    app.operation_status_label = QLabel("Status: Idle")
    app.operation_status_label.setWordWrap(True)
    app.operation_status_label.setFont(QFont("Monospace", 10))
    app.operation_status_label.setStyleSheet(f"color: {Colors.TEXT_MAIN}; font-weight: bold; background-color: transparent; padding: 3px 0px;")
    app.operation_status_label.setMinimumHeight(20)
    app.operation_status_label.setMaximumHeight(60)  # Limit status label height
    layout.addWidget(app.operation_status_label)
    
    group.setLayout(layout)
    return group


def create_camera_control_group(app):
    """Create fisheye camera rotation controls"""
    group = QGroupBox("Fisheye Camera Rotation")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"""
        QGroupBox {{ 
            background-color: {Colors.BG_BOX}; 
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
    
    rotate_left_btn = QPushButton("← 45°")
    rotate_left_btn.setStyleSheet(Styles.camera_button())
    rotate_left_btn.clicked.connect(lambda: app.rotate_fisheye_camera(0.785398))
    button_row.addWidget(rotate_left_btn)
    
    center_column = QVBoxLayout()
    center_column.setSpacing(8)
    
    center_btn = QPushButton("Center")
    center_btn.setStyleSheet(Styles.camera_center_button())
    center_btn.clicked.connect(lambda: app.set_fisheye_camera_position(0.0))
    center_column.addWidget(center_btn)
    
    btn_180 = QPushButton("180°")
    btn_180.setStyleSheet(Styles.camera_center_button())
    btn_180.clicked.connect(lambda: app.set_fisheye_camera_position(3.14159))
    center_column.addWidget(btn_180)
    
    button_row.addLayout(center_column)
    
    rotate_right_btn = QPushButton("45° →")
    rotate_right_btn.setStyleSheet(Styles.camera_button())
    rotate_right_btn.clicked.connect(lambda: app.rotate_fisheye_camera(-0.785398))
    button_row.addWidget(rotate_right_btn)
    
    layout.addLayout(button_row)
    
    group.setLayout(layout)
    return group


def create_hardware_group(app):
    """Create hardware control buttons"""
    group = QGroupBox("Hardware")
    group.setAutoFillBackground(True)
    group.setStyleSheet(f"QGroupBox {{ background-color: {Colors.BG_BOX}; }}")
    layout = QVBoxLayout()
    layout.setSpacing(4)  # Spacing between buttons
    layout.setContentsMargins(4, 4, 4, 4)
    
    can_btn = QPushButton("Start CAN Interface")
    can_btn.setStyleSheet(Styles.standard_button(size=12))
    can_btn.setMinimumHeight(32)
    can_btn.clicked.connect(app.start_can_interface)
    layout.addWidget(can_btn)
    
    app.hardware_btn = QPushButton("Launch Hardware")
    app.hardware_btn.setStyleSheet(Styles.standard_button(size=12))
    app.hardware_btn.setMinimumHeight(32)
    app.hardware_btn.clicked.connect(app.launch_hardware)
    layout.addWidget(app.hardware_btn)
    
    group.setLayout(layout)
    return group

