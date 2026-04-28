#!/usr/bin/env python3
from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QColor, QFont, QPainter
from PyQt5.QtWidgets import (QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                              QProgressBar, QPushButton, QSizePolicy,
                              QVBoxLayout, QWidget)

from gui_styles import Colors, Styles, ACTION_BTN_CSS, ESTOP_BTN_NORMAL_CSS

_FONT_SM = 9
_FONT_MD = 10

_GROUP_BG_DARK  = "QGroupBox { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1d1d1d); padding-top: 16px; }"
_GROUP_BG_DARK2 = "QGroupBox { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #202020, stop:1 #1a1a1a); padding-top: 24px; }"


def _mono_label(text, color=None):
    label = QLabel(text)
    label.setFont(QFont("Monospace", _FONT_SM))
    label.setStyleSheet(f"background-color: transparent; color: {color or Colors.TEXT_MAIN};")
    return label


def _action_btn(text, slot, max_height=32):
    btn = QPushButton(text)
    btn.setStyleSheet(ACTION_BTN_CSS)
    btn.setMaximumHeight(max_height)
    btn.clicked.connect(slot)
    return btn


def create_condensed_telemetry_group(app):
    group = QGroupBox("Telemetry")
    group.setAutoFillBackground(True)
    group.setStyleSheet(_GROUP_BG_DARK)
    main_layout = QHBoxLayout()
    main_layout.setContentsMargins(3, 6, 3, 2)
    main_layout.setSpacing(8)
    group.setLayout(main_layout)

    # --- Power ---
    power_group = QGroupBox("Power")
    power_group.setAutoFillBackground(True)
    power_group.setStyleSheet(Styles.subbox())
    power_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    power_layout = QVBoxLayout()
    power_layout.setSpacing(1)
    power_layout.setContentsMargins(3, 4, 3, 2)

    g = QGridLayout()
    g.setSpacing(1)
    g.setHorizontalSpacing(2)
    g.setVerticalSpacing(1)

    left_rows = [("V:", "power_voltage_label", "V"),
                 ("I:", "power_current_label",  "A"),
                 ("T:", "power_temp_label",      "°C")]
    for row, (lbl, attr, unit) in enumerate(left_rows):
        g.addWidget(_mono_label(lbl), row, 0)
        val = _mono_label("0.00")
        setattr(app, attr, val)
        g.addWidget(val, row, 1)
        g.addWidget(_mono_label(unit), row, 2)

    g.setColumnMinimumWidth(3, 8)

    right_rows = [("P:", "power_watts_label",  "W"),
                  ("E:", "power_energy_label", "Wh")]
    for row, (lbl, attr, unit) in enumerate(right_rows):
        g.addWidget(_mono_label(lbl), row, 4)
        val = _mono_label("0.00")
        setattr(app, attr, val)
        g.addWidget(val, row, 5)
        g.addWidget(_mono_label(unit), row, 6)

    row_wrap = QHBoxLayout()
    row_wrap.addLayout(g)
    row_wrap.addStretch()
    power_layout.addLayout(row_wrap)
    power_group.setLayout(power_layout)
    main_layout.addWidget(power_group, 0)

    # --- Velocity ---
    vel_group = QGroupBox("Velocity")
    vel_group.setAutoFillBackground(True)
    vel_group.setStyleSheet(Styles.subbox())
    vel_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    vel_layout = QVBoxLayout()
    vel_layout.setSpacing(1)
    vel_layout.setContentsMargins(3, 4, 3, 2)

    vg = QGridLayout()
    vg.setSpacing(1)
    vg.setHorizontalSpacing(2)
    vg.setVerticalSpacing(1)

    vel_rows = [("Lin:", "linear_vel_label",  "m/s"),
                ("Ang:", "angular_vel_label", "rad/s")]
    for row, (lbl, attr, unit) in enumerate(vel_rows):
        vg.addWidget(_mono_label(lbl), row, 0)
        val = _mono_label("0.00")
        setattr(app, attr, val)
        vg.addWidget(val, row, 1)
        vg.addWidget(_mono_label(unit), row, 2)

    vel_wrap = QHBoxLayout()
    vel_wrap.addLayout(vg)
    vel_wrap.addStretch()
    vel_layout.addLayout(vel_wrap)
    vel_layout.addStretch()
    vel_group.setLayout(vel_layout)
    main_layout.addWidget(vel_group, 0)

    # --- Pose ---
    pose_group = QGroupBox("Pose")
    pose_group.setAutoFillBackground(True)
    pose_group.setStyleSheet(Styles.subbox())
    pose_group.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Preferred)
    pose_layout = QVBoxLayout()
    pose_layout.setSpacing(1)
    pose_layout.setContentsMargins(3, 4, 3, 2)

    pg = QGridLayout()
    pg.setSpacing(1)
    pg.setHorizontalSpacing(2)
    pg.setVerticalSpacing(1)

    pos_rows = [("X:", "position_x_label", "m"),
                ("Y:", "position_y_label", "m"),
                ("Z:", "position_z_label", "m")]
    for row, (lbl, attr, unit) in enumerate(pos_rows):
        pg.addWidget(_mono_label(lbl), row, 0)
        val = _mono_label("0.00")
        setattr(app, attr, val)
        pg.addWidget(val, row, 1)
        pg.addWidget(_mono_label(unit), row, 2)

    pg.setColumnMinimumWidth(3, 8)

    ori_rows = [("R:", "orientation_roll_label",  "rad"),
                ("P:", "orientation_pitch_label", "rad"),
                ("Y:", "orientation_yaw_label",   "rad")]
    for row, (lbl, attr, unit) in enumerate(ori_rows):
        pg.addWidget(_mono_label(lbl), row, 4)
        val = _mono_label("0.00")
        setattr(app, attr, val)
        pg.addWidget(val, row, 5)
        pg.addWidget(_mono_label(unit), row, 6)

    pose_wrap = QHBoxLayout()
    pose_wrap.addLayout(pg)
    pose_wrap.addStretch()
    pose_layout.addLayout(pose_wrap)
    pose_layout.addStretch()
    pose_group.setLayout(pose_layout)
    main_layout.addWidget(pose_group, 0)

    # --- Vibration ---
    vib_group = QGroupBox("Vibration State")
    vib_group.setAutoFillBackground(True)
    vib_group.setStyleSheet(Styles.subbox())
    vib_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    vib_layout = QVBoxLayout()
    vib_layout.setSpacing(1)
    vib_layout.setContentsMargins(3, 4, 3, 2)
    vib_row = QHBoxLayout()
    app.vibration_state_label = _mono_label("OFF", Colors.STATUS_ERROR)
    vib_row.addWidget(app.vibration_state_label)
    vib_row.addStretch()
    vib_layout.addLayout(vib_row)
    vib_layout.addStretch()
    vib_group.setLayout(vib_layout)
    main_layout.addWidget(vib_group, 0)

    # --- Bucket Angle ---
    bucket_group = QGroupBox("Bucket Angle")
    bucket_group.setAutoFillBackground(True)
    bucket_group.setStyleSheet(Styles.subbox())
    bucket_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    bucket_layout = QVBoxLayout()
    bucket_layout.setSpacing(1)
    bucket_layout.setContentsMargins(3, 4, 3, 2)
    bucket_row = QHBoxLayout()
    app.bucket_angle_label = _mono_label("0.00 rad")
    bucket_row.addWidget(app.bucket_angle_label)
    bucket_row.addStretch()
    bucket_layout.addLayout(bucket_row)
    bucket_layout.addStretch()
    bucket_group.setLayout(bucket_layout)
    main_layout.addWidget(bucket_group, 0)

    main_layout.addStretch()
    return group


def create_bandwidth_group(app):
    group = QGroupBox("Bandwidth")
    group.setAutoFillBackground(True)
    group.setStyleSheet(_GROUP_BG_DARK)
    group.setMaximumHeight(100)
    layout = QVBoxLayout()
    layout.setContentsMargins(4, 6, 4, 2)
    layout.setSpacing(1)
    group.setLayout(layout)

    vals_layout = QHBoxLayout()
    vals_layout.setSpacing(6)

    pairs = [
        ("Avg:",   'bandwidth_total_label',        "0.00 Mbps"),
        ("Total:", 'bandwidth_total_current_label', "0.00 Mbps"),
        ("RX:",    'bandwidth_rx_current_label',    "0.00 Mbps"),
        ("TX:",    'bandwidth_tx_current_label',    "0.00 Mbps"),
    ]
    for i, (lbl_text, attr, init_val) in enumerate(pairs):
        if i > 0:
            vals_layout.addSpacing(8)
        vals_layout.addWidget(_mono_label(lbl_text))
        val = _mono_label(init_val)
        setattr(app, attr, val)
        vals_layout.addWidget(val)

    vals_layout.addStretch()
    layout.addLayout(vals_layout)
    layout.addSpacing(2)

    app.bandwidth_progress = QProgressBar()
    app.bandwidth_progress.setMaximum(100)
    app.bandwidth_progress.setValue(0)
    app.bandwidth_progress.setTextVisible(False)
    app.bandwidth_progress.setMaximumHeight(8)
    layout.addWidget(app.bandwidth_progress)

    return group


def create_bucket_state_widget(app):
    group = QGroupBox("Bucket State")
    group.setAutoFillBackground(True)
    group.setStyleSheet(_GROUP_BG_DARK)
    layout = QVBoxLayout()
    layout.setSpacing(6)
    layout.setContentsMargins(3, 18, 3, 2)
    group.setLayout(layout)

    app.bucket_slider = BucketSliderWidget()
    app.bucket_slider.setMaximumHeight(8)
    layout.addWidget(app.bucket_slider)

    deposit_w, travel_w, excavate_w = BucketSliderWidget.get_zone_proportions()
    zone_labels = QHBoxLayout()
    zone_labels.setSpacing(0)
    zone_labels.setContentsMargins(0, 4, 0, 0)

    label_style = f"color: {Colors.TEXT_MAIN}; font-size: 12px; font-family: Monospace; background: transparent;"
    for text, weight in [("Deposit", deposit_w), ("Travel", travel_w), ("Excavate", excavate_w)]:
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(label_style)
        zone_labels.addWidget(lbl, weight)

    layout.addLayout(zone_labels)
    return group


def create_mode_group(app):
    group = QGroupBox("Robot Mode")
    group.setAutoFillBackground(True)
    group.setStyleSheet("QGroupBox { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1d1d1d); }")
    group.setMaximumHeight(100)
    group.setMinimumWidth(220)
    layout = QHBoxLayout()
    layout.setSpacing(3)
    layout.setContentsMargins(4, 3, 4, 3)
    group.setLayout(layout)

    if not app.robot.is_real_mode:
        mode_text, mode_color, btn_text = "Simulation", Colors.TEXT_DARKER, "Switch to Auto"
    else:
        is_manual = (app.robot.robot_mode == "MANUAL")
        mode_text  = "Manual" if is_manual else "Auto"
        mode_color = Colors.STATUS_SUCCESS if is_manual else Colors.STATUS_ERROR
        btn_text   = "Switch to Auto" if is_manual else "Switch to Manual"

    app.mode_label = QLabel(mode_text)
    app.mode_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
    app.mode_label.setStyleSheet(
        f"color: {mode_color}; font-weight: bold; font-size: 13px; background-color: transparent;")
    layout.addWidget(app.mode_label, 1)

    mode_btn = QPushButton(btn_text)
    mode_btn.setStyleSheet(f"""
        QPushButton {{
            background-color: {Colors.BTN_INACTIVE};
            color: white;
            font-size: 11px;
            font-weight: bold;
            padding: 4px 10px;
            border: none;
            border-radius: 2px;
        }}
        QPushButton:hover {{ background-color: {Colors.BTN_HOVER}; }}
        QPushButton:pressed {{ background-color: {Colors.BTN_PRESSED}; }}
        QPushButton:disabled {{
            background-color: {Colors.BTN_DISABLED};
            color: {Colors.TEXT_GRAY};
            border: none;
            border-radius: 2px;
        }}
    """)
    mode_btn.clicked.connect(app.toggle_mode)
    layout.addWidget(mode_btn)
    app.mode_switch_btn = mode_btn

    return group


def create_teleop_control_group(app):
    group = QGroupBox("Keyboard Teleop")
    group.setAutoFillBackground(True)
    group.setStyleSheet("QGroupBox { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #202020, stop:1 #1a1a1a); }")
    layout = QVBoxLayout()
    layout.setSpacing(4)
    layout.setContentsMargins(4, 4, 4, 4)
    group.setLayout(layout)

    btn_css = f"""
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
        QPushButton:hover {{ background-color: {Colors.BTN_HOVER}; }}
        QPushButton:pressed {{ background-color: {Colors.BTN_PRESSED}; }}
    """
    btn_active_css = f"""
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

    # Top row: bucket controls (left) + speed info (right)
    top_row = QHBoxLayout()

    bucket_section = QVBoxLayout()
    bucket_section.setSpacing(3)
    bucket_lbl = QLabel("Bucket")
    bucket_lbl.setStyleSheet(f"color: {Colors.TEXT_SECONDARY}; font-size: 11px; background-color: transparent;")
    bucket_lbl.setAlignment(Qt.AlignCenter)
    bucket_section.addWidget(bucket_lbl)

    arrow_row = QHBoxLayout()
    arrow_row.setSpacing(4)
    app.btn_up   = QPushButton("↑")
    app.btn_down = QPushButton("↓")
    for btn in [app.btn_up, app.btn_down]:
        btn.setStyleSheet(btn_css)
        btn.setFocusPolicy(Qt.NoFocus)
    app.btn_up.pressed.connect(lambda: app.teleop_key_press('up'))
    app.btn_up.released.connect(lambda: app.teleop_key_release('up'))
    app.btn_down.pressed.connect(lambda: app.teleop_key_press('down'))
    app.btn_down.released.connect(lambda: app.teleop_key_release('down'))
    arrow_row.addWidget(app.btn_up)
    arrow_row.addWidget(app.btn_down)
    bucket_section.addLayout(arrow_row)
    top_row.addLayout(bucket_section)
    top_row.addStretch()

    speed_html = (
        f'<div style="text-align: right; line-height: 1.2;">'
        f'<span style="color: {Colors.TEXT_DIM}; font-size: 13px;">'
        f'Lin: {app.linear_speed:.2f} m/s | Ang: {app.angular_speed:.2f} rad/s</span>'
        f'<br><span style="color: {Colors.TEXT_DARKER}; font-size: 11px; font-style: italic;">'
        f'Q/Z: Linear Speed | E/C: Angular Speed</span></div>'
    )
    app.speed_label = QLabel(speed_html)
    app.speed_label.setStyleSheet("background-color: transparent; padding: 0px; margin: 0px;")
    app.speed_label.setAlignment(Qt.AlignRight)
    top_row.addWidget(app.speed_label)
    layout.addLayout(top_row)

    # WASD movement grid
    move_lbl = QLabel("Movement")
    move_lbl.setStyleSheet(f"color: {Colors.TEXT_SECONDARY}; font-size: 11px; margin-top: 2px; background-color: transparent;")
    move_lbl.setAlignment(Qt.AlignCenter)
    layout.addWidget(move_lbl)

    wasd_grid = QGridLayout()
    wasd_grid.setSpacing(12)
    app.btn_w = QPushButton("W")
    app.btn_a = QPushButton("A")
    app.btn_s = QPushButton("S")
    app.btn_d = QPushButton("D")
    for btn in [app.btn_w, app.btn_a, app.btn_s, app.btn_d]:
        btn.setStyleSheet(btn_css)
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
        'up': app.btn_up, 'down': app.btn_down,
    }
    app.teleop_button_style = btn_css
    app.teleop_active_style = btn_active_css

    return group


def create_launch_group(app):
    group = QGroupBox("Launch")
    group.setAutoFillBackground(True)
    group.setStyleSheet(_GROUP_BG_DARK2)
    layout = QVBoxLayout()
    layout.setSpacing(6)
    layout.setContentsMargins(4, 8, 4, 4)
    group.setLayout(layout)

    launch_btns = [
        ('pointlio_btn', 'Point-LIO',   lambda: app.launch_system('pointlio')),
        ('mapping_btn',  'RTAB-Map',    lambda: app.launch_system('mapping')),
        ('nav2_btn',     'Navigation2', lambda: app.launch_system('nav2')),
    ]
    for attr, text, slot in launch_btns:
        btn = _action_btn(text, slot)
        setattr(app, attr, btn)
        layout.addWidget(btn)

    layout.addWidget(_action_btn("RViz2", app.launch_rviz))
    return group


def create_action_control_group(app):
    group = QGroupBox("Actions")
    group.setAutoFillBackground(True)
    group.setStyleSheet(_GROUP_BG_DARK2)
    layout = QVBoxLayout()
    layout.setSpacing(6)
    layout.setContentsMargins(4, 8, 4, 4)
    group.setLayout(layout)

    action_btns = [
        ('excavate_btn', 'Excavate',       app.send_excavate_goal),
        ('deposit_btn',  'Deposit',         app.send_deposit_goal),
        ('auto_btn',     'One Cycle Auto',  app.send_full_auto_goal),
    ]
    for attr, text, slot in action_btns:
        btn = _action_btn(text, slot)
        setattr(app, attr, btn)
        layout.addWidget(btn)

    app.emergency_stop_btn = QPushButton("Emergency Stop")
    app.emergency_stop_btn.setStyleSheet(ESTOP_BTN_NORMAL_CSS)
    app.emergency_stop_btn.setMaximumHeight(50)
    app.emergency_stop_btn.clicked.connect(app.emergency_stop)
    app.emergency_stop_btn.setEnabled(app.robot.is_real_mode)
    layout.addWidget(app.emergency_stop_btn)

    app.operation_status_label = QLabel("Status: Idle")
    app.operation_status_label.setWordWrap(True)
    app.operation_status_label.setFont(QFont("Monospace", _FONT_MD))
    app.operation_status_label.setStyleSheet(
        f"color: {Colors.TEXT_MAIN}; font-weight: bold; background-color: transparent; padding: 3px 0px;")
    app.operation_status_label.setMinimumHeight(20)
    app.operation_status_label.setMaximumHeight(60)
    layout.addWidget(app.operation_status_label)

    return group


class BucketSliderWidget(QWidget):
    _MAX_POS = 0.0
    _MIN_POS = 1.5708
    _MID1    = 0.5236
    _MID2    = 1.309

    @classmethod
    def get_zone_proportions(cls):
        total = cls._MIN_POS - cls._MAX_POS
        deposit_w  = int(((cls._MID1 - cls._MAX_POS) / total) * 1000)
        travel_w   = int(((cls._MID2 - cls._MID1)    / total) * 1000)
        excavate_w = int(((cls._MIN_POS - cls._MID2) / total) * 1000)
        return deposit_w, travel_w, excavate_w

    def __init__(self, parent=None):
        super().__init__(parent)
        self._position = 0.7854
        self.setMinimumSize(100, 8)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_position(self, pos):
        self._position = max(self._MAX_POS, min(self._MIN_POS, pos))
        self.update()

    def paintEvent(self, event):
        from PyQt5.QtGui import QPen, QLinearGradient, QPainterPath
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h   = self.width(), self.height()
        total  = self._MIN_POS - self._MAX_POS
        r      = 3

        def _x(pos):
            return int((pos - self._MAX_POS) / total * w)

        x1, x2 = _x(self._MID1), _x(self._MID2)

        bg_grad = QLinearGradient(0, 0, 0, h)
        bg_grad.setColorAt(0, QColor("#0d0d0d"))
        bg_grad.setColorAt(1, QColor("#0a0a0a"))

        bar_rect = QRectF(0, 0, w, h)
        clip = QPainterPath()
        clip.addRoundedRect(bar_rect, r, r)
        painter.setClipPath(clip)
        painter.fillRect(bar_rect, bg_grad)

        def draw_zone(x_start, x_end, top, bottom):
            grad = QLinearGradient(0, 0, 0, h)
            grad.setColorAt(0, QColor(top))
            grad.setColorAt(1, QColor(bottom))
            painter.fillRect(QRectF(x_start, 0, x_end - x_start, h), grad)

        draw_zone(0,  x1, "#ffa726", "#f57c00")
        draw_zone(x1, x2, "#66bb6a", "#43a047")
        draw_zone(x2, w,  "#ef5350", "#d32f2f")

        painter.setClipping(False)

        painter.setPen(QPen(QColor("#3e3e3e"), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(QRectF(0.5, 0.5, w - 1, h - 1), r, r)

        ix    = _x(self._position)
        ind_w = 4
        ind_x = max(ind_w // 2, min(w - ind_w // 2, ix))
        painter.setPen(QPen(QColor("#000000"), ind_w + 2))
        painter.drawLine(ind_x, -3, ind_x, h + 3)
        painter.setPen(QPen(QColor("#ffffff"), ind_w))
        painter.drawLine(ind_x, -3, ind_x, h + 3)


def create_controls_reference_group(app):
    group = QGroupBox("Controller Reference")
    group.setAutoFillBackground(True)
    group.setStyleSheet(_GROUP_BG_DARK2)
    layout = QVBoxLayout()
    layout.setSpacing(5)
    layout.setContentsMargins(4, 8, 4, 4)
    group.setLayout(layout)

    def add_header(text):
        lbl = QLabel(text)
        lbl.setStyleSheet(
            "color: #ffa726; font-size: 12px; font-weight: bold; font-family: Monospace; padding-top: 3px; background: transparent;")
        layout.addWidget(lbl)

    def add_control(control, description):
        row = QHBoxLayout()
        row.setSpacing(10)
        row.setContentsMargins(0, 0, 0, 0)
        ctrl_lbl = QLabel(control)
        ctrl_lbl.setStyleSheet(
            "color: #ffcc80; font-size: 11px; font-weight: bold; font-family: Monospace; background: transparent;")
        ctrl_lbl.setFixedWidth(85)
        ctrl_lbl.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
        row.addWidget(ctrl_lbl)
        desc_lbl = QLabel(description)
        desc_lbl.setStyleSheet(
            f"color: {Colors.TEXT_MAIN}; font-size: 11px; font-family: Monospace; background: transparent;")
        desc_lbl.setWordWrap(True)
        desc_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        row.addWidget(desc_lbl, 1)
        layout.addLayout(row)

    sections = [
        ("MODE", [
            ("Plus (+)",   "Enable Manual"),
            ("Minus (-)",  "Enable Auto"),
            ("Home",       "Enable/Disable Robot"),
        ]),
        ("DRIVING", [
            ("Left Paddle",  "Toggle Speed"),
            ("Right Paddle", "Vibration"),
        ]),
        ("AUTONOMOUS ACTIONS", [
            ("X Button", "Excavate"),
            ("Y Button", "Deposit"),
        ]),
        ("BUCKET PRESET", [
            ("D-Pad Up",   "Deposit Ready"),
            ("D-Pad Down", "Excavate Ready"),
        ]),
        ("CAMERA CONTROL", [
            ("L4 Button", "Bucket View (0°)"),
            ("R4 Button", "Deposit View (180°)"),
        ]),
    ]
    for i, (header, controls) in enumerate(sections):
        if i > 0:
            layout.addSpacing(6)
        add_header(header)
        for ctrl, desc in controls:
            add_control(ctrl, desc)

    return group


def create_hardware_group(app):
    group = QGroupBox("Hardware")
    group.setAutoFillBackground(True)
    group.setStyleSheet(_GROUP_BG_DARK2)
    layout = QVBoxLayout()
    layout.setSpacing(6)
    layout.setContentsMargins(4, 8, 4, 4)
    group.setLayout(layout)

    hw_btns = [
        ('can_btn',         'Start CAN',       app.start_can_interface),
        ('can_restart_btn', 'Restart CAN',     app.restart_can),
        ('hardware_btn',    'Launch Hardware', app.launch_hardware),
    ]
    for attr, text, slot in hw_btns:
        btn = _action_btn(text, slot)
        setattr(app, attr, btn)
        layout.addWidget(btn)

    return group
