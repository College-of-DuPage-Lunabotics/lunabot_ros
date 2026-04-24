# Color Palette
class Colors:
    """Color constants for GUI theme"""
    # Background colors
    BG_MAIN = "#1a1a1a"
    BG_BOX = "#252525"
    BG_TRANSPARENT = "transparent"
    BG_DISABLED = "#1f1f1f"

    # Border and accent colors
    BORDER = "#505050"
    BORDER_DARK = "#404040"
    BORDER_DISABLED = "#333333"

    # Text colors
    TEXT_MAIN = "#e0e0e0"
    TEXT_SECONDARY = "#bbb"
    TEXT_DIM = "#aaa"
    TEXT_DARKER = "#888888"
    TEXT_GRAY = "#666"
    TEXT_DISABLED = "#555555"
    LABEL_SECTION = TEXT_DARKER

    # Status colors
    STATUS_ERROR = "#d32f2f"
    STATUS_SUCCESS = "#66bb6a"
    STATUS_WARNING = "#ffa726"
    STATUS_INFO = "#2196f3"

    # Button colors
    BTN_INACTIVE = "#424242"
    BTN_HOVER = "#616161"
    BTN_PRESSED = "#757575"
    BTN_ACTIVE = "#4a7ba7"
    BTN_DISABLED = "#2a2a2a"
    BTN_DISABLED_BORDER = "#404040"
    
    # Special button colors
    BTN_ORANGE = "#f57c00"
    BTN_ORANGE_HOVER = "#e65100"
    BTN_ORANGE_PRESSED = "#d84315"
    BTN_BLUE = "#1976d2"
    BTN_BLUE_HOVER = "#1565c0"
    BTN_BLUE_PRESSED = "#0d47a1"
    BTN_RED = "#d32f2f"
    BTN_RED_HOVER = "#b71c1c"
    BTN_RED_PRESSED = "#9a0007"
    BTN_LIGHT_BLUE = "#3a8fbf"
    BTN_LIGHT_BLUE_HOVER = "#5aa3cc"
    BTN_LIGHT_BLUE_PRESSED = "#2a7fa8"

    # Qt QPalette colors (RGB tuples for QColor)
    PALETTE_WINDOW = (53, 53, 53)
    PALETTE_BASE = (35, 35, 35)
    PALETTE_TOOLTIP_BASE = (25, 25, 25)
    PALETTE_LINK = (42, 130, 218)
    PALETTE_HIGHLIGHT = (42, 130, 218)


# Global Application Stylesheet
MAIN_STYLESHEET = f"""
    QMainWindow {{
        background-color: {Colors.BG_MAIN};
    }}
    QWidget#centralWidget {{
        background-color: {Colors.BG_MAIN};
    }}
    QGroupBox {{
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2a2a2a, stop:1 #252525);
        border: none;
        border-top: 2px solid {Colors.LABEL_SECTION};
        margin: 0px;
        font-weight: bold;
        padding-top: 18px;
        padding-left: 4px;
        padding-right: 4px;
        padding-bottom: 4px;
        color: {Colors.TEXT_MAIN};
    }}
    QGroupBox:disabled {{
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #242424, stop:1 #1f1f1f);
        border: none;
        border-top: 2px solid {Colors.LABEL_SECTION};
        color: {Colors.TEXT_DISABLED};
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        subcontrol-position: top left;
        padding: 0 5px;
        top: 4px;
        left: 2px;
        color: {Colors.TEXT_MAIN};
    }}
    QGroupBox:disabled::title {{
        color: {Colors.TEXT_DISABLED};
    }}
    QLabel {{
        background-color: {Colors.BG_TRANSPARENT};
        color: {Colors.TEXT_MAIN};
    }}
    QLabel:disabled {{
        color: {Colors.TEXT_DISABLED};
    }}
    QPushButton {{
        font-weight: bold;
    }}
    QPushButton:disabled {{
        background-color: {Colors.BG_DISABLED};
        color: {Colors.TEXT_DISABLED};
        border-color: {Colors.BORDER_DISABLED};
    }}
    QProgressBar {{
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #1e1e1e, stop:1 #1a1a1a);
        border: 1px solid #3e3e3e;
        border-radius: 3px;
        text-align: center;
        color: {Colors.TEXT_MAIN};
        font-weight: bold;
        height: 20px;
    }}
    QProgressBar::chunk {{
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #42a5f5, stop:1 #1976d2);
        border-radius: 2px;
    }}
"""


# Common Widget Styles
class Styles:
    """Common stylesheet patterns"""

    _DISABLED_BTN_CSS = f"""
        QPushButton:disabled {{
            background-color: {Colors.BG_DISABLED};
            color: {Colors.TEXT_DISABLED};
            border: none;
            border-radius: 2px;
        }}
    """

    @staticmethod
    def camera_label():
        return f"background-color: {Colors.BG_MAIN}; color: {Colors.TEXT_DARKER}; border: none;"
    
    @staticmethod
    def status_label(status_type='idle'):
        """Get status label style based on type"""
        color_map = {
            'idle': Colors.TEXT_DIM,
            'active': Colors.STATUS_SUCCESS,
            'warning': Colors.STATUS_WARNING,
            'error': Colors.STATUS_ERROR,
            'info': Colors.STATUS_INFO,
        }
        color = color_map.get(status_type, Colors.TEXT_DIM)
        return f"color: {color}; background-color: {Colors.BG_TRANSPARENT};"
    
    @staticmethod
    def robot_status(is_active=True):
        """Get robot status label style"""
        if is_active:
            return f"color: {Colors.STATUS_SUCCESS}; font-weight: bold;"
        return f"color: {Colors.STATUS_ERROR}; font-weight: bold;"
    
    @staticmethod
    def mode_label(mode='manual'):
        """Get mode label style"""
        if mode.upper() == 'MANUAL':
            return f"color: {Colors.STATUS_SUCCESS}; padding: 10px;"
        return f"color: {Colors.STATUS_INFO}; padding: 10px;"
    
    @staticmethod
    def _make_button(bg, hover, pressed, size, padding="4px"):
        """Build a gradient button stylesheet"""
        # Create slightly darker version for bottom of gradient
        def darken(hex_color, amount=0.1):
            hex_color = hex_color.lstrip('#')
            r, g, b = int(hex_color[0:2], 16), int(hex_color[2:4], 16), int(hex_color[4:6], 16)
            r = max(0, int(r * (1 - amount)))
            g = max(0, int(g * (1 - amount)))
            b = max(0, int(b * (1 - amount)))
            return f"#{r:02x}{g:02x}{b:02x}"
        
        bg_dark = darken(bg, 0.15)
        hover_dark = darken(hover, 0.15)
        pressed_dark = darken(pressed, 0.15)
        
        return f"""
            QPushButton {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 {bg}, stop:1 {bg_dark});
                color: white;
                font-size: {size}px;
                font-weight: bold;
                padding: {padding};
                border: 1px solid {darken(bg, 0.3)};
                border-radius: 4px;
            }}
            QPushButton:hover {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 {hover}, stop:1 {hover_dark});
                border: 1px solid {darken(hover, 0.3)};
            }}
            QPushButton:pressed {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 {pressed}, stop:1 {pressed_dark});
                border: 1px solid {darken(pressed, 0.3)};
            }}
            {Styles._DISABLED_BTN_CSS}
        """

    @staticmethod
    def standard_button(size=12):
        """Standard button style"""
        return Styles._make_button(Colors.BTN_INACTIVE, Colors.BTN_HOVER, Colors.BTN_PRESSED, size)

    @staticmethod
    def orange_button(size=12):
        """Orange action button style (e.g., Home)"""
        return Styles._make_button(Colors.BTN_ORANGE, Colors.BTN_ORANGE_HOVER, Colors.BTN_ORANGE_PRESSED, size)

    @staticmethod
    def blue_button(size=12):
        """Blue action button style (e.g., Auto)"""
        return Styles._make_button(Colors.BTN_BLUE, Colors.BTN_BLUE_HOVER, Colors.BTN_BLUE_PRESSED, size)

    @staticmethod
    def red_button(size=12):
        """Red action button style (e.g., Emergency Stop)"""
        return Styles._make_button(Colors.BTN_RED, Colors.BTN_RED_HOVER, Colors.BTN_RED_PRESSED, size)

    @staticmethod
    def light_blue_button(size=11):
        """Light blue button style (e.g., RViz2)"""
        return Styles._make_button(Colors.BTN_LIGHT_BLUE, Colors.BTN_LIGHT_BLUE_HOVER, Colors.BTN_LIGHT_BLUE_PRESSED, size, padding="6px")
    
    @staticmethod
    def emergency_button():
        """Emergency stop button style (large format)"""
        return f"""
            QPushButton {{
                background-color: {Colors.BTN_RED};
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 20px;
                border: none;
                border-radius: 2px;
            }}
            QPushButton:hover {{
                background-color: {Colors.BTN_RED_HOVER};
            }}
            QPushButton:pressed {{
                background-color: {Colors.BTN_RED_PRESSED};
            }}
        """
    
    @staticmethod
    def teleop_button(size=14):
        """Teleop control button style with gradient"""
        return f"""
            QPushButton {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2e2e2e, stop:1 #262626);
                color: white;
                font-size: {size}px;
                font-weight: bold;
                padding: 10px 20px;
                border: 1px solid #3e3e3e;
                border-radius: 4px;
                min-width: 50px;
                min-height: 50px;
            }}
            QPushButton:hover {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #383838, stop:1 #303030);
                border: 1px solid #484848;
            }}
            QPushButton:pressed {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1a1a1a);
                border: 1px solid #2e2e2e;
            }}
            QPushButton:checked {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #5a8fbf, stop:1 #4a7ba7);
                border: 1px solid #3e6e97;
            }}
        """
    
    @staticmethod
    def camera_button(size=14, padding="20px 15px"):
        """Camera control button style with gradient"""
        return f"""
            QPushButton {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2e2e2e, stop:1 #262626);
                color: white;
                font-size: {size}px;
                font-weight: bold;
                padding: {padding};
                border: 1px solid #3e3e3e;
                border-radius: 4px;
            }}
            QPushButton:hover {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #383838, stop:1 #303030);
                border: 1px solid #484848;
            }}
            QPushButton:pressed {{
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1a1a1a);
                border: 1px solid #2e2e2e;
            }}
        """
    
    @staticmethod
    def subbox():
        """Sub-box group style with gradient - no border for nested boxes"""
        return f"QGroupBox {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #222222, stop:1 #1d1d1d); border: none; font-size: 10pt; }} QGroupBox::title {{ color: {Colors.TEXT_SECONDARY}; top: 1px; }}"
    
    @staticmethod
    def transparent_label(color=None, font_size=11, italic=False):
        """Transparent label with customizable properties"""
        color = color or Colors.TEXT_DIM
        style = f"background-color: {Colors.BG_TRANSPARENT}; color: {color}; font-size: {font_size}px;"
        if italic:
            style += " font-style: italic;"
        return style
