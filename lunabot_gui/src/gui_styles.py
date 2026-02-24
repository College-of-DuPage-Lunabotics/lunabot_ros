# Color Palette
class Colors:
    """Color constants for GUI theme"""
    # Background colors
    BG_MAIN = "#1a1a1a"
    BG_BOX = "#2d2d2d"
    BG_SUBBOX = "#3a3a3a"
    BG_TRANSPARENT = "transparent"
    
    # Border and accent colors
    BORDER = "#505050"
    BORDER_DARK = "#404040"
    
    # Text colors
    TEXT_MAIN = "#e0e0e0"
    TEXT_SECONDARY = "#bbb"
    TEXT_DIM = "#aaa"
    TEXT_DARKER = "#888"
    TEXT_GRAY = "#666"
    
    # Status colors
    STATUS_ERROR = "#d32f2f"
    STATUS_SUCCESS = "#66bb6a"
    STATUS_WARNING = "#ffa726"
    STATUS_INFO = "#2196f3"
    
    # Section label color
    LABEL_SECTION = "#4da3f0"
    
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


# Global Application Stylesheet
MAIN_STYLESHEET = f"""
    QMainWindow {{
        background-color: {Colors.BG_MAIN};
    }}
    QWidget#centralWidget {{
        background-color: {Colors.BG_MAIN};
    }}
    QGroupBox {{
        background-color: {Colors.BG_BOX};
        border: 2px solid {Colors.BORDER};
        border-radius: 5px;
        margin: 0px;
        font-weight: bold;
        padding-top: 18px;
        padding-left: 4px;
        padding-right: 4px;
        padding-bottom: 4px;
        color: {Colors.TEXT_MAIN};
    }}
    QGroupBox:disabled {{
        background-color: #1f1f1f;
        border-color: #333333;
        color: #555555;
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        subcontrol-position: top left;
        padding: 0 5px;
        top: 5px;
        left: 2px;
        color: {Colors.LABEL_SECTION};
    }}
    QGroupBox:disabled::title {{
        color: #555555;
    }}
    QLabel {{
        background-color: {Colors.BG_TRANSPARENT};
        color: {Colors.TEXT_MAIN};
    }}
    QLabel:disabled {{
        color: #555555;
    }}
    QPushButton:disabled {{
        background-color: #1f1f1f;
        color: #555555;
        border-color: #333333;
    }}
    QProgressBar {{
        background-color: {Colors.BG_MAIN};
    }}
"""


# Common Widget Styles
class Styles:
    """Common stylesheet patterns"""
    
    @staticmethod
    def camera_label():
        return f"background-color: {Colors.BG_MAIN}; color: {Colors.TEXT_DARKER}; border: 1px solid {Colors.BORDER_DARK};"
    
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
    def standard_button(size=12):
        """Standard button style"""
        return f"""
            QPushButton {{
                background-color: {Colors.BTN_INACTIVE};
                color: white;
                font-size: {size}px;
                font-weight: bold;
                padding: 8px;
                border-radius: 4px;
                border: 2px solid {Colors.BTN_HOVER};
            }}
            QPushButton:hover {{
                background-color: {Colors.BTN_HOVER};
            }}
            QPushButton:pressed {{
                background-color: {Colors.BTN_PRESSED};
            }}
            QPushButton:disabled {{
                background-color: #1f1f1f;
                color: #555555;
                border: 2px solid #333333;
            }}
        """
    
    @staticmethod
    def orange_button(size=12):
        """Orange action button style (e.g., Home)"""
        return f"""
            QPushButton {{
                background-color: {Colors.BTN_ORANGE};
                color: white;
                font-size: {size}px;
                font-weight: bold;
                padding: 8px;
                border-radius: 4px;
                border: 2px solid {Colors.BTN_ORANGE_HOVER};
            }}
            QPushButton:hover {{
                background-color: {Colors.BTN_ORANGE_HOVER};
            }}
            QPushButton:pressed {{
                background-color: {Colors.BTN_ORANGE_PRESSED};
            }}
            QPushButton:disabled {{
                background-color: #1f1f1f;
                color: #555555;
                border: 2px solid #333333;
            }}
        """
    
    @staticmethod
    def blue_button(size=12):
        """Blue action button style (e.g., Auto)"""
        return f"""
            QPushButton {{
                background-color: {Colors.BTN_BLUE};
                color: white;
                font-size: {size}px;
                font-weight: bold;
                padding: 8px;
                border-radius: 4px;
                border: 2px solid {Colors.BTN_BLUE_HOVER};
            }}
            QPushButton:hover {{
                background-color: {Colors.BTN_BLUE_HOVER};
            }}
            QPushButton:pressed {{
                background-color: {Colors.BTN_BLUE_PRESSED};
            }}
            QPushButton:disabled {{
                background-color: #1f1f1f;
                color: #555555;
                border: 2px solid #333333;
            }}
        """
    
    @staticmethod
    def red_button(size=12):
        """Red action button style (e.g., Emergency Stop)"""
        return f"""
            QPushButton {{
                background-color: {Colors.BTN_RED};
                color: white;
                font-size: {size}px;
                font-weight: bold;
                padding: 8px;
                border-radius: 4px;
                border: 2px solid {Colors.BTN_RED_HOVER};
            }}
            QPushButton:hover {{
                background-color: {Colors.BTN_RED_HOVER};
            }}
            QPushButton:pressed {{
                background-color: {Colors.BTN_RED_PRESSED};
            }}
            QPushButton:disabled {{
                background-color: #1f1f1f;
                color: #555555;
                border: 2px solid #333333;
            }}
        """
    
    @staticmethod
    def light_blue_button(size=11):
        """Light blue button style (e.g., RViz2)"""
        return f"""
            QPushButton {{
                background-color: #3a8fbf;
                color: white;
                font-size: {size}px;
                font-weight: bold;
                padding: 6px;
                border-radius: 4px;
                border: 2px solid #5aa3cc;
            }}
            QPushButton:hover {{
                background-color: #5aa3cc;
            }}
            QPushButton:pressed {{
                background-color: #2a7fa8;
            }}
            QPushButton:disabled {{
                background-color: #1f1f1f;
                color: #555555;
                border: 2px solid #333333;
            }}
        """
    
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
                border-radius: 8px;
                border: 2px solid {Colors.BTN_RED_HOVER};
            }}
            QPushButton:hover {{
                background-color: {Colors.BTN_RED_HOVER};
                border: 2px solid {Colors.BTN_RED_PRESSED};
            }}
            QPushButton:pressed {{
                background-color: {Colors.BTN_RED_PRESSED};
            }}
        """
    
    @staticmethod
    def teleop_button(size=14):
        """Teleop control button style"""
        return f"""
            QPushButton {{
                background-color: {Colors.BTN_INACTIVE};
                color: white;
                font-size: {size}px;
                font-weight: bold;
                padding: 10px 20px;
                border-radius: 5px;
                border: 2px solid {Colors.BTN_HOVER};
                min-width: 50px;
                min-height: 50px;
            }}
            QPushButton:hover {{
                background-color: {Colors.BTN_HOVER};
            }}
            QPushButton:pressed {{
                background-color: {Colors.BTN_PRESSED};
            }}
            QPushButton:checked {{
                background-color: {Colors.BTN_ACTIVE};
                border: 2px solid {Colors.BTN_BLUE};
            }}
        """
    
    @staticmethod
    def subbox():
        """Sub-box group style"""
        return f"QGroupBox {{ background-color: {Colors.BG_SUBBOX}; }}"
    
    @staticmethod
    def transparent_label(color=None, font_size=11, italic=False):
        """Transparent label with customizable properties"""
        color = color or Colors.TEXT_DIM
        style = f"background-color: {Colors.BG_TRANSPARENT}; color: {color}; font-size: {font_size}px;"
        if italic:
            style += " font-style: italic;"
        return style
