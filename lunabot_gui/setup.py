from setuptools import setup
import os
from glob import glob

package_name = "lunabot_gui"

setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    py_modules=["lunabot_gui", "gui_styles", "ros_interface", "ui_widgets"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.png')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Grayson Arendt",
    maintainer_email="grayson.n.arendt5@gmail.com",
    description="PyQt5-based GUI for Lunabot robot control and monitoring",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lunabot_gui = lunabot_gui:main",
        ],
    },
)
