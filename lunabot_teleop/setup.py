from setuptools import setup

package_name = "lunabot_teleop"

setup(
    name=package_name,
    version="0.0.0",
    package_dir={"": "src"},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Grayson Arendt",
    maintainer_email="grayson.n.arendt@gmail.com",
    description="This package contains code to control the robot with either a keyboard or game controller.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_teleop = keyboard_teleop:main",
        ],
    },
)
