from setuptools import setup

package_name = "lunabot_util"

setup(
    name=package_name,
    version="1.0.0",
    package_dir={"": "src"},
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Grayson Arendt",
    maintainer_email="grayson.n.arendt5@gmail.com",
    description="This package contains various utility nodes.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
