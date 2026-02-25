# scripts

This folder contains various setup and utility scripts.

## Content
- **config**
  - **99-realsense-d4xx-mipi-dfu.rules**: Udev rules for RealSense D456 cameras.
  - **99-realsense-libusb.rules**: Udev rules for RealSense cameras using USB.
- **canable_start.sh**: Sets up CAN bus for motor controller communication.
- **install_dependencies.sh**: Installs all required dependencies including ROS packages, CycloneDDS, Livox SDK, GUI dependencies, and camera drivers.
- **setup_robot_services.sh**: Master setup script for configuring robot services (CAN and launch manager) using robot_upstart. Run this on the robot with sudo.
- **install_launch_manager_service.py**: robot_upstart installer script for the launch manager service. Called automatically by setup_robot_services.sh.
- **setup_udev_rules.sh**: Sets up udev rules for Intel RealSense cameras.

## Robot Setup
To set up services on the robot (nucbox):
```bash
sudo ./setup_robot_services.sh
```

This will install and enable:
- canable-setup.service (CAN bus initialization)
- lunabot-launch-manager (remote launch control via ROS service)
