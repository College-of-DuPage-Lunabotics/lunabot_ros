#!/usr/bin/env python3
import os
import sys
import robot_upstart

def help():
    print('Lunabot Launch Manager robot_upstart install script.')
    print('Usage: install_launch_manager_service.py <ROS_DOMAIN_ID>')
    print('ROS_DOMAIN_ID: optional, defaults to 0')

argc = len(sys.argv)

domain_id = 0
if argc == 2:
    try:
        domain_id = int(sys.argv[1])
    except ValueError:
        print('Invalid ROS_DOMAIN_ID {0}'.format(sys.argv[1]))
        help()
        sys.exit(1)

print('Installing Lunabot Launch Manager service. ROS_DOMAIN_ID={0}'.format(domain_id))

# Get workspace path from environment or use default
workspace_dir = os.environ.get('LUNABOT_WS', os.path.expanduser('~/lunabot_ws'))
workspace_setup = workspace_dir + '/install/setup.bash'
cyclonedds_config = workspace_dir + '/src/lunabot_ros/lunabot_bringup/config/cyclonedds.xml'

print(f'Using workspace: {workspace_setup}')
print(f'CycloneDDS config: {cyclonedds_config}')

launch_manager_job = robot_upstart.Job(
    name='lunabot-launch-manager',
    rmw='rmw_cyclonedds_cpp',
    workspace_setup=workspace_setup,
    ros_domain_id=domain_id
)

# Configure CycloneDDS to use all network interfaces, not just localhost
launch_manager_job.environment = {'CYCLONEDDS_URI': f'file://{cyclonedds_config}'}

launch_manager_job.symlink = True
launch_manager_job.add(package='lunabot_bringup', filename='launch/launch_manager.launch.py')

launch_manager_job.install()

print('')
print('Launch Manager service installed!')
print('')
print('To start the service now:')
print('  sudo systemctl start lunabot-launch-manager')
print('')
print('To check status:')
print('  sudo systemctl status lunabot-launch-manager')
print('')
print('To view logs:')
print('  sudo journalctl -u lunabot-launch-manager -f')
print('')
