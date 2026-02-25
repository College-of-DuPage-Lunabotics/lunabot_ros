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
workspace_setup = os.environ.get('LUNABOT_WS', os.path.expanduser('~/lunabot_ws')) + '/install/setup.bash'
print(f'Using workspace: {workspace_setup}')

launch_manager_job = robot_upstart.Job(
    name='lunabot-launch-manager',
    rmw='rmw_cyclonedds_cpp',
    workspace_setup=workspace_setup,
    ros_domain_id=domain_id
)

launch_manager_job.symlink = True
launch_manager_job.add(package='lunabot_bringup', filename='launch/launch_manager.launch.py')

# Add systemd dependencies to wait for network to be fully online (WiFi connected)
launch_manager_job.add_dependencies = ['network-online.target']

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
