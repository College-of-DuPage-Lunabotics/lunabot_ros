#!/usr/bin/env python3
import os
import signal
import subprocess

import rclpy
from rclpy.node import Node

from lunabot_logger import Logger
from lunabot_msgs.srv import LaunchSystem, StopSystem


class LaunchManagerNode(Node):
    def __init__(self):
        super().__init__('launch_manager')
        self.log = Logger(self)

        self.launch_srv = self.create_service(
            LaunchSystem,
            'launch_system',
            self.launch_system_callback
        )
        self.stop_srv = self.create_service(
            StopSystem,
            'stop_system',
            self.stop_system_callback
        )

        self.processes = {
            'pointlio': None,
            'mapping': None,
            'nav2': None,
            'hardware': None
        }

        self.workspace = os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0]
        if 'install' in self.workspace:
            self.workspace = self.workspace.split('/install')[0]
        else:
            self.workspace = os.path.expanduser('~/lunabot_ws')
        
        self.log.success(f'Launch manager started. Workspace: {self.workspace}')
        self.log.info('Services: /launch_system, /stop_system')
    
    def launch_system_callback(self, request, response):
        system_name = request.system_name
        use_sim = request.use_sim
        
        self.log.action(f'Launch request: {system_name} (sim={use_sim})')

        if self.processes.get(system_name) is not None:
            response.success = False
            response.message = f'{system_name} is already running'
            self.log.warning(response.message)
            return response
        
        try:
            if system_name == 'hardware':
                response.success, response.message = self._launch_hardware()
            else:
                response.success, response.message = self._launch_ros_system(system_name, use_sim)
            
            if response.success:
                self.log.success(f'Launched {system_name} successfully')
            else:
                self.log.failure(f'Failed to launch {system_name}: {response.message}')
        
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.log.failure(f'Exception launching {system_name}: {e}')
        
        return response
    
    def stop_system_callback(self, request, response):
        system_name = request.system_name

        self.log.action(f'Stop request: {system_name}')

        if self.processes.get(system_name) is None:
            response.success = True
            response.message = f'{system_name} is not running'
            self.log.info(response.message)
            return response
        
        try:
            process = self.processes[system_name]
            
            # Kill the entire process group (session) since we used start_new_session=True
            if process.poll() is None:  # Process still running
                pgid = os.getpgid(process.pid)
                try:
                    # Send SIGTERM to entire process group
                    os.killpg(pgid, signal.SIGTERM)
                    process.wait(timeout=5)
                    self.log.success(f'Terminated {system_name} gracefully')
                except subprocess.TimeoutExpired:
                    # Force kill if still running
                    os.killpg(pgid, signal.SIGKILL)
                    process.wait()
                    self.log.warning(f'Force killed {system_name} process group')
            
            self.processes[system_name] = None
            response.success = True
            response.message = f'{system_name} stopped successfully'
            
        except Exception as e:
            # Also try pkill as fallback
            self.log.failure(f'Error stopping {system_name}: {e}')
            self._pkill_system(system_name)
            self.processes[system_name] = None
            response.success = True
            response.message = f'{system_name} stopped (with pkill fallback)'
            self.log.warning(f'Used pkill fallback for {system_name}')
        
        return response
    
    def _launch_ros_system(self, system_name, use_sim):
        """Launch a ROS 2 system (pointlio, mapping, nav2)"""
        launch_files = {
            'pointlio': 'pointlio_launch.py',
            'mapping': 'mapping_launch.py',
            'nav2': 'nav2_stack_launch.py'
        }
        
        launch_file = launch_files.get(system_name)
        if not launch_file:
            return False, f'Unknown system: {system_name}'
        
        use_sim_arg = 'true' if use_sim else 'false'
        
        try:
            process = subprocess.Popen(
                ['ros2', 'launch', 'lunabot_bringup', launch_file, f'use_sim:={use_sim_arg}'],
                stdin=subprocess.DEVNULL,
                start_new_session=True
            )
            
            self.processes[system_name] = process
            return True, f'{system_name} launched (PID: {process.pid})'
        
        except Exception as e:
            return False, str(e)
    
    def _launch_hardware(self):
        """Launch hardware (CAN + sensors)"""
        try:
            process = subprocess.Popen(
                ['ros2', 'launch', 'lunabot_bringup', 'hardware_launch.py'],
                stdin=subprocess.DEVNULL,
                start_new_session=True
            )
            
            self.processes['hardware'] = process
            return True, f'Hardware launched (PID: {process.pid})'
        
        except Exception as e:
            return False, str(e)
    
    def _pkill_system(self, system_name):
        """Kill system by pattern using pkill (fallback)"""
        patterns = {
            'pointlio': 'pointlio_launch',
            'mapping': 'mapping_launch',
            'nav2': 'nav2_stack_launch',
            'hardware': 'hardware_launch'
        }
        
        pattern = patterns.get(system_name, system_name)
        try:
            subprocess.run(['pkill', '-9', '-f', pattern], timeout=5)
        except Exception as e:
            self.log.failure(f'pkill failed for {system_name}: {e}')
    
    def cleanup(self):
        self.log.action('Shutting down launch manager, stopping all processes')
        
        for name, process in self.processes.items():
            if process is not None:
                try:
                    if process.poll() is None:  # Still running
                        pgid = os.getpgid(process.pid)
                        os.killpg(pgid, signal.SIGTERM)
                        process.wait(timeout=3)
                        self.log.success(f'Stopped {name}')
                except subprocess.TimeoutExpired:
                    try:
                        pgid = os.getpgid(process.pid)
                        os.killpg(pgid, signal.SIGKILL)
                        process.wait()
                        self.log.warning(f'Force killed {name}')
                    except:
                        pass
                except:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = LaunchManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
