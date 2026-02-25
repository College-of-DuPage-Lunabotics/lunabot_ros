#!/usr/bin/env python3
import os
import signal
import subprocess
import rclpy
from rclpy.node import Node
from lunabot_msgs.srv import LaunchSystem, StopSystem


class LaunchManagerNode(Node):
    def __init__(self):
        super().__init__('launch_manager')
        
        # Services for launching and stopping systems
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
        
        # Track running processes
        self.processes = {
            'pointlio': None,
            'mapping': None,
            'nav2': None,
            'localization': None,
            'hardware': None
        }
        
        # Get workspace path from environment
        self.workspace = os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0]
        if 'install' in self.workspace:
            self.workspace = self.workspace.split('/install')[0]
        else:
            self.workspace = os.path.expanduser('~/lunabot_ws')
        
        self.get_logger().info(f'Launch manager started. Workspace: {self.workspace}')
        self.get_logger().info('Services: /launch_system, /stop_system')
    
    def launch_system_callback(self, request, response):
        """Handle launch system service request"""
        system_name = request.system_name
        use_sim = request.use_sim
        
        self.get_logger().info(f'Launch request: {system_name} (sim={use_sim})')
        
        # Check if already running
        if self.processes.get(system_name) is not None:
            response.success = False
            response.message = f'{system_name} is already running'
            self.get_logger().warn(response.message)
            return response
        
        try:
            if system_name == 'hardware':
                # Special case: hardware launch
                response.success, response.message = self._launch_hardware()
            else:
                # Launch standard system
                response.success, response.message = self._launch_ros_system(system_name, use_sim)
            
            if response.success:
                self.get_logger().info(f'Successfully launched {system_name}')
            else:
                self.get_logger().error(f'Failed to launch {system_name}: {response.message}')
        
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Exception launching {system_name}: {e}')
        
        return response
    
    def stop_system_callback(self, request, response):
        """Handle stop system service request"""
        system_name = request.system_name
        
        self.get_logger().info(f'Stop request: {system_name}')
        
        # Check if running
        if self.processes.get(system_name) is None:
            response.success = True
            response.message = f'{system_name} is not running'
            self.get_logger().info(response.message)
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
                    self.get_logger().info(f'Terminated {system_name} process group gracefully')
                except subprocess.TimeoutExpired:
                    # Force kill if still running
                    os.killpg(pgid, signal.SIGKILL)
                    process.wait()
                    self.get_logger().warn(f'Force killed {system_name} process group')
            
            self.processes[system_name] = None
            response.success = True
            response.message = f'{system_name} stopped successfully'
            
        except Exception as e:
            # Also try pkill as fallback
            self.get_logger().error(f'Error stopping {system_name}: {e}')
            self._pkill_system(system_name)
            self.processes[system_name] = None
            response.success = True
            response.message = f'{system_name} stopped (with pkill fallback)'
            self.get_logger().warn(f'Used pkill fallback for {system_name}')
        
        return response
    
    def _launch_ros_system(self, system_name, use_sim):
        """Launch a ROS 2 system (pointlio, mapping, nav2, localization)"""
        # Map system names to launch files
        launch_files = {
            'pointlio': 'pointlio_launch.py',
            'mapping': 'mapping_launch.py',
            'nav2': 'nav2_stack_launch.py',
            'localization': 'localization_launch.py'
        }
        
        launch_file = launch_files.get(system_name)
        if not launch_file:
            return False, f'Unknown system: {system_name}'
        
        use_sim_arg = 'true' if use_sim else 'false'
        
        try:
            # Launch in background (output goes to systemd journal)
            process = subprocess.Popen(
                ['ros2', 'launch', 'lunabot_bringup', launch_file, f'use_sim:={use_sim_arg}'],
                stdin=subprocess.DEVNULL,
                stdout=None,  # Inherit parent's stdout (systemd journal)
                stderr=None,  # Inherit parent's stderr (systemd journal)
                start_new_session=True  # Detach from this process
            )
            
            self.processes[system_name] = process
            return True, f'{system_name} launched (PID: {process.pid})'
        
        except Exception as e:
            return False, str(e)
    
    def _launch_hardware(self):
        """Launch hardware (CAN + sensors)"""
        try:
            # Launch hardware (output goes to systemd journal)
            process = subprocess.Popen(
                ['ros2', 'launch', 'lunabot_bringup', 'hardware_launch.py'],
                stdin=subprocess.DEVNULL,
                stdout=None,  # Inherit parent's stdout (systemd journal)
                stderr=None,  # Inherit parent's stderr (systemd journal)
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
            'localization': 'localization_launch',
            'hardware': 'hardware_launch'
        }
        
        pattern = patterns.get(system_name, system_name)
        try:
            subprocess.run(['pkill', '-9', '-f', pattern], timeout=5)
        except Exception as e:
            self.get_logger().error(f'pkill failed for {system_name}: {e}')
    
    def cleanup(self):
        """Clean up all running processes on shutdown"""
        self.get_logger().info('Shutting down launch manager, stopping all processes...')
        
        for name, process in self.processes.items():
            if process is not None:
                try:
                    if process.poll() is None:  # Still running
                        pgid = os.getpgid(process.pid)
                        os.killpg(pgid, signal.SIGTERM)
                        process.wait(timeout=3)
                        self.get_logger().info(f'Stopped {name}')
                except subprocess.TimeoutExpired:
                    try:
                        pgid = os.getpgid(process.pid)
                        os.killpg(pgid, signal.SIGKILL)
                        process.wait()
                        self.get_logger().info(f'Force killed {name}')
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
