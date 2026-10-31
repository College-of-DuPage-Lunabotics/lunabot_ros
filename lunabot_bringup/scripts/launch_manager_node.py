#!/usr/bin/env python3
import os
import re
import signal
import subprocess
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from lunabot_logger import Logger
from lunabot_msgs.msg import LaunchStatus
from lunabot_msgs.srv import LaunchSystem, StopSystem

_LAUNCH_FILES = {
    'lio': 'fastlio_launch.py',
    'mapping': 'mapping_launch.py',
    'nav2': 'nav2_stack_launch.py',
    'hardware': 'hardware_launch.py',
    'record': 'record_launch.py',
}

# ros2 launch escalates SIGINT -> SIGTERM -> SIGKILL on its own nodes, so give it time
_STOP_TIMEOUT = 15.0

# Full output of each launch, for `tail -f` over SSH
_LOG_DIR = os.path.join(os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')), 'launch_manager')

# Lines of a node's own output to report along with its death
_DEATH_CONTEXT_LINES = 5

# "[rtabmap-2] some output" -> a node's own stdout/stderr, prefixed by ros2 launch
_NODE_OUTPUT = re.compile(r'^\[(?P<label>[\w.\-]+-\d+)\] (?P<text>.*)')
# "[ERROR] [rtabmap-2]: process has died [pid 9056, exit code -6, ...]"
_NODE_DIED = re.compile(
    r'\[ERROR\] \[(?P<label>(?P<node>[\w.\-]+)-\d+)\]: process has died \[pid \d+, exit code (?P<code>-?\d+)')


def _pid_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _signal_until_gone(send, alive, reap=lambda: None):
    """SIGINT, then SIGKILL whatever is left after _STOP_TIMEOUT.
    Returns 'graceful', 'forced' or 'failed'."""
    for sig, timeout in ((signal.SIGINT, _STOP_TIMEOUT), (signal.SIGKILL, 2.0)):
        result = 'graceful' if sig == signal.SIGINT else 'forced'
        try:
            send(sig)
        except ProcessLookupError:
            return result
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            reap()
            if not alive():
                return result
            time.sleep(0.1)
    return 'failed'


def _stop_group(pgid, reap=lambda: None):
    """Stop every process in a group. Waiting on the ros2 launch PID alone is not enough:
    it can exit while its nodes ignore the signal and keep running as orphans that still
    hold the cameras, serial ports and LiDAR."""
    return _signal_until_gone(
        lambda sig: os.killpg(pgid, sig), lambda: _group_alive(pgid), reap)


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

        status_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(LaunchStatus, '/launch_manager/status', status_qos)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.processes = {
            'lio': None,
            'mapping': None,
            'nav2': None,
            'hardware': None
        }

        # Nodes that died while their launch kept running, per system
        self.dead_nodes = {}
        self.dead_nodes_lock = threading.Lock()

        self.workspace = os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0]
        if 'install' in self.workspace:
            self.workspace = self.workspace.split('/install')[0]
        else:
            self.workspace = os.path.expanduser('~/lunabot_ws')
        
        self.log.success(f'Launch manager started. Workspace: {self.workspace}')
        self.log.info('Services: /launch_system, /stop_system')

    def _reap_exited(self):
        """Forget managed launches that exited on their own (crash, Ctrl-C, etc.)"""
        for name, process in self.processes.items():
            if process is not None and process.poll() is not None:
                self.log.warning(f'{name} exited on its own (code {process.returncode})')
                process.stopping = True
                # start_new_session=True makes the launch PID the group id
                if _group_alive(process.pid):
                    self.log.warning(f'{name} left nodes running, stopping them')
                    _stop_group(process.pid)
                self.processes[name] = None

    def _find_external(self):
        """Find lunabot_bringup launches not started by this node, e.g. typed over SSH"""
        owned = {p.pid for p in self.processes.values() if p is not None}
        external = {}
        try:
            result = subprocess.run(
                ['pgrep', '-af', 'ros2 launch lunabot_bringup'],
                capture_output=True, text=True, timeout=2)
        except Exception as e:
            self.log.warning(f'pgrep failed: {e}')
            return external

        for line in result.stdout.splitlines():
            pid_str, _, cmdline = line.partition(' ')
            if not pid_str.isdigit() or int(pid_str) in owned:
                continue
            args = cmdline.split()
            for name, launch_file in _LAUNCH_FILES.items():
                if launch_file in args:
                    external.setdefault(name, []).append(int(pid_str))
        return external

    def publish_status(self):
        self._reap_exited()
        external = self._find_external()
        managed = {name for name, p in self.processes.items() if p is not None}
        with self.dead_nodes_lock:
            degraded = {name for name, nodes in self.dead_nodes.items() if nodes and name in managed}

        msg = LaunchStatus()
        msg.running = sorted(managed | set(external))
        msg.external = sorted(external)
        msg.degraded = sorted(degraded)
        self.status_pub.publish(msg)
    
    def launch_system_callback(self, request, response):
        system_name = request.system_name
        use_sim = request.use_sim
        
        self.log.action(f'Launch request: {system_name} (sim={use_sim})')

        self._reap_exited()
        if self.processes.get(system_name) is not None:
            response.success = False
            response.message = f'{system_name} is already running'
            self.log.warning(response.message)
            return response

        external_pids = self._find_external().get(system_name)
        if external_pids:
            response.success = False
            response.message = (f'{system_name} is already running outside the launch manager '
                                f'(PID {", ".join(map(str, external_pids))})')
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

        self.publish_status()
        return response
    
    def stop_system_callback(self, request, response):
        system_name = request.system_name

        self.log.action(f'Stop request: {system_name}')

        self._reap_exited()
        if self.processes.get(system_name) is None:
            external_pids = self._find_external().get(system_name)
            if external_pids:
                response.success, response.message = self._stop_external(system_name, external_pids)
            else:
                response.success = True
                response.message = f'{system_name} is not running'
                self.log.info(response.message)
            self.publish_status()
            return response
        
        try:
            process = self.processes[system_name]
            process.stopping = True   # nodes dying from here on are expected

            # start_new_session=True makes the launch PID the group id
            result = _stop_group(process.pid, reap=process.poll)
            if result == 'failed':
                response.success = False
                response.message = f'{system_name} did not exit, even after SIGKILL'
                self.log.failure(response.message)
            else:
                if result == 'forced':
                    self.log.warning(f'Force killed {system_name} process group')
                else:
                    self.log.success(f'Terminated {system_name} gracefully')
                self.processes[system_name] = None
                response.success = True
                response.message = f'{system_name} stopped successfully'

        except Exception as e:
            response.success = False
            response.message = f'Error stopping {system_name}: {e}'
            self.log.failure(response.message)

        self.publish_status()
        return response
    
    def _launch_ros_system(self, system_name, use_sim):
        """Launch a ROS 2 system (lio, mapping, nav2, hardware, record)"""
        launch_file = _LAUNCH_FILES.get(system_name)
        if not launch_file:
            return False, f'Unknown system: {system_name}'

        use_sim_arg = 'true' if use_sim else 'false'
        return self._start_launch(system_name, [launch_file, f'use_sim:={use_sim_arg}'])

    def _launch_hardware(self):
        """Launch hardware (CAN + sensors)"""
        return self._start_launch('hardware', [_LAUNCH_FILES['hardware']])

    def _start_launch(self, system_name, launch_args):
        try:
            # Unbuffered so lines reach the reader as they happen, not in 4 KB chunks
            env = dict(os.environ, PYTHONUNBUFFERED='1')
            process = subprocess.Popen(
                ['ros2', 'launch', 'lunabot_bringup', *launch_args],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors='replace',
                env=env,
                start_new_session=True
            )
        except Exception as e:
            return False, str(e)

        process.stopping = False
        with self.dead_nodes_lock:
            self.dead_nodes[system_name] = set()
        self.processes[system_name] = process
        threading.Thread(
            target=self._read_output, args=(system_name, process), daemon=True).start()

        log_path = os.path.join(_LOG_DIR, f'{system_name}.log')
        return True, f'{system_name} launched (PID: {process.pid}, log: {log_path})'

    def _read_output(self, system_name, process):
        """Drain a launch's output (an unread pipe would stall it). Everything goes to the
        journal and a per-system log file; nodes that die unexpectedly are reported on
        /rosout, which the GUI shows, along with their last lines of output."""
        recent = {}   # launch label (e.g. rtabmap-2) -> last lines it printed
        try:
            os.makedirs(_LOG_DIR, exist_ok=True)
            log_file = open(os.path.join(_LOG_DIR, f'{system_name}.log'), 'w')
        except OSError as e:
            self.log.warning(f'Cannot write {system_name} log file: {e}')
            log_file = None

        for line in process.stdout:
            sys.stdout.write(line)
            if log_file:
                log_file.write(line)
                log_file.flush()

            output = _NODE_OUTPUT.match(line)
            if output:
                recent.setdefault(output['label'], deque(maxlen=_DEATH_CONTEXT_LINES)).append(
                    output['text'].rstrip())
                continue

            died = _NODE_DIED.search(line)
            if died and not process.stopping:
                with self.dead_nodes_lock:
                    self.dead_nodes.setdefault(system_name, set()).add(died['node'])
                self.log.failure(f'{system_name}: {died["node"]} died (exit code {died["code"]})')
                for text in recent.get(died['label'], ()):
                    self.log.failure(f'  {died["node"]}: {text}')

        if log_file:
            log_file.close()

    def _stop_external(self, system_name, pids):
        """Stop a launch this node did not start. A command typed in an interactive shell
        leads its own process group, so its nodes are stopped with it; otherwise SIGINT the
        launch and let it shut down its own nodes."""
        pid_list = ", ".join(map(str, pids))
        self.log.action(f'Stopping externally started {system_name} (PID {pid_list})')

        for pid in pids:
            try:
                is_leader = os.getpgid(pid) == pid
                if is_leader:
                    result = _stop_group(pid)
                else:
                    result = _signal_until_gone(
                        lambda sig: os.kill(pid, sig), lambda: _pid_alive(pid))
            except ProcessLookupError:
                continue
            except PermissionError:
                return False, f'No permission to stop {system_name} (PID {pid}), it belongs to another user'

            if result == 'failed':
                return False, f'{system_name} (PID {pid}) did not exit, even after SIGKILL'
            if result == 'forced':
                self.log.warning(f'Force killed externally started {system_name} (PID {pid})')

        self.log.success(f'Stopped externally started {system_name}')
        return True, f'{system_name} stopped'

    def cleanup(self):
        self.log.action('Shutting down launch manager, stopping all processes')
        
        for name, process in self.processes.items():
            if process is not None:
                process.stopping = True
                try:
                    if _stop_group(process.pid, reap=process.poll) == 'failed':
                        self.log.failure(f'{name} did not exit')
                    else:
                        self.log.success(f'Stopped {name}')
                except Exception as e:
                    self.log.warning(f'Error stopping {name}: {e}')


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
