#!/usr/bin/env python3
"""Keyboard and topic control for the Sieglinde simulation."""
import math
import os
import select
import signal
import sys
import termios
import time
import tty
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Twist, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import xacro
from control_kinematics import ArmKinematics

HELP = '''Sieglinde controls (focus this terminal):
  w/s: forward/back    a/d: turn left/right    space: stop base + hold joints
  i/k: tool X +/-      j/l: tool Y +/-         u/o: tool Z +/- (base_link)
  1..5: select arm joint   6: gripper   7: flippers   +/-: jog selected joint
  p: show measured tool XYZ and joints    h: help    q or Ctrl-C: quit
Drive keys must repeat: base stops after 0.5 s without a drive command.
Arm keys continuously jog while repeating; release timeout is 0.2 s.
XYZ is position-only control, not collision-aware path planning.
'''


class ControlNode(Node):
    def __init__(self):
        super().__init__('sieglinde_control')
        defaults = {'keyboard': True, 'linear_speed': 0.2, 'angular_speed': 0.5,
                    'motion_duration': 1.0,
                    'command_timeout': 0.5, 'feedback_timeout': 1.0,
                    'jog_timeout': 0.2, 'jog_horizon': 0.15,
                    'joint_jog_speed': 0.3, 'xyz_jog_speed': 0.08}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        for key, value in self.settings.items():
            if key != 'keyboard' and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{key} must be finite and positive')
        share = get_package_share_directory('sieglinde_sim')
        description = xacro.process_file(os.path.join(share, 'urdf/tracker/tracker.urdf.xacro'), mappings={'simulation': 'false'}).toxml()
        self.model = ArmKinematics(description)
        self.groups = {'arm_controller': self.model.names, 'gripper_controller': ['Grip'], 'flipper_controller': ['flipper_joint']}
        self.joint_names = self.model.names + ['Grip', 'flipper_joint']
        self.positions = {}
        self.received = {}
        self.selected = 0
        self.jog = None
        self.jog_time = -math.inf
        self.jog_group = None
        self.last_jog_sim = None
        self.last_jog_wall = -math.inf
        self.drive = Twist()
        self.drive_time = -math.inf
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pubs = {name: self.create_publisher(JointTrajectory, f'/{name}/joint_trajectory', 10) for name in self.groups}
        self.status_pub = self.create_publisher(String, '/control/status', 10)
        self.ee_pub = self.create_publisher(PointStamped, '/control/ee_position', 10)
        self.create_subscription(JointState, '/joint_states', self.feedback, qos_profile_sensor_data)
        self.create_subscription(Twist, '/control/cmd_vel', self.drive_command, 10)
        self.create_subscription(JointState, '/control/joint_targets', self.joint_command, 10)
        self.create_subscription(PointStamped, '/control/xyz_target', self.xyz_command, 10)
        self.create_subscription(Vector3, '/control/xyz_delta', self.delta_command, 10)
        self.create_subscription(Empty, '/control/stop', lambda _: self.stop(), 10)
        # Watchdog uses real elapsed time even if Gazebo is paused.
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.exit_requested = False
        self.terminal = None
        if self.settings['keyboard'] and sys.stdin.isatty():
            self.terminal = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            print(HELP, flush=True)
        elif self.settings['keyboard']:
            self.report('No interactive terminal; ROS topic control is available.')
        self.report('Ready; waiting for joint feedback. XYZ frame: base_link, tip: tool0.')

    def report(self, message):
        self.status_pub.publish(String(data=message))
        self.get_logger().info(message)

    def feedback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names and math.isfinite(position):
                self.positions[name] = position
                self.received[name] = time.monotonic()

    def ready(self, names):
        return all(time.monotonic() - self.received.get(name, -math.inf) < self.settings['feedback_timeout'] for name in names)

    def drive_command(self, msg):
        if not all(math.isfinite(v) for v in [msg.linear.x, msg.angular.z]):
            self.drive = Twist()
            self.drive_time = -math.inf
            self.report('Rejected non-finite base command; stopping.')
            return
        self.drive = Twist()
        self.drive.linear.x = float(np.clip(msg.linear.x, -self.settings['linear_speed'], self.settings['linear_speed']))
        self.drive.angular.z = float(np.clip(msg.angular.z, -self.settings['angular_speed'], self.settings['angular_speed']))
        self.drive_time = time.monotonic()

    def joint_command(self, msg):
        self.cancel_jog()
        if not msg.name or len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            self.report('Rejected joint command: names and positions must be nonempty, unique and equal length.')
            return
        targets = dict(zip(msg.name, msg.position))
        for name, value in targets.items():
            if name not in self.joint_names or not math.isfinite(value):
                self.report('Rejected joint command: unknown joint or non-finite value.')
                return
            low, high, _ = self.model.limits[name]
            if not low <= value <= high:
                self.report(f'Rejected {name}: {value:.3f} outside [{low:.3f}, {high:.3f}].')
                return
        groups = {group: names for group, names in self.groups.items() if set(names) & targets.keys()}
        if not all(self.ready(names) and self.arm_pubs[group].get_subscription_count() for group, names in groups.items()):
            self.report('Rejected motion: fresh joint feedback and controller subscribers are required.')
            return
        for group, names in groups.items():
            positions = [targets.get(name, self.positions[name]) for name in names]
            duration = max(self.settings['motion_duration'], max(2 * abs(p - self.positions[n]) / self.model.limits[n][2] for n, p in zip(names, positions)))
            point = JointTrajectoryPoint(positions=positions, velocities=[0.] * len(names), time_from_start=Duration(seconds=duration).to_msg())
            self.arm_pubs[group].publish(JointTrajectory(joint_names=names, points=[point]))
        self.report('Accepted joint target: ' + ', '.join(f'{n}={v:.3f}' for n, v in targets.items()))

    def xyz_command(self, msg):
        if msg.header.frame_id != 'base_link':
            self.report('Rejected XYZ target: header.frame_id must be base_link.')
            return
        if not self.ready(self.model.names):
            self.report('Rejected XYZ target: joint feedback is missing or stale.')
            return
        seed = [self.positions[n] for n in self.model.names]
        solution = self.model.inverse([msg.point.x, msg.point.y, msg.point.z], seed)
        if solution is None:
            self.report('Rejected XYZ target: no local joint-limit-respecting IK solution.')
            return
        self.joint_command(JointState(name=self.model.names, position=solution.tolist()))

    def delta_command(self, msg):
        if not self.ready(self.model.names):
            self.report('Rejected XYZ jog: joint feedback is missing or stale.')
            return
        point, _ = self.model.forward([self.positions[n] for n in self.model.names])
        target = PointStamped()
        target.header.frame_id = 'base_link'
        target.point.x, target.point.y, target.point.z = (point + [msg.x, msg.y, msg.z]).tolist()
        self.xyz_command(target)

    def stop(self):
        self.cancel_jog()
        self.drive = Twist()
        self.drive_time = -math.inf
        self.base_pub.publish(Twist())
        for group, names in self.groups.items():
            if self.ready(names):
                point = JointTrajectoryPoint(positions=[self.positions[n] for n in names], velocities=[0.] * len(names), time_from_start=Duration(seconds=0.15).to_msg())
                self.arm_pubs[group].publish(JointTrajectory(joint_names=names, points=[point]))
        self.report('Stopped base; holding joints with available feedback.')

    def key(self, key):
        if key in 'wasd':
            msg = Twist()
            msg.linear.x = {'w': 1, 's': -1}.get(key, 0) * self.settings['linear_speed']
            msg.angular.z = {'a': 1, 'd': -1}.get(key, 0) * self.settings['angular_speed']
            self.drive_command(msg)
        elif key in 'ikjluo':
            axis, sign = {'i': (0, 1), 'k': (0, -1), 'j': (1, 1), 'l': (1, -1), 'u': (2, 1), 'o': (2, -1)}[key]
            self.set_jog(('xyz', axis, sign))
        elif key in '1234567':
            self.cancel_jog()
            self.selected = int(key) - 1
            self.report('Selected ' + self.joint_names[self.selected])
        elif key in '+=-':
            self.set_jog(('joint', self.joint_names[self.selected], -1 if key == '-' else 1))
        elif key == ' ':
            self.stop()
        elif key == 'h':
            print(HELP, flush=True)
        elif key == 'p' and self.ready(self.model.names):
            point, _ = self.model.forward([self.positions[n] for n in self.model.names])
            self.report(f'tool0 XYZ in base_link: {point.round(4).tolist()}; joints: {self.positions}')
        elif key == 'q':
            self.exit_requested = True

    def read_keys(self):
        # Drain bytes in chunks, not one character per timer callback.
        chunks = []
        while select.select([sys.stdin], [], [], 0)[0]:
            data = os.read(sys.stdin.fileno(), 4096)
            if not data:
                break
            chunks.append(data)
        return b''.join(chunks).decode(errors='ignore')

    def process_keys(self, keys):
        # A stop/quit overrides the entire buffered batch, including later repeats.
        if 'q' in keys or ' ' in keys:
            self.stop()
            self.exit_requested = 'q' in keys
            return
        # Repeats refresh the intent; they never queue trajectories or jog distances.
        for key in keys:
            self.key(key)

    def set_jog(self, intent):
        if self.jog != intent:
            self.cancel_jog()
        self.jog = intent
        self.jog_time = time.monotonic()

    def cancel_jog(self):
        self.jog = None
        if self.jog_group is not None:
            names = self.groups[self.jog_group]
            if self.ready(names):
                point = JointTrajectoryPoint(
                    positions=[self.positions[n] for n in names],
                    velocities=[0.] * len(names),
                    time_from_start=Duration(seconds=0.05).to_msg())
                self.arm_pubs[self.jog_group].publish(JointTrajectory(joint_names=names, points=[point]))
        self.jog_group = None
        self.last_jog_sim = None

    def update_jog(self):
        if self.jog is None:
            return
        now = time.monotonic()
        if now - self.jog_time > self.settings['jog_timeout']:
            self.cancel_jog()
            return
        mode, axis, sign = self.jog
        group = 'arm_controller' if mode == 'xyz' else next(g for g, names in self.groups.items() if axis in names)
        names = self.groups[group]
        if not self.ready(names) or not self.arm_pubs[group].get_subscription_count():
            self.cancel_jog()
            self.report('Jog stopped: fresh feedback and a controller subscriber are required.')
            return
        sim_now = self.get_clock().now().nanoseconds
        if self.last_jog_sim is not None and sim_now == self.last_jog_sim:
            return  # Never accumulate commands while simulation is paused.
        if now - self.last_jog_wall < 0.05:
            return
        q = np.array([self.positions[n] for n in names])
        velocity = np.zeros(len(names))
        if mode == 'xyz':
            _, jacobian = self.model.forward(q)
            desired = np.zeros(3)
            desired[axis] = sign * self.settings['xyz_jog_speed']
            velocity = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 0.02**2 * np.eye(3), desired)
        else:
            velocity[names.index(axis)] = sign * self.settings['joint_jog_speed']
        limits = np.array([self.model.limits[n][2] for n in names])
        velocity /= max(1., float(np.max(np.abs(velocity) / limits)))
        horizon = self.settings['jog_horizon']
        lower = np.array([self.model.limits[n][0] for n in names])
        upper = np.array([self.model.limits[n][1] for n in names])
        # Bound lookahead by measured state, preventing windup in a slow simulator.
        brake_time = 0.1
        final = np.clip(q + velocity * (horizon + brake_time / 2), lower, upper)
        velocity = (final - q) / (horizon + brake_time / 2)
        target = q + velocity * horizon
        point = JointTrajectoryPoint(positions=target.tolist(), velocities=velocity.tolist(),
                                     time_from_start=Duration(seconds=horizon).to_msg())
        # End at rest if updates cease, and satisfy the controller's zero-end-velocity policy.
        brake = JointTrajectoryPoint(positions=final.tolist(), velocities=[0.] * len(names),
                                     time_from_start=Duration(seconds=horizon + brake_time).to_msg())
        self.arm_pubs[group].publish(JointTrajectory(joint_names=names, points=[point, brake]))
        self.jog_group = group
        self.last_jog_sim = sim_now
        self.last_jog_wall = now

    def tick(self):
        if self.terminal is not None:
            self.process_keys(self.read_keys())
        self.update_jog()
        self.base_pub.publish(self.drive if time.monotonic() - self.drive_time < self.settings['command_timeout'] else Twist())
        if self.ready(self.model.names):
            point, _ = self.model.forward([self.positions[n] for n in self.model.names])
            msg = PointStamped()
            msg.header.frame_id = 'base_link'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x, msg.point.y, msg.point.z = point.tolist()
            self.ee_pub.publish(msg)

    def restore_terminal(self):
        if self.terminal is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = ControlNode()
        while rclpy.ok() and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.restore_terminal()
            if rclpy.ok():
                node.stop()
                # Let the zero command reach the bridge before destroying publishers.
                for _ in range(3):
                    node.base_pub.publish(Twist())
                    time.sleep(0.05)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
