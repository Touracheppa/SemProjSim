#!/usr/bin/env python3
"""Exercise a running control_node and simulation; moves arm and base."""
import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
from rclpy.qos import qos_profile_sensor_data
from control_kinematics import ArmKinematics
from ament_index_python.packages import get_package_share_directory
import xacro


def main():
    rclpy.init()
    node = rclpy.create_node('control_smoke_test')
    latest = {}
    topics = {'/control/ee_position': PointStamped, '/joint_states': JointState,
              '/control/status': String, '/cmd_vel': Twist, '/ground_truth/odom': Odometry}
    subscriptions = [node.create_subscription(cls, key, lambda m, k=key: latest.update({k: m}), qos_profile_sensor_data) for key, cls in topics.items()]
    joint_pub = node.create_publisher(JointState, '/control/joint_targets', 10)
    xyz_pub = node.create_publisher(PointStamped, '/control/xyz_target', 10)
    drive_pub = node.create_publisher(Twist, '/control/cmd_vel', 10)
    stop_pub = node.create_publisher(Empty, '/control/stop', 10)
    def wait(predicate, seconds=45):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise RuntimeError('Control test timeout; last status: ' + str(latest.get('/control/status')))
    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
    def positions():
        m = latest['/joint_states']
        return dict(zip(m.name, m.position))
    try:
        wait(lambda: all(key in latest for key in ['/control/ee_position', '/joint_states', '/ground_truth/odom', '/cmd_vel']) and joint_pub.get_subscription_count())
        joint_pub.publish(JointState(name=['arm_joint_1'], position=[.2]))
        wait(lambda: abs(positions()['arm_joint_1'] - .2) < .015)
        model = ArmKinematics(xacro.process_file(get_package_share_directory('sieglinde_sim') + '/urdf/tracker/tracker.urdf.xacro', mappings={'simulation': 'false'}).toxml())
        seed = np.array([positions()[name] for name in model.names])
        target = model.forward(seed + np.array([.03, .04, -.03, .02, 0.]))[0]
        msg = PointStamped()
        msg.header.frame_id = 'base_link'
        msg.point.x, msg.point.y, msg.point.z = target.tolist()
        xyz_pub.publish(msg)
        def reached():
            p = latest['/control/ee_position'].point
            return np.linalg.norm(np.array([p.x, p.y, p.z]) - target) < .006
        wait(reached)
        msg.point.x = 100.
        xyz_pub.publish(msg)
        wait(lambda: 'Rejected XYZ target' in latest.get('/control/status', String()).data)
        start = latest['/ground_truth/odom'].pose.pose.position
        command = Twist()
        command.linear.x = .15
        end = time.monotonic() + 4
        while time.monotonic() < end:
            drive_pub.publish(command)
            spin(.1)
        wait(lambda: latest['/cmd_vel'].linear.x == 0., seconds=2)
        finish = latest['/ground_truth/odom'].pose.pose.position
        displacement = math.hypot(finish.x - start.x, finish.y - start.y)
        assert displacement > .03, displacement
        stop_pub.publish(Empty())
        wait(lambda: 'Stopped base' in latest.get('/control/status', String()).data)
        print(f'PASS: individual joint, Cartesian target, unreachable-target rejection, drive ({displacement:.3f} m), command timeout, stop')
    finally:
        for _ in range(3):
            stop_pub.publish(Empty())
            spin(.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
