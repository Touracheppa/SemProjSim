#!/usr/bin/env python3
"""Exercise a running simulation, including actual joint and base movement."""
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu, PointCloud2
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('sieglinde_smoke_test')
    latest = {}
    subscriptions = []
    for topic, cls in [('/odom', Odometry), ('/ground_truth/odom', Odometry), ('/joint_states', JointState), ('/imu/data', Imu), ('/points', PointCloud2), ('/clock', Clock)]:
        subscriptions.append(node.create_subscription(cls, topic, lambda msg, key=topic: latest.update({key: msg}), qos_profile_sensor_data))
    drive = node.create_publisher(Twist, '/cmd_vel', 10)
    publishers = {name: node.create_publisher(JointTrajectory, f'/{name}/joint_trajectory', 10) for name in ['arm_controller', 'gripper_controller', 'flipper_controller']}
    def wait(predicate, seconds=60):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return
        raise RuntimeError(f'Timed out; received topics: {list(latest)}')
    try:
        wait(lambda: len(latest) == 6 and all(p.get_subscription_count() for p in publishers.values()))
        assert latest['/points'].width * latest['/points'].height > 0
        start = latest['/ground_truth/odom'].pose.pose.position
        targets = {'arm_controller': ([f'arm_joint_{i}' for i in range(1, 6)], [0.15, 0.1, -0.1, 0.1, 0.1]), 'gripper_controller': (['Grip'], [0.2]), 'flipper_controller': (['flipper_joint'], [0.2])}
        for name, (joints, positions) in targets.items():
            msg = JointTrajectory(joint_names=joints)
            point = JointTrajectoryPoint(positions=positions)
            point.time_from_start.sec = 3
            msg.points = [point]
            publishers[name].publish(msg)
        def moved():
            state = latest['/joint_states']
            positions = dict(zip(state.name, state.position))
            expected = dict(zip(*targets['arm_controller'])) | {'Grip': 0.2, 'g2': 0.2, 'g3': 0.2, 'flipper_joint': 0.2, 'flipper_joint_mimic': 0.092}
            return all(abs(positions.get(j, 999) - v) < 0.05 for j, v in expected.items())
        wait(moved)
        command = Twist()
        command.linear.x = 0.15
        end = time.monotonic() + 8
        while time.monotonic() < end:
            drive.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)
        position = latest['/ground_truth/odom'].pose.pose.position
        distance = math.hypot(position.x - start.x, position.y - start.y)
        assert distance > 0.1, f'Base did not move: {distance}'
        print(f'PASS: clock, IMU, point cloud, all actuated/mimic joints, actual base displacement {distance:.3f} m')
    finally:
        for _ in range(5):
            drive.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
