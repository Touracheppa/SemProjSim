"""Verify URDF-derived IK and controller input validation without ROS transport."""
import sys
from pathlib import Path
import numpy as np
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from control_kinematics import ArmKinematics
from control_node import ControlNode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


@pytest.fixture
def model():
    path = Path(get_package_share_directory('sieglinde_sim')) / 'urdf/tracker/tracker.urdf.xacro'
    return ArmKinematics(xacro.process_file(str(path), mappings={'simulation': 'false'}).toxml())


def test_jacobian_matches_finite_difference(model):
    q = np.array([0.2, 0.3, -0.4, 0.5, 0.2])
    p, jacobian = model.forward(q)
    numeric = np.column_stack([(model.forward(q + np.eye(5)[i] * 1e-6)[0] - p) / 1e-6 for i in range(5)])
    assert np.allclose(jacobian, numeric, atol=1e-6)


@pytest.mark.parametrize('seed', [np.zeros(5), np.array([0.2, 0.3, -0.4, 0.5, 0.2])])
def test_reachable_xyz(model, seed):
    target = model.forward(seed + np.array([0.05, 0.03, -0.03, 0.02, 0]))[0]
    solution = model.inverse(target, seed)
    assert solution is not None
    assert np.linalg.norm(model.forward(solution)[0] - target) < 0.002
    assert np.all(solution >= model.lower) and np.all(solution <= model.upper)


def test_unreachable_and_invalid_xyz(model):
    assert model.inverse([100, 100, 100], np.zeros(5)) is None
    assert model.inverse([np.nan, 0, 0], np.zeros(5)) is None


class Recorder:
    def __init__(self):
        self.messages = []
    def publish(self, msg):
        self.messages.append(msg)
    def get_subscription_count(self):
        return 1


class Harness:
    joint_command = ControlNode.joint_command
    drive_command = ControlNode.drive_command
    cancel_jog = ControlNode.cancel_jog
    set_jog = ControlNode.set_jog
    update_jog = ControlNode.update_jog
    process_keys = ControlNode.process_keys
    key = ControlNode.key
    def __init__(self, model):
        self.jog = None
        self.jog_group = None
        self.last_jog_sim = None
        self.last_jog_wall = -float("inf")
        self.selected = 0
        self.exit_requested = False
        self.model = model
        self.joint_names = model.names + ['Grip', 'flipper_joint']
        self.groups = {'arm_controller': model.names}
        self.arm_pubs = {'arm_controller': Recorder()}
        self.positions = dict.fromkeys(self.joint_names, 0.)
        self.settings = {'motion_duration': 1., 'linear_speed': .2, 'angular_speed': .5, 'jog_timeout': .2, 'jog_horizon': .15,
                         'joint_jog_speed': .3, 'xyz_jog_speed': .08}
        self.fresh = True
    def report(self, message):
        self.last_report = message
    def ready(self, names):
        return self.fresh


def test_partial_joint_and_rejections(model):
    node = Harness(model)
    node.joint_command(JointState(name=['arm_joint_1'], position=[.2]))
    messages = node.arm_pubs['arm_controller'].messages
    assert len(messages) == 1
    assert list(messages[0].points[0].positions) == [.2, 0., 0., 0., 0.]
    for msg in [JointState(name=['arm_joint_2'], position=[99.]),
                JointState(name=['g2'], position=[.1]),
                JointState(name=['arm_joint_1'], position=[float('nan')]),
                JointState(name=['arm_joint_1'], position=[]),
                JointState(name=['arm_joint_1'] * 2, position=[.1, .2])]:
        node.joint_command(msg)
        assert len(messages) == 1
    node.fresh = False
    node.joint_command(JointState(name=['arm_joint_1'], position=[.3]))
    assert len(messages) == 1


def test_drive_clamp_and_invalid_stop(model):
    node = Harness(model)
    msg = Twist()
    msg.linear.x = 10.
    msg.angular.z = -10.
    node.drive_command(msg)
    assert node.drive.linear.x == .2 and node.drive.angular.z == -.5
    msg.linear.x = float('nan')
    node.drive_command(msg)
    assert node.drive.linear.x == 0 and node.drive.angular.z == 0


class FakeClock:
    nanoseconds = 1_000_000_000
    def now(self):
        return self


def test_buffered_repeats_do_not_queue_jogs(model, monkeypatch):
    node = Harness(model)
    clock = FakeClock()
    node.get_clock = lambda: clock
    monkeypatch.setattr('control_node.time.monotonic', lambda: 10.)
    node.process_keys('1' + '+' * 300)
    messages = node.arm_pubs['arm_controller'].messages
    assert not messages
    node.update_jog()
    assert len(messages) == 1
    point = messages[0].points[0]
    assert abs(point.positions[0] - .045) < 1e-9
    assert point.velocities[0] == pytest.approx(.3)
    assert list(messages[0].points[-1].velocities) == [0.] * 5
    assert point.time_from_start.sec == 0 and point.time_from_start.nanosec == 150000000
    # Even after wall time passes, a paused simulator gets no new trajectory.
    monkeypatch.setattr('control_node.time.monotonic', lambda: 10.1)
    node.update_jog()
    assert len(messages) == 1
    monkeypatch.setattr('control_node.time.monotonic', lambda: 10.21)
    node.update_jog()
    assert node.jog is None
    assert list(messages[-1].points[0].velocities) == [0.] * 5
    assert list(messages[-1].points[0].positions) == [0.] * 5


def test_stop_overrides_whole_input_batch(model):
    node = Harness(model)
    stopped = []
    node.stop = lambda: stopped.append(True)
    node.process_keys('w' * 200 + ' ' + '+' * 200)
    assert stopped == [True] and node.jog is None
    node.process_keys('+++qww')
    assert node.exit_requested


def test_drain_all_ready_input(monkeypatch):
    import os
    from types import SimpleNamespace
    from control_node import ControlNode
    read_fd, write_fd = os.pipe()
    try:
        os.write(write_fd, b'w' * 300 + b' ')
        with os.fdopen(read_fd, 'rb') as stream:
            monkeypatch.setattr('control_node.sys.stdin', stream)
            assert ControlNode.read_keys(SimpleNamespace()) == 'w' * 300 + ' '
            assert ControlNode.read_keys(SimpleNamespace()) == ''
    finally:
        os.close(write_fd)


def test_cartesian_jog_is_bounded(model, monkeypatch):
    node = Harness(model)
    clock = FakeClock()
    node.get_clock = lambda: clock
    monkeypatch.setattr('control_node.time.monotonic', lambda: 10.)
    node.set_jog(('xyz', 2, 1))
    node.update_jog()
    point = node.arm_pubs['arm_controller'].messages[-1].points[0]
    v = np.array(point.velocities)
    assert np.all(np.abs(v) <= [model.limits[n][2] for n in model.names])
    q = np.array(point.positions)
    assert np.all(q >= model.lower) and np.all(q <= model.upper)
    assert (model.forward(np.zeros(5))[1] @ v)[2] > 0
