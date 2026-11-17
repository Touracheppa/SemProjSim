"""URDF-derived position kinematics; no hard-coded arm transform chain."""
import math
import xml.etree.ElementTree as ET
import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew


class ArmKinematics:
    def __init__(self, description, base='base_link', tip='tool0'):
        root = ET.fromstring(description)
        joints = root.findall('joint')
        by_child = {j.find('child').get('link'): j for j in joints}
        self.chain = []
        while tip != base:
            joint = by_child[tip]
            self.chain.insert(0, joint)
            tip = joint.find('parent').get('link')
        self.names = [j.get('name') for j in self.chain if j.get('type') != 'fixed']
        self.limits = {}
        for joint in joints:
            if joint.get('type') not in ('revolute', 'continuous'):
                continue
            limit = joint.find('limit')
            self.limits[joint.get('name')] = (
                float(limit.get('lower', '-inf')), float(limit.get('upper', 'inf')),
                float(limit.get('velocity')))
        self.lower = np.array([self.limits[n][0] for n in self.names])
        self.upper = np.array([self.limits[n][1] for n in self.names])
        self.transforms = []
        for joint in self.chain:
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = rotation([0, 0, 1], rpy[2]) @ rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0])
            axis = joint.find('axis')
            self.transforms.append((transform, np.fromstring(axis.get('xyz'), sep=' ') if axis is not None else np.array([1., 0, 0]), joint.get('type') != 'fixed'))

    def forward(self, positions):
        transform = np.eye(4)
        axes, origins = [], []
        index = 0
        for origin, axis, moving in self.transforms:
            transform = transform @ origin
            if moving:
                axes.append(transform[:3, :3] @ axis)
                origins.append(transform[:3, 3].copy())
                turn = np.eye(4)
                turn[:3, :3] = rotation(axis, positions[index])
                transform = transform @ turn
                index += 1
        point = transform[:3, 3]
        jacobian = np.array([np.cross(axis, point - origin) for axis, origin in zip(axes, origins)]).T
        return point, jacobian

    def inverse(self, target, seed, tolerance=0.002):
        target = np.asarray(target, dtype=float)
        seed = np.asarray(seed, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            return None
        # Local IK deliberately avoids remote/random solutions and large branch jumps.
        q = np.clip(seed, self.lower, self.upper)
        for _ in range(150):
            point, jacobian = self.forward(q)
            error = target - point
            if np.linalg.norm(error) <= tolerance:
                return q
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 0.002**2 * np.eye(3), error)
            step *= min(1., 0.15 / max(np.max(np.abs(step)), 1e-12))
            improved = False
            for scale in [1., .5, .25, .1]:
                candidate = np.clip(q + scale * step, self.lower, self.upper)
                if np.linalg.norm(target - self.forward(candidate)[0]) < np.linalg.norm(error) - 1e-10:
                    q = candidate
                    improved = True
                    break
            if not improved:
                return None
        return None
