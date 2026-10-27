"""Leg kinematics shared by ik.py and walking.py.

Both files import from here, so FK and IK are written once:

    TODO 1     LegKinematics: paste your four-leg FK from lab 2   (handout Part 1)
    TODO 2-4   inverse_kinematics                                 (handout Part 2)

The fr_leg_fk / fl_leg_fk / br_leg_fk / bl_leg_fk wrappers take a leg's three joint
angles as one array and return the foot position in the body frame (meters). Check
your work without the robot:

    python3 kinematics.py
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class LegKinematics:
    @staticmethod
    def rotation_x(angle):
        return np.array(
            [
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1],
            ]
        )

    @staticmethod
    def rotation_y(angle):
        return np.array(
            [
                [np.cos(angle), 0, np.sin(angle), 0],
                [0, 1, 0, 0],
                [-np.sin(angle), 0, np.cos(angle), 0],
                [0, 0, 0, 1],
            ]
        )

    @staticmethod
    def rotation_z(angle):
        return np.array(
            [
                [np.cos(angle), -np.sin(angle), 0, 0],
                [np.sin(angle), np.cos(angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

    @staticmethod
    def translation(x, y, z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1],
        ])

    @staticmethod
    def fk_front_left(theta1, theta2, theta3):
        rotation_x, rotation_y, rotation_z, translation = (
            LegKinematics.rotation_x,
            LegKinematics.rotation_y,
            LegKinematics.rotation_z,
            LegKinematics.translation,
        )

        # T_0_1 (base_link to leg_front_l_1)
        T_0_1 = translation(0.07500, 0.04450, 0) @ rotation_x(1.57080) @ rotation_z(-theta1)
        # T_1_2 (leg_front_l_1 to leg_front_l_2)
        T_1_2 = translation(0, 0, -0.039) @ rotation_y(-1.57080) @ rotation_z(theta2)
        # T_2_3 (leg_front_l_2 to leg_front_l_3)
        T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(-theta3)
        # T_3_ee (leg_front_l_3 to end-effector)
        T_3_ee = translation(.06231, -0.06216, -.018)

        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
        return T_0_ee[:3, 3]

    @staticmethod
    def fk_front_right(theta1, theta2, theta3):
        rot_x, rot_y, rot_z, trans = (
            LegKinematics.rotation_x,
            LegKinematics.rotation_y,
            LegKinematics.rotation_z,
            LegKinematics.translation,
        )

        # Each leg has 3 dof.
        # Rotations accord to sign convention; translations are physical meters on robot.
        
        # T_0_1 (base_link to leg_front_r_1)
        T_0_1 = trans(0.07500, -0.04450, 0) @ rot_x(1.57080) @ rot_z(theta1)
        # T_1_2 (leg_front_r_1 to leg_front_r_2)
        T_1_2 = trans(0, 0, 0.039) @ rot_y(-1.57080) @ rot_z(theta2)
        # T_2_3 (leg_front_r_2 to leg_front_r_3)
        T_2_3 = trans(0, -0.0494, 0.0685) @ rot_y(1.57080) @ rot_z(theta3)
        # T_3_ee (leg_front_r_3 to end-effector)
        T_3_ee = trans(.06231, -0.06216, .018)

        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
        return T_0_ee[:3, 3]

    @staticmethod
    def fk_back_left(theta1, theta2, theta3):
        rot_x, rot_y, rot_z, trans = (
            LegKinematics.rotation_x,
            LegKinematics.rotation_y,
            LegKinematics.rotation_z,
            LegKinematics.translation,
        )

        # T_0_1 (base_link to leg_back_l_1)
        T_0_1 = trans(-0.07500, 0.03350, 0) @ rot_x(1.57080) @ rot_z(-theta1)
        # T_1_2 (leg_back_l_1 to leg_back_l_2)
        T_1_2 = trans(0, 0, -0.039) @ rot_y(-1.57080) @ rot_z(theta2)
        # T_2_3 (leg_back_l_2 to leg_back_l_3)
        T_2_3 = trans(0, -0.0494, 0.0685) @ rot_y(1.57080) @ rot_z(-theta3)
        # T_3_ee (leg_back_l_3 to end-effector)
        T_3_ee = trans(.06231, -0.06216, -.018)

        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
        return T_0_ee[:3, 3]

    @staticmethod
    def fk_back_right(theta1, theta2, theta3):
        rotation_x, rotation_y, rotation_z, translation = (
            LegKinematics.rotation_x,
            LegKinematics.rotation_y,
            LegKinematics.rotation_z,
            LegKinematics.translation,
        )

        # T_0_1 (base_link to leg_back_r_1)
        T_0_1 = translation(-0.07500, -0.03350, 0) @ rotation_x(1.57080) @ rotation_z(theta1)
        # T_1_2 (leg_back_r_1 to leg_back_r_2)
        T_1_2 = translation(0, 0, 0.039) @ rotation_y(-1.57080) @ rotation_z(theta2)
        # T_2_3 (leg_back_r_2 to leg_back_r_3)
        T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)
        # T_3_ee (leg_back_r_3 to end-effector)
        T_3_ee = translation(.06231, -0.06216, .018)

        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
        return T_0_ee[:3, 3]


def fr_leg_fk(theta):
    return LegKinematics.fk_front_right(*theta)


def fl_leg_fk(theta):
    return LegKinematics.fk_front_left(*theta)


def br_leg_fk(theta):
    return LegKinematics.fk_back_right(*theta)


def bl_leg_fk(theta):
    return LegKinematics.fk_back_left(*theta)


# In joint order: front right, front left, back right, back left (see walking.yaml).
LEG_FK = [fr_leg_fk, fl_leg_fk, br_leg_fk, bl_leg_fk]


# Limits from the corrected course URDF, in FR, FL, BR, BL order.
RIGHT_LIMITS = np.array([[-1.22, 2.51], [-0.42, 3.14], [-2.79, 0.71]])
LEFT_LIMITS = np.array([[-2.51, 1.22], [-3.14, 0.42], [-0.71, 2.79]])
LEG_LIMITS = [RIGHT_LIMITS, LEFT_LIMITS, RIGHT_LIMITS, LEFT_LIMITS]


def inverse_kinematics(leg_fk, target_ee, initial_guess=(0, 0, 0),
                       learning_rate=20, max_iterations=15, tolerance=1e-5,
                       method='gauss_newton', damping=1e-4,
                       joint_limits=None, strict=False):
    """Solve foot position with damped steps and explicit failure handling.

    Pure learning callers may inspect the best iterate (strict=False). Robot
    command producers use strict=True and joint_limits; an unreachable target
    must never silently become an angle command. Tolerance is Euclidean metres.
    """
    target = np.asarray(target_ee, dtype=float)
    theta = np.asarray(initial_guess, dtype=float).copy()
    if target.shape != (3,) or theta.shape != (3,) or not np.isfinite([target, theta]).all():
        raise ValueError("target and initial guess must contain three finite values")
    if method not in ('gauss_newton', 'gradient_descent'):
        raise ValueError("unknown IK method")
    if max_iterations < 1 or tolerance <= 0 or damping <= 0 or learning_rate <= 0:
        raise ValueError("solver parameters must be positive")
    limits = None if joint_limits is None else np.asarray(joint_limits, dtype=float)
    if limits is not None:
        if limits.shape != (3, 2) or not np.isfinite(limits).all() or np.any(limits[:, 0] >= limits[:, 1]):
            raise ValueError("joint_limits must be a finite (3, 2) lower/upper array")
        theta = np.clip(theta, limits[:, 0], limits[:, 1])
    for _ in range(max_iterations):
        residual = np.asarray(leg_fk(theta)) - target
        error = float(np.linalg.norm(residual))
        if error <= tolerance:
            return theta
        eps = 1e-6
        jac = np.column_stack([(leg_fk(theta + eps*np.eye(3)[j]) -
                                leg_fk(theta - eps*np.eye(3)[j])) / (2*eps)
                               for j in range(3)])
        if method == 'gauss_newton':
            delta = np.linalg.solve(jac.T @ jac + damping*np.eye(3), jac.T @ residual)
        else:
            delta = learning_rate * 2 * jac.T @ residual
        # Bound nonlinear steps, then backtrack instead of accepting a worse pose.
        delta *= min(1.0, 0.35 / max(float(np.linalg.norm(delta)), 1e-12))
        improved = False
        for scale in (1.0, 0.5, 0.25, 0.125, 0.0625):
            candidate = theta - scale*delta
            if limits is not None:
                candidate = np.clip(candidate, limits[:, 0], limits[:, 1])
            if np.linalg.norm(leg_fk(candidate) - target) < error:
                theta, improved = candidate, True
                break
        if not improved:
            break
    error = float(np.linalg.norm(leg_fk(theta) - target))
    if strict and error > tolerance:
        raise ValueError(f"IK did not converge: foot error {1000*error:.3f} mm (limit {1000*tolerance:.3f} mm)")
    return theta


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    names = ['front right', 'front left', 'back right', 'back left']

    print('Foot positions at the zero pose (compare with lab 2):')
    for name, leg_fk in zip(names, LEG_FK):
        print(f'  {name:12s} {leg_fk(np.zeros(3))}')

    # IK round trip: pick reachable angles, ask IK to find the foot position they give.
    # A few millimeters of error or less means IK is working.
    print('IK round trip (walking.py settings, from the zero pose):')
    goal_angles = np.array([0.1, 0.4, -0.8])
    for name, leg_fk in zip(names, LEG_FK):
        target = leg_fk(goal_angles)
        theta = inverse_kinematics(leg_fk, target, max_iterations=100, tolerance=1e-4)
        error_mm = 1000 * np.linalg.norm(leg_fk(theta) - target)
        print(f'  {name:12s} foot error {error_mm:.2f} mm')
