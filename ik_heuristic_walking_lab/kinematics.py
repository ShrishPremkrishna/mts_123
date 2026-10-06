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


def inverse_kinematics(leg_fk, target_ee, initial_guess=(0, 0, 0),
                       learning_rate=20, max_iterations=100, tolerance=1e-5):
    """Joint angles that put leg_fk's foot at target_ee, found by gradient descent.

    leg_fk is one of the FK functions above, so the same solver works for every leg.
    """
    ################################################################################################
    # TODO 4: Set default values for learning_rate, max_iterations, and tolerance in the
    # signature above. Tolerance is in meters. walking.py overrides max_iterations and tolerance.
    ################################################################################################

    def cost_function(theta):
        # Compute the cost function and the L1 error vector
        # return the cost (a scalar) and l1 (a vector of size 3)
        ################################################################################################
        p = leg_fk(theta)
        p_end = target_ee
        l1 = p - p_end
        cost = np.sum((l1) ** 2)
        return cost, l1
        ################################################################################################

    def gradient(theta, epsilon=1e-4):
        # Compute the gradient of the cost function using finite differences
        ###############################################################################################
        grad =np.zeros(len(theta))
        for j in range(len(theta)):
            n =np.zeros(len(theta))
            n[j] = epsilon
            plus, error =cost_function(theta +n)
            minus, merror =cost_function(theta -n)
            grad[j] =(plus - minus) / (2 * epsilon)
        return grad

        # diff = lambda ei: (cost_function(theta + epsilon * ei)[0] - cost_function(theta - epsilon * ei)[0]) / (2 * epsilon)
        # return np.array([diff(ei) for ei in np.eye(len(theta))])
        ################################################################################################

    theta = np.array(initial_guess).astype(np.float64)

    cost_l = []
    diff_l = []
    for i in range(max_iterations):
        grad = gradient(theta)

        # Update the theta (parameters) using the gradient and the learning rate
        ################################################################################################

        cost, l1 = cost_function(theta)
        cost_l.append(cost.item())
        diff_l.append((cost_l[-1] if len(cost_l) else 0) - cost.item())
        if np.mean(np.abs(l1)) < tolerance:
            break  # IK Converged
        theta = theta - learning_rate * grad
        # print(f'Grads: {grad}')
        
        # TODO (BONUS): Implement the (quasi-)Newton's method instead of finite differences for faster
        # convergence
        ################################################################################################
        
    # print(cost_l[-1])
    # if cost_l[-1] > .0050:
    #     print(f'Cost: {cost_l}') # If strictly decreasing, should be all negative

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
