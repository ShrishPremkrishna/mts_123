"""Leg kinematics shared by ik.py and walking.py.

Both files import from here, so FK and IK are written once:

    TODO 1     LegKinematics: paste your four-leg FK from lab 2   (handout Part 1)
    TODO 2-4   inverse_kinematics                                 (handout Part 2)

The fr_leg_fk / fl_leg_fk / br_leg_fk / bl_leg_fk wrappers take a leg's three joint
angles as one array and return the foot position in the body frame (meters). Check
your work without the robot:

    python3 kinematics.py
"""

import numpy as np


class LegKinematics:
    ################################################################################################
    # TODO 1: Paste your rotation_x, rotation_y, rotation_z, translation, fk_front_left,
    # fk_front_right, fk_back_left, and fk_back_right methods from lab 2's
    # forward_kinematics.py over the stubs below. The names and arguments match lab 2, so they
    # should paste in unchanged.
    ################################################################################################

    def rotation_x(self, angle):
        raise NotImplementedError()

    def rotation_y(self, angle):
        raise NotImplementedError()

    def rotation_z(self, angle):
        raise NotImplementedError()

    def translation(self, x, y, z):
        raise NotImplementedError()

    def fk_front_left(self, theta1, theta2, theta3):
        raise NotImplementedError()

    def fk_front_right(self, theta1, theta2, theta3):
        raise NotImplementedError()

    def fk_back_left(self, theta1, theta2, theta3):
        raise NotImplementedError()

    def fk_back_right(self, theta1, theta2, theta3):
        raise NotImplementedError()


_legs = LegKinematics()


def fr_leg_fk(theta):
    return _legs.fk_front_right(*theta)


def fl_leg_fk(theta):
    return _legs.fk_front_left(*theta)


def br_leg_fk(theta):
    return _legs.fk_back_right(*theta)


def bl_leg_fk(theta):
    return _legs.fk_back_left(*theta)


# In joint order: front right, front left, back right, back left (see walking.yaml).
LEG_FK = [fr_leg_fk, fl_leg_fk, br_leg_fk, bl_leg_fk]


def inverse_kinematics(leg_fk, target_ee, initial_guess=(0, 0, 0),
                       learning_rate=None, max_iterations=None, tolerance=None):
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
        # TODO 2: Implement the cost function using leg_fk
        ################################################################################################
        return None, None

    def gradient(theta, epsilon=1e-3):
        # Compute the gradient of the cost function using finite differences
        ################################################################################################
        # TODO 3: Implement the gradient computation
        ################################################################################################
        return

    theta = np.array(initial_guess).astype(np.float64)

    cost_l = []
    for _ in range(max_iterations):
        grad = gradient(theta)

        # Update the theta (parameters) using the gradient and the learning rate
        ################################################################################################
        # TODO 4: Implement the gradient update. Use the cost function you implemented, and use tolerance
        # to determine if IK has converged
        # TODO (BONUS): Implement the (quasi-)Newton's method instead of finite differences for faster
        # convergence
        ################################################################################################

    # print(f'Cost: {cost_l}') # Use to debug to see if your cost function converges within max_iterations

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
