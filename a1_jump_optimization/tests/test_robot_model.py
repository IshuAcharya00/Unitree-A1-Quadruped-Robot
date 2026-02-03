import numpy as np

from a1_jump_optimization.robot_model import UnitreeA1


def test_robot_model_loads_config():
    robot = UnitreeA1()

    assert robot.mass > 0
    assert robot.leg_length > 0
    assert robot.gravity > 0
    assert robot.n_legs == 4


def test_kinematics_and_torque():
    robot = UnitreeA1()

    body_pos = np.array([0.0, robot.initial_height])
    body_angle = 0.0
    leg_extensions = np.array([0.5, 0.5, 0.5, 0.5])

    foot_pos = robot.get_foot_position(body_pos, body_angle, leg_extensions)
    assert foot_pos.shape == (4, 2)

    assert robot.is_kinematically_feasible(robot.initial_height, leg_extensions)

    ground_force = np.array([50.0, 100.0])
    torque = robot.compute_joint_torques_from_force(ground_force, 0.5)
    assert torque > 0
    assert torque <= robot.max_torque
