"""Smoke tests added in 2026: the library works for chains that are not 7 joints long."""

from pathlib import Path

import numpy as np
from loguru import logger

from arm_control.robot import Robot
from arm_control.urdf import URDF

logger.remove()
UR10 = Path(__file__).parent.parent / "urdf" / "ur10.urdf"


def make_robot():
    urdf = URDF()
    urdf.get_urdf_parameters(str(UR10))
    robot = Robot()
    robot.joint_list = urdf.joint_list[:6]  # 6 joints: the 2023 code assumed exactly 7
    return robot


def test_symbolic_fk_matches_numeric_fk():
    robot = make_robot()
    n = len(robot.joint_list)
    assert n != 7  # the case the 2023 code could not handle
    robot.jacobian_xyz(robot.forward_kine_sympy()[-1])
    q = np.random.default_rng(0).uniform(-1, 1, n)
    assert np.allclose(robot.fk_lambda(q), robot.forward_kine(q)[-1])
    assert robot.Jac(q).shape == (3, n)


def test_ik_scipy_runs_for_any_joint_count():
    robot = make_robot()
    n = len(robot.joint_list)
    robot.forward_kine_sympy()
    target = robot.fk_lambda(np.full(n, 0.3))[:3, 3]
    hist = robot.ik_scipy(np.zeros(n), target)
    assert np.linalg.norm(robot.fk_lambda(hist[-1])[:3, 3] - target) < 0.02
