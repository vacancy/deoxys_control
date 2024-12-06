"""
This is an experimental file where we have some standard abstractions for certain manipulation behaviors. This part will be made standard once we've tested.
"""

import time
from typing import Optional, Union
from copy import deepcopy

import numpy as np

import deoxys.utils.transform_utils as transform_utils
from deoxys.franka_interface.franka_interface import FrankaInterface
from deoxys.utils.config_utils import get_default_controller_config, verify_controller_config

__all__ = [
    'reset_joints_to', 'reset_joints_to_v1', 'reset_joints_to_v2',
    'follow_joint_traj', 'joint_interpolation_traj',
    'follow_ee_traj',
    'position_only_gripper_move_to', 'position_only_gripper_move_by'
]

def _canonicalize_gripper_open_close(gripper_open, gripper_close, default='close'):
    if gripper_open is None and gripper_close is None:
        return default == 'open', default == 'close'
    if gripper_open is not None and gripper_close is not None:
        raise ValueError("Cannot specify both gripper_open and gripper_close")
    if gripper_open is None:
        gripper_open = not gripper_close if type(gripper_close) is bool else ~gripper_close
    if gripper_close is None:
        gripper_close = not gripper_open if type(gripper_open) is bool else ~gripper_open
    return gripper_open, gripper_close


def reset_joints_to_v1(
    robot_interface: FrankaInterface,
    start_joint_pos: Union[list, np.ndarray],
    controller_cfg: Optional[dict] = None,
    timeout: int = 7,
    gripper_open: Optional[bool] = None, gripper_close: Optional[bool] = None, gripper_default: str = 'close'
):
    """
    Resets the robot's joints to the specified joint positions.

    Args:
        robot_interface (FrankaInterface): The robot interface.
        start_joint_pos (Union[list, np.ndarray]): The target joint positions.
        controller_cfg (Optional[dict], optional): The controller configuration. Defaults to None.
        timeout (int, optional): The timeout for the reset operation. Defaults to 7.
        gripper_open (Optional[bool], optional): Whether to open the gripper. Defaults to False. You should specify one of gripper_open and gripper_close.
        gripper_close (Optional[bool], optional): Whether to close the gripper. Defaults to True. You should specify one of gripper_open and gripper_close.
        gripper_default (str, optional): The default gripper action. Defaults to 'close'.

    Raises:
        ValueError: If both gripper_open and gripper_close are specified.
        AssertionError: If the controller type is not JOINT_POSITION.
        AssertionError: If the controller type is not JOINT_IMPEDANCE.
    """
    gripper_open, gripper_close = _canonicalize_gripper_open_close(gripper_open, gripper_close, default='close')

    assert type(start_joint_pos) is list or type(start_joint_pos) is np.ndarray
    if controller_cfg is None:
        controller_cfg = get_default_controller_config(controller_type="JOINT_POSITION")
    else:
        assert controller_cfg["controller_type"] == "JOINT_POSITION", (
            "This function is only for JOINT POSITION mode. You specified "
            + controller_cfg["controller_type"]
        )
        controller_cfg = verify_controller_config(controller_cfg)

    if gripper_open:
        gripper_action = -1
    else:
        gripper_action = 1
    if type(start_joint_pos) is list:
        action = start_joint_pos + [gripper_action]
    else:
        action = start_joint_pos.tolist() + [gripper_action]
    start_time = time.time()
    while True:
        if (robot_interface.received_states and robot_interface.check_nonzero_configuration()):
            if (np.max(np.abs(np.array(robot_interface.last_q) - np.array(start_joint_pos))) < 1e-3):
                break

        robot_interface.control(
            controller_type="JOINT_POSITION",
            action=action,
            controller_cfg=controller_cfg,
        )
        end_time = time.time()

        # Add timeout
        if end_time - start_time > timeout:
            break

    return True


def reset_joints_to_v2(
    robot_interface: FrankaInterface,
    desired_joint_pos: Union[list, np.ndarray],
    controller_cfg: Optional[dict] = None,
    gripper_open: Optional[bool] = None, gripper_close: Optional[bool] = None, gripper_default: str = 'close',
    timeout: float = 20, fps: int = 20, max_rad_per_second: float = np.pi / 8,
    skip_goal_reached_check: bool = False
):
    gripper_open, gripper_close = _canonicalize_gripper_open_close(gripper_open, gripper_close, default=gripper_default)

    robot_interface.wait_for_state()
    if controller_cfg is None:
        controller_cfg = get_default_controller_config(controller_type="JOINT_IMPEDANCE")
    else:
        assert controller_cfg["controller_type"] == "JOINT_IMPEDANCE", (
            "This function is only for JOINT IMPEDANCE mode. You specified "
            + controller_cfg["controller_type"]
        )
        controller_cfg = verify_controller_config(controller_cfg)

    current_joint_pos = np.asarray(robot_interface.last_q)
    desired_joint_pos = np.asarray(desired_joint_pos)

    max_diff = np.max(np.abs(desired_joint_pos - current_joint_pos))

    if max_diff / max_rad_per_second > timeout:
        raise RuntimeError(f"The desired joint position is too far away to reach in the given time. Required time: {max_diff / max_rad_per_second}, given time: {timeout}")

    num_steps = max(int(max_diff / max_rad_per_second * fps), 2)

    joint_traj = joint_interpolation_traj(
        current_joint_pos, desired_joint_pos, num_steps=num_steps
    )

    follow_joint_traj(robot_interface, joint_traj, controller_cfg=controller_cfg, gripper_close=not gripper_open)

    if np.max(np.abs(np.array(robot_interface.last_q) - np.array(desired_joint_pos))) < 1e-3:
        return True
    if not skip_goal_reached_check:
        if np.max(np.abs(np.array(robot_interface.last_q) - np.array(desired_joint_pos))) > 0.1:
            print(np.array(robot_interface.last_q), np.array(desired_joint_pos))
            raise RuntimeError("Failed to reach the desired joint position")

    reset_joints_to_v1(robot_interface, desired_joint_pos, controller_cfg=None, gripper_open=gripper_open, timeout=1.0)
    return True


reset_joints_to = reset_joints_to_v2


def joint_interpolation_traj(
    start_q: Union[list, np.ndarray], end_q: Union[list, np.ndarray], num_steps: int, traj_interpolator_type: str = "min_jerk"
):
    assert traj_interpolator_type in ["min_jerk", "linear"]

    traj = []

    if traj_interpolator_type == "min_jerk":
        for i in range(0, num_steps + 1):
            t = float(i) * (1 / num_steps)
            transformed_step_size = 10.0 * (t**3) - 15 * (t**4) + 6 * (t**5)
            traj.append(start_q + transformed_step_size * (end_q - start_q))
        traj = np.array(traj)
    elif traj_interpolator_type == "linear":
        step_size = (end_q - start_q) / float(num_steps)
        grid = np.arange(num_steps).astype(np.float64)
        traj = np.array([start_q + grid[i] * step_size for i in range(num_steps)])

        # add endpoint
        traj = np.concatenate([traj, end_q[None]], axis=0)
    return traj


def open_gripper(robot_interface: FrankaInterface, num_steps: int = 35, controller_cfg: Optional[dict] = None) -> None:
    """This is a simple function to open the gripper.

    Args:
        robot_interface (FrankaInterface): the python interface for robot control
        num_steps (int, optional): the number of steps to control. Defaults to 100.
        controller_cfg (dict, optional): controller configurations. Defaults to None.

    Returns:
        joint_pos_history (list): a list of recorded joint positions
        action_history (list): a list of recorded action commands
    """
    if controller_cfg is None:
        controller_cfg = get_default_controller_config(controller_type="JOINT_IMPEDANCE")
    else:
        assert controller_cfg["controller_type"] == "JOINT_IMPEDANCE", (
            "This function is only for JOINT IMPEDANCE mode. You specified "
            + controller_cfg["controller_type"]
        )
        controller_cfg = verify_controller_config(controller_cfg)

    current_joint_pos = np.array(robot_interface.last_q)
    action = current_joint_pos.tolist() + [-1.0]
    for i in range(num_steps):
        robot_interface.control(
            controller_type="JOINT_IMPEDANCE",
            action=action,
            controller_cfg=controller_cfg,
        )


def close_gripper(robot_interface: FrankaInterface, num_steps: int = 35, controller_cfg: Optional[dict] = None) -> None:
    """This is a simple function to close the gripper.

    Args:
        robot_interface (FrankaInterface): the python interface for robot control
        num_steps (int, optional): the number of steps to control. Defaults to 100.
        controller_cfg (dict, optional): controller configurations. Defaults to None.
    """

    if controller_cfg is None:
        controller_cfg = get_default_controller_config(controller_type="JOINT_IMPEDANCE")
    else:
        assert controller_cfg["controller_type"] == "JOINT_IMPEDANCE", (
            "This function is only for JOINT IMPEDANCE mode. You specified "
            + controller_cfg["controller_type"]
        )
        controller_cfg = verify_controller_config(controller_cfg)

    current_joint_pos = np.array(robot_interface.last_q)
    action = current_joint_pos.tolist() + [1.0]
    for i in range(num_steps):
        robot_interface.control(
            controller_type="JOINT_IMPEDANCE",
            action=action,
            controller_cfg=controller_cfg,
        )


def follow_joint_traj(
    robot_interface: FrankaInterface,
    joint_traj: list,
    num_addition_steps: int = 30,
    controller_cfg: Optional[dict] = None,
    gripper_open: Optional[bool] = None, gripper_close: Optional[bool] = None, gripper_default: str = 'close'
):
    """This is a simple function to follow a given trajectory in joint space.

    Args:
        robot_interface (FrankaInterface): the python interface for robot control
        joint_traj (list): the joint trajectory to follow
        num_addition_steps (int, optional): the number of steps to add to the end of the trajectory. Defaults to 100.
        controller_cfg (dict, optional): controller configurations. Defaults to None.
        gripper_open (bool, optional): whether to open the gripper. Defaults to False. You should specify one of gripper_open and gripper_close.
        gripper_close (bool, optional): whether to close the gripper. Defaults to True. You should specify one of gripper_open and gripper_close.
        gripper_default (str, optional): the default gripper action. Defaults to 'close'.

    Returns:
        joint_pos_history (list): a list of recorded joint positions
        action_history (list): a list of recorded action commands
    """
    gripper_open, gripper_close = _canonicalize_gripper_open_close(gripper_open, gripper_close, default=gripper_default)

    if controller_cfg is None:
        controller_cfg = get_default_controller_config(
            controller_type="JOINT_IMPEDANCE"
        )
    else:
        assert controller_cfg["controller_type"] == "JOINT_IMPEDANCE", (
            "This function is only for JOINT IMPEDANCE mode. You specified "
            + controller_cfg["controller_type"]
        )
        controller_cfg = verify_controller_config(controller_cfg)

    # controller_cfg['enable_residual_tau'] = True
    # controller_cfg['residual_tau_translation_vec'] = [0, 0, -1.5]

    prev_action = np.array([0.0] * 8)
    joint_pos_history = []
    action_history = []

    assert (
        robot_interface.last_q is not None
        and robot_interface.check_nonzero_configuration()
    )

    if type(gripper_close) is not bool:
        gripper_close_list = gripper_close
    else:
        gripper_close_list = [gripper_close for _ in joint_traj]

    for target_joint_pos, gripper_close in zip(joint_traj, gripper_close_list):
        assert len(target_joint_pos) >= 7
        if type(target_joint_pos) is np.ndarray:
            action = target_joint_pos.tolist()
        else:
            action = target_joint_pos
        if len(action) == 7:
            if gripper_open:
                action = action + [-1.0]
            else:
                action = action + [1.0]
        current_joint_pos = np.array(robot_interface.last_q)
        robot_interface.control(
            controller_type="JOINT_IMPEDANCE",
            action=action,
            controller_cfg=controller_cfg,
        )
        joint_pos_history.append(current_joint_pos.flatten().tolist())
        action_history.append(prev_action.tolist())
        prev_action = np.array(action)

    for i in range(num_addition_steps):
        current_joint_pos = np.array(robot_interface.last_q)
        robot_interface.control(
            controller_type="JOINT_IMPEDANCE",
            action=action,
            controller_cfg=controller_cfg,
        )
        joint_pos_history.append(current_joint_pos.flatten().tolist())
        action_history.append(prev_action.tolist())
        prev_action = np.array(action)

    return joint_pos_history, action_history


def follow_ee_traj(
    robot_interface: FrankaInterface,
    ee_traj: list[tuple[np.ndarray, np.ndarray] | tuple[np.ndarray, np.ndarray, float]],
    compliance_traj: Optional[list] = None, *,
    follow_position_only: bool = False,
    controller_cfg: Optional[dict] = None,
    gripper_open: Optional[bool] = None, gripper_close: Optional[bool] = None, gripper_default: str = 'close'
) -> tuple[list, list]:
    """This is a simple function to follow a given trajectory in end-effector space.

    Args:
        robot_interface: the robot interface.
        ee_traj: the end-effector trajectory to follow.
        compliance_traj: the compliance trajectory to follow. Each entry is a list of 6 values [d_x, d_y, d_z, d_rx, d_ry, d_rz].
        follow_position_only: whether to follow the position only. Defaults to False.
        controller_cfg: the controller configuration.
        gripper_open: whether to open the gripper. Defaults to False. You should specify one of gripper_open and gripper_close.
        gripper_close: whether to close the gripper. Defaults to True. You should specify one of gripper_open and gripper_close.
        gripper_default: the default gripper action. Defaults to 'close'.

    Returns:
        ee_pose_history: a list of recorded end effector poses.
        action_history: a list of recorded action commands.

    Raises:
        AssertionError: if the controller type is not OSC_POSE.
    """
    gripper_open, gripper_close = _canonicalize_gripper_open_close(gripper_open, gripper_close, default=gripper_default)

    if controller_cfg is None:
        controller_cfg = get_default_controller_config(controller_type="OSC_POSE")
    else:
        assert controller_cfg["controller_type"] == "OSC_POSE", (
            "This function is only for OSC_POSE mode. You specified "
            + controller_cfg["controller_type"]
        )
        controller_cfg = verify_controller_config(controller_cfg)

    controller_cfg['is_delta'] = False

    if compliance_traj is None:
        assert len(ee_traj) == len(compliance_traj)

    robot_interface.wait_for_state()
    current_ee_pose = robot_interface.last_eef_pose
    current_ee_rot = current_ee_pose[:3, :3]
    current_ee_quat = transform_utils.mat2quat(current_ee_rot)

    ee_pose_history = []
    action_history = []

    for i, ee_pose in enumerate(ee_traj):
        target_pos, target_rot = ee_pose[:2]
        if follow_position_only:
            target_rot = current_ee_quat
        target_axis_angle = transform_utils.quat2axisangle(target_rot)

        action = np.concatenate([target_pos, target_axis_angle])

        if len(ee_pose) == 3:
            action = np.concatenate([action, [ee_pose[2]]])
        else:
            if gripper_open:
                action = np.concatenate([action, [-1.0]])
            else:
                action = np.concatenate([action, [1.0]])

        if compliance_traj is not None:
            controller_cfg = deepcopy(controller_cfg)
            controller_cfg["Kp"]['translation'] = compliance_traj[i][:3]
            controller_cfg["Kp"]['rotation'] = compliance_traj[i][3:]

        ee_pose_history.append(robot_interface.last_eef_pose.flatten().tolist())
        action_history.append(action.tolist())

        robot_interface.control(
            controller_type="OSC_POSE",
            action=action,
            controller_cfg=controller_cfg,
        )

    return ee_pose_history, action_history


def position_only_gripper_move_to(
    robot_interface, target_pos, num_steps=100, controller_cfg: dict = None, grasp=False
):
    """_summary_

    Args:
        robot_interface (FrankaInterface): the python interface for robot control
        target_pos (np.array or list): target xyz location
        num_steps (int, optional): number of steps to control. Defaults to 100.
        controller_cfg (dict, optional): controller configurations. Defaults to None.
        grasp (bool, optional): close the gripper if set to True. Defaults to False.
    Return:
        eef_pos_history (list): a list of recorded end effector positions
        action_history (list): a list of recorded action commands
    """
    if controller_cfg is None:
        controller_cfg = get_default_controller_config(controller_type="OSC_POSITION")
    else:
        assert controller_cfg["controller_type"] == "OSC_POSITION", (
            "This function is only for OSC_POSITION mode. You specified "
            + controller_cfg["controller_type"]
        )
        controller_cfg = verify_controller_config(controller_cfg)
    eef_pos_history = []
    action_history = []

    current_pos = None
    while current_pos is None:
        _, current_pos = robot_interface.last_eef_rot_and_pos

    prev_action = np.array([0.0] * 6 + [int(grasp) * 2 - 1])
    for i in range(num_steps):
        _, current_pos = robot_interface.last_eef_rot_and_pos
        action = np.array([0.0] * 6 + [int(grasp) * 2 - 1])
        action[:3] = (target_pos - current_pos).flatten() * 10
        robot_interface.control(
            controller_type="OSC_POSITION", action=action, controller_cfg=controller_cfg
        )
        eef_pos_history.append(current_pos.flatten().tolist())
        action_history.append(prev_action)
        prev_action = np.array(action)
    return eef_pos_history, action_history


def position_only_gripper_move_by(
    robot_interface, delta_pos, num_steps=100, controller_cfg: dict = None, grasp=True
):
    """_summary_

    Args:
        robot_interface (FrankaInterface): the python interface for robot control
        target_pos (np.array or list): target xyz location
        num_steps (int, optional): number of steps to control. Defaults to 100.
        controller_cfg (dict, optional): controller configurations. Defaults to None.
        grasp (bool, optional): close the gripper if set to True. Defaults to False.
    Return:
        eef_pos_history (list): a list of recorded end effector positions
        action_history (list): a list of recorded action commands
    """
    current_pos = None
    while current_pos is None:
        _, current_pos = robot_interface.last_eef_rot_and_pos

    delta_pos = np.array(delta_pos).reshape(3, 1)
    assert delta_pos.shape == current_pos.shape
    target_pos = current_pos + delta_pos
    return position_only_gripper_move_to(
        robot_interface,
        target_pos,
        num_steps=num_steps,
        controller_cfg=controller_cfg,
        grasp=grasp,
    )
