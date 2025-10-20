
import math
import ast
import statistics
from typing import List, Tuple, Union
from typing import Annotated

from langchain_core.tools import tool
import subprocess


def execute_ros_command(command: str) -> Tuple[bool, str]:
    """
    Execute a ROS2 command.

    :param command: The ROS2 command to execute.
    :return: A tuple containing a boolean indicating success and the output of the command.
    """

    # Validate the command is a proper ROS2 command
    cmd = command.split(" ")
    valid_ros2_commands = ["node", "topic", "service", "action", "param", "doctor"]

    if len(cmd) < 2:
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[0] != "ros2":
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[1] not in valid_ros2_commands:
        raise ValueError(f"'ros2 {cmd[1]}' is not a valid ros2 subcommand.")

    try:
        output = subprocess.check_output(command, shell=True).decode()
        return True, output
    except Exception as e:
        return False, str(e)

@tool
def reset_ears() -> str:
    """
    Resets both ears to their default position (0.0, 0.0).

    Returns:
        A confirmation string.
    """

    
    cmd = f"ros2 topic pub --once /edie8_r_ear_position_controller/commands std_msgs/msg/Float64MultiArray 'data: {[0.0]}'"
    
    success, output = execute_ros_command(cmd)
    
    if not success:
        return [output]
    
    cmd = f"ros2 topic pub --once /edie8_l_ear_position_controller/commands std_msgs/msg/Float64MultiArray 'data: {[0.0]}'"
    
    success, output = execute_ros_command(cmd)
    
    if not success:
        return [output]
    
    
    return "Ears reset to default position (0.0, 0.0)."

@tool
def action_ears(
    left_pos: Annotated[float, "Left ear position in range [0.0 ~ 0.9]"] = 0.9,
    right_pos: Annotated[float, "Right ear position in range [0.0 ~ 0.9]"] = 0.9,
) -> str:
    """
    Sets the position of the left and right ears.
    
    Args:
        left_pos: Position of the left ear in the range [0.0 ~ 0.9]
        right_pos: Position of the right ear in the range [0.0 ~ 0.9]

    Returns:
        A confirmation string of ear positions.
    """
    
    left_pos = float(left_pos)
    right_pos = float(right_pos)
    
    left_pos = max(0, min(0.9, left_pos))
    right_pos = max(0, min(0.9, right_pos))
    
    cmd = f"ros2 topic pub --once /edie8_r_ear_position_controller/commands std_msgs/msg/Float64MultiArray 'data: {[right_pos]}'"
    
    success, output = execute_ros_command(cmd)
    
    if not success:
        return [output]
    
    cmd = f"ros2 topic pub --once /edie8_l_ear_position_controller/commands std_msgs/msg/Float64MultiArray 'data: {[left_pos]}'"
    
    success, output = execute_ros_command(cmd)
    
    if not success:
        return [output]
    
    
    return f"Set ear positions: left={left_pos}, right={right_pos}"

@tool
def reset_legs() -> str:
    """
    Resets both legs to their default position (0.0, 0.0).

    Returns:
        A confirmation string.
    """
    cmd = (
        "ros2 action send_goal /edie8_leg_trajectory_controller/command "
        "edie8_msgs/action/EdieMotionTrajectory "
        "'position: [0, 0]\ntraj_type: \"\"\ntime: 0.6'"
    )
    success, output = execute_ros_command(cmd)
    if not success:
        return f"Leg reset failed: {output}"
    return "Legs reset to default position (0, 0)."



@tool
def action_legs(
    left_pos: Annotated[int, "Left leg position [0 ~ 1000]"] = 0.0,
    right_pos: Annotated[int, "Right leg position [0 ~ 1000]"] = 0.0,
) -> str:
    """
    Sets the position of the left and right legs.

    Args:
        left_pos: Left leg position [0 ~ 1000]
        right_pos: Right leg position [0 ~ 1000]

    Returns:
        A confirmation string of leg positions.
    """
    left_pos = int(left_pos)
    right_pos = int(right_pos)
    # 값 범위 제한
    left_pos = max(0, min(1000, left_pos))
    right_pos = max(0, min(1000, right_pos))

    cmd = (
        "ros2 action send_goal /edie8_leg_trajectory_controller/command "
        "edie8_msgs/action/EdieMotionTrajectory "
        f"'position: [{left_pos}, {right_pos}]\ntraj_type: \"\"\ntime: 0.6'"
    )
    success, output = execute_ros_command(cmd)
    if not success:
        return f"Leg action failed: {output}"
    return f"Set leg positions: left={left_pos}, right={right_pos}"

