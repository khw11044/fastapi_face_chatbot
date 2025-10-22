from typing import Annotated

from langchain_core.tools import tool

# ros2_service의 싱글톤 퍼블리셔 import
from app.services.ros2_service import ros2_publisher

@tool
def reset_ears() -> str:
    """
    Resets both ears to their default position (0.0, 0.0).

    Returns:
        A confirmation string.
    """
    if not ros2_publisher.initialized:
        ros2_publisher.initialize()
    success = ros2_publisher.publish_ear_position(0.0, 0.0)
    if success:
        return "Ears reset to default position (0.0, 0.0)."
    else:
        return "귀 위치 퍼블리시 실패: ROS2 퍼블리셔가 초기화되지 않았거나 오류가 발생했습니다."

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
    if not ros2_publisher.initialized:
        ros2_publisher.initialize()
    success = ros2_publisher.publish_ear_position(left_pos, right_pos)
    if success:
        return f"Set ear positions: left={left_pos}, right={right_pos}"
    else:
        return "귀 위치 퍼블리시 실패: ROS2 퍼블리셔가 초기화되지 않았거나 오류가 발생했습니다."

# 다리(leg) 관련 함수는 기존 subprocess 방식 유지 (별도 리팩터링 필요시 적용)
import math
import ast
import statistics
from typing import List, Tuple, Union

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

    import subprocess
    try:
        output = subprocess.check_output(command, shell=True).decode()
        return True, output
    except Exception as e:
        return False, str(e)

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
