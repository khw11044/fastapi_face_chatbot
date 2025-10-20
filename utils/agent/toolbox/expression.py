
from typing import List, Union, Dict, Any, Optional, Tuple

from langchain_core.tools import tool
import time
import subprocess


emotions = {
        1: "궁금함",    
        2: "졸림",      
        3: "웃김(즐거움)",       
        4: "기쁨(뿌듯함)",     
        5: "슬픔",       
        6: "놀람",       
        7: "매우놀람",     
        8: "실망",        
        9: "사랑",      
        10: "어지러움",        
        11: "아주 어지러움",        
        # "low-dattery": 12,
    }


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
def call_expression_action(action_index: Optional[int] = 1) -> str:
    """
    Publishes an emotional action command to the Edie robot.

    This tool triggers the Edie robot's emotional response by publishing an `action_index`
    value to the ROS2 topic `/edie8/emotion/action_index`.  
    Each index corresponds to a predefined emotional state of the robot.

    Emotion Mapping:
        1: "Curiosity"  
        2: "Sleepiness"  
        3: "Amusement (Joy)"  
        4: "Happiness (Pride)"  
        5: "Sadness"  
        6: "Surprise"  
        7: "Great Surprise"  
        8: "Disappointment"  
        9: "Love"  
        10: "Dizziness"  
        11: "Severe Dizziness"  

    Args:
        action_index (Optional[int], default=1):  
            The index number representing the desired emotion.  
            Must be an integer between 1 and 11.  
            Example: `action_index=4` → triggers the "Happiness (Pride)" emotion.

    Returns:
        str:  
            A short text message describing the emotion expressed by Edie.  
            Example: `"에디가 자신의 생각과 감정을 행복으로 표현하였습니다."`

    Usage:
        Use this tool when you want Edie to display a specific emotion.
        Choose the corresponding number from the emotion mapping list above.
    """
    
    cmd = f"ros2 topic pub -1 /edie8/emotion/action_index std_msgs/msg/UInt8 'data: {action_index}'"
    
    success, output = execute_ros_command(cmd)
    
    if not success:
        return [output]
    
    # # time.sleep(0.1)
    # cmd = "ros2 topic pub -1 /edie8/emotion/motion_done std_msgs/msg/Bool 'data: true'"
    # success, output = execute_ros_command(cmd)
    # cmd = "ros2 topic pub -1 /edie8/emotion/display_done std_msgs/msg/Bool 'data: true'"
    # success, output = execute_ros_command(cmd)
    # cmd = "ros2 topic pub -1 /edie8/emotion/sound_done std_msgs/msg/Bool 'data: true'"
    # success, output = execute_ros_command(cmd)
    
    return f"에디가 자신의 생각과 감정을 {emotions[action_index]}으로 표현하였습니다."

