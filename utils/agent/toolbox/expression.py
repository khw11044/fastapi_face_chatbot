from typing import Optional

from langchain_core.tools import tool

# ros2_service의 싱글톤 퍼블리셔 import
from app.services.ros2_service import ros2_publisher

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
    # 퍼블리셔 초기화가 안 되어 있으면 자동 초기화
    if not ros2_publisher.initialized:
        ros2_publisher.initialize()
    success = ros2_publisher.publish_emotion_action(action_index)
    if success:
        return f"에디가 자신의 생각과 감정을 {emotions[action_index]}으로 표현하였습니다."
    else:
        return "감정 퍼블리시 실패: ROS2 퍼블리셔가 초기화되지 않았거나 오류가 발생했습니다."
