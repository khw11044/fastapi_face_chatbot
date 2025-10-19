
import math
import ast
import statistics
from typing import List, Tuple, Union
from typing import Annotated

from langchain_core.tools import tool



@tool
def reset_ears() -> str:
    """
    Resets both ears to their default position (0.0, 0.0).

    Returns:
        A confirmation string.
    """
    return "Ears reset to default position (0.0, 0.0)."

@tool
def action_ears(
    left_pos: Annotated[float, "Left ear position in range [0.0 ~ 1.047]"] = 1.0,
    right_pos: Annotated[float, "Right ear position in range [0.0 ~ 1.047]"] = 1.0,
) -> str:
    """
    Sets the position of the left and right ears.
    
    Args:
        left_pos: Position of the left ear in the range [0.0 ~ 1.047]
        right_pos: Position of the right ear in the range [0.0 ~ 1.047]

    Returns:
        A confirmation string of ear positions.
    """
    
    left_pos = float(left_pos)
    right_pos = float(right_pos)
    
    
    return f"Set ear positions: left={left_pos}, right={right_pos}"

@tool
def reset_legs() -> str:
    """
    Resets both legs to their default position (0.0, 0.0).

    Returns:
        A confirmation string.
    """

    return "Legs reset to default position (0.0, 0.0)."


@tool
def action_legs(
    left_pos: Annotated[float, "Left leg position [0.0, 350.0, 700.0]"] = 0.0,
    right_pos: Annotated[float, "Right leg position [0.0, 350.0, 700.0]"] = 0.0,
) -> str:
    """
    Sets the position of the left and right legs.
    
    A value of 0.0 indicates the default leg position, while 700.0 represents a fully raised leg.

    Args:
        left_pos: Left leg position [0.0, 350.0, 700.0]
        right_pos: Right leg position [0.0, 350.0, 700.0]

    Returns:
        A confirmation string of leg positions.
    """
    
    left_pos = float(left_pos)
    right_pos = float(right_pos)

    return f"Set leg positions: left={left_pos}, right={right_pos}"


