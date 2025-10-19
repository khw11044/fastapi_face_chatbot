"""
기본 계산 도구들
"""

from langchain_core.tools import tool
from typing import Annotated
import math


@tool
def add(a: Annotated[float, "첫 번째 숫자"], b: Annotated[float, "두 번째 숫자"]) -> float:
    """두 숫자를 더합니다."""
    return float(a) + float(b)


@tool  
def subtract(a: Annotated[float, "첫 번째 숫자"], b: Annotated[float, "두 번째 숫자"]) -> float:
    """첫 번째 숫자에서 두 번째 숫자를 뺍니다."""
    return float(a) - float(b)


@tool
def multiply(a: Annotated[float, "첫 번째 숫자"], b: Annotated[float, "두 번째 숫자"]) -> float:
    """두 숫자를 곱합니다."""
    return float(a) * float(b)


@tool
def divide(a: Annotated[float, "나누어질 숫자"], b: Annotated[float, "나누는 숫자"]) -> float:
    """첫 번째 숫자를 두 번째 숫자로 나눕니다."""
    if float(b) == 0:
        raise ValueError("0으로 나눌 수 없습니다.")
    return float(a) / float(b)


@tool
def power(base: Annotated[float, "밑"], exponent: Annotated[float, "지수"]) -> float:
    """주어진 밑을 지수만큼 거듭제곱합니다."""
    return math.pow(float(base), float(exponent))


@tool  
def square_root(number: Annotated[float, "제곱근을 구할 숫자"]) -> float:
    """숫자의 제곱근을 구합니다."""
    if float(number) < 0:
        raise ValueError("음수의 제곱근을 구할 수 없습니다.")
    return math.sqrt(float(number))
