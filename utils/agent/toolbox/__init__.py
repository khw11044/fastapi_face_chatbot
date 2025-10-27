from typing import Literal, List, Optional
from langchain.agents import Tool



class ToolBox:
    def __init__(
        self
    ):
        self.__tools: list = []

        # Add the default tools
        # from . import calculation as calculation_tool
        # from . import action as action_tool
        # from . import expression as expression_tool
        
        # self.__iterative_add(calculation_tool)
        # self.__iterative_add(action_tool)
        # self.__iterative_add(expression_tool)

  
    def get_tools(self) -> List[Tool]:
        return self.__tools
    
    def get_tool(self, tool_name: str) -> Optional[Tool]:
        """
        특정 이름의 도구를 반환합니다.
        
        :param tool_name: 찾을 도구의 이름
        :return: 도구 객체 또는 None
        """
        for tool in self.__tools:
            if tool.name == tool_name:
                return tool
        return None

    def __add_tool(self, tool):
        if hasattr(tool, "name") and hasattr(tool, "func"):
            self.__tools.append(tool)

    def __iterative_add(self, package):
        """
        Iterate through a package and add each @tool to the tools list.

        :param package: The package to iterate through.
        :param blacklist: A parameter used by some tools to filter out certain results.
        """
        for tool_name in dir(package):
            if not tool_name.startswith("_"):
                t = getattr(package, tool_name)
                self.__add_tool(t)

    def add_packages(self, tool_packages: List):
        """
        Add a list of tools to the Tools object by iterating through each package.

        :param tool_packages: A list of tool packages to add to the Tools object.
        """
        for pkg in tool_packages:
            self.__iterative_add(pkg)

    def add_tools(self, tools: list):
        """
        Add a single tool to the Tools object.

        :param tools: A list of tools to add
        """
        for tool in tools:
            self.__add_tool(tool)
