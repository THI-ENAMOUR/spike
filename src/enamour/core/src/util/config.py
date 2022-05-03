import os
import types
from typing import Any

import rospy


class Config:
    """Application config containing all parametrized data"""

    loop_rate: int
    log_location: str

    def __init__(self, loop_rate: int):
        Config.loop_rate = loop_rate

    @staticmethod
    def init_config(base_path: str):
        Config.loop_rate = Config.get_parameter("loop_rate", 10)
        Config.log_location = Config.get_parameter("log_dir", os.path.join(base_path, "logs"))

    @staticmethod
    def get_parameter(name: str, fallback) -> Any:
        """Searches for the parameter in the ROS system. if not available, returns the fallback.
        The fallback can also be a function for lazy evaluation."""
        try:
            return rospy.get_param(name)
        except KeyError:
            if isinstance(fallback, types.FunctionType):
                return fallback()
            else:
                return fallback
