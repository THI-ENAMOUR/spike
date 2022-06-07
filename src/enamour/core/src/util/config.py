import os
import types

import rospy


class Config(object):
    """Application config containing all parametrized data"""

    loop_rate = None
    log_location = None
    hardware_connected = False

    @staticmethod
    def init_config(sys_args_dict, base_path="."):
        Config.parse_sys_args(sys_args_dict)

        Config.loop_rate = Config.get_parameter("loop_rate", 700)
        Config.log_location = Config.get_parameter("log_dir", os.path.join(base_path, "logs"))

        from util.logger import Logger

        Logger.setup_file_logger(Config.log_location)

    @staticmethod
    def parse_sys_args(sys_arg_dict):
        hardware_connected = bool(sys_arg_dict.get("hardware_connected", False))
        rospy.set_param("hardware_connected", hardware_connected)
        Config.hardware_connected = hardware_connected

    @staticmethod
    def get_parameter(name, fallback):
        """Searches for the parameter in the ROS system. if not available, returns the fallback.
        The fallback can also be a function for lazy evaluation."""
        try:
            return rospy.get_param(name)
        except KeyError:
            if isinstance(fallback, types.FunctionType):
                return fallback()
            else:
                return fallback
