from src.util.ros_node import get_ros_parameter


class Config:
    """Application config container"""

    def __init__(self, loop_rate: int):
        self.loop_rate = loop_rate


class ConfigProvider:
    """Global config provider for accessing the application config"""

    config: Config

    def __init__(self, config: Config):
        ConfigProvider.config = config

    @staticmethod
    def init_config() -> Config:
        ConfigProvider.config = Config(loop_rate=get_ros_parameter("loop_rate", 10))
        return ConfigProvider.config
