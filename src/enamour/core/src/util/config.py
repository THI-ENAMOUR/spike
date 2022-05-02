from core.event_bus import event_bus


class Config:
    """Application config"""

    loop_rate: int

    def __init__(self, loop_rate: int):
        Config.loop_rate = loop_rate

    @staticmethod
    def init_config():
        Config.loop_rate = event_bus.get_parameter("loop_rate", 10)
