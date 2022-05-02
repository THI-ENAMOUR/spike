import rospy


class Logger:
    def __init__(self, name):
        self.name = name

    def info(self, message):
        rospy.loginfo(f"\n[{self.name}]: \n{message}")

    def warning(self, message):
        rospy.logwarn(f"\n[{self.name}]: \n{message}")

    def error(self, message):
        rospy.logerr(f"\n[{self.name}]: \n{message}")

    def fatal(self, message):
        rospy.logfatal(f"\n[{self.name}]: \n{message}")

    def debug(self, message):
        rospy.logdebug(f"\n[{self.name}]: \n{message}")
