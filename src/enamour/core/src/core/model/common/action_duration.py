import rospy


# TODO: Find better name.
class ActionDuration(rospy.Duration):
    def __init__(self, secs=0, ns=0, ms=None):
        super(ActionDuration, self).__init__(secs=secs, nsecs=ns if ms is None else ms * 1000000)

    def to_ms(self):
        return self.to_nsec() / 1000000
