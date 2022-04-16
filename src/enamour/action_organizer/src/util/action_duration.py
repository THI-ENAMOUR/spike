import rospy


# TODO: Find better name.


class ActionDuration(rospy.Duration):
    def __init__(self, secs=0, ns=0, ms=None):
        if ms is None:
            super(ActionDuration, self).__init__(secs=secs, nsecs=ns)
        else:
            super(ActionDuration, self).__init__(secs=secs, nsecs=ms * 1000000)

    def to_ms(self):
        return self.to_nsec() / 1000000
