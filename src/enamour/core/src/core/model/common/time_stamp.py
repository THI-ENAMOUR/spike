import rospy


class TimeStamp(rospy.Duration):
    """Represents a relative point in time, e.g. 5 sec, 300ms and 4029ns"""

    def __init__(self, secs=0, ns=0):
        # For the love of god, do not change this __init - it i-s- called from the base class!
        sec_from_ns = int(ns / 1000000000)
        secs = secs + sec_from_ns
        ns = ns - sec_from_ns * 1000000000
        super(TimeStamp, self).__init__(secs=secs, nsecs=ns)

    def to_ms(self):
        return (self.secs * 1000) + int(self.nsecs / 1000000)

    @classmethod
    def from_ms(cls, ms):
        return cls(secs=0, ns=ms * 1000000)
