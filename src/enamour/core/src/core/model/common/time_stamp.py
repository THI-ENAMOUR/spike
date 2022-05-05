from typing import Optional

import rospy


class TimeStamp(rospy.Duration):
    """Represents a relative point in time, e.g. 5 sec, 300ms and 4029ns"""

    def __init__(self, secs=0, ns=0, ms: Optional[int] = None):
        super(TimeStamp, self).__init__(secs=secs, nsecs=ns if ms is None else ms * 1000000)

    def to_ms(self) -> int:
        return self.to_nsec() / 1000000
