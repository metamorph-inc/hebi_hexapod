from geometry_msgs.msg import Twist
from smach import State


def check_if_twist_msg_is_zero(twist_msg, linear_threshold, angular_threshold):
    assert isinstance(twist_msg, Twist)
    if abs(twist_msg.linear.x) > linear_threshold:
        return False
    elif abs(twist_msg.linear.y) > linear_threshold:
        return False
    elif abs(twist_msg.linear.z) > linear_threshold:
        return False
    elif abs(twist_msg.angular.x) > angular_threshold:
        return False
    elif abs(twist_msg.angular.y) > angular_threshold:
        return False
    elif abs(twist_msg.angular.z) > angular_threshold:
        return False
    else:
        return True


class CheckIfTwistMsgIsZero(State):
    def __init__(self, lin_threshold, ang_threshold):
        """SMACH State
        :type lin_threshold: float
        :param lin_threshold: Minimum absolute value for twist.linear.(x,y,z) value to be considered non-zero

        :type ang_threshold: float
        :param ang_threshold: Minimum absolute value for twist.angular.(x,y,z) value to be considered non-zero
        """
        State.__init__(self, outcomes=['true', 'false'], input_keys=['twist_msg'])

        self.lin_threshold = lin_threshold
        self.ang_threshold = ang_threshold

        self.active = False

    def enter(self, ud):
        self.active = True

    def execute(self, ud):
        self.enter(ud)

        twist_msg = ud.twist_msg
        if check_if_twist_msg_is_zero(twist_msg, self.lin_threshold, self.ang_threshold):
            return self.exit(ud, 'true')
        else:
            return self.exit(ud, 'false')

    def exit(self, ud, outcome):
        self.active = False
        return outcome
