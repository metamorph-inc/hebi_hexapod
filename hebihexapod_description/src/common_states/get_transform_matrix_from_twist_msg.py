from geometry_msgs.msg import Twist
from smach import State

import sys
sys.path.append("..")
from common import transformations as transforms


def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n


class GetTransformMatrixFromTwistMsg(State):
    def __init__(self, lin_displace_lmt, ang_displace_lmt, time_step):
        """SMACH State
        :type lin_displace_lmt: float
        :param lin_displace_lmt: Max move_base linear displacement [m]

        :type ang_displace_lmt: float
        :param ang_displace_lmt: Max move_base_angular displacement [rad]

        :type time_step: float
        :param time_step: Estimated time step for which to calculate displacement [s]
        """
        State.__init__(self, outcomes=['done'], input_keys=['twist_msg'], output_keys=['transform_matrix'])

        self.lin_displace_lmt = lin_displace_lmt
        self.ang_displace_lmt = ang_displace_lmt
        self.time_step = time_step

        self.active = False

    def enter(self, ud):
        self.active = True

    def execute(self, ud):
        self.enter(ud)

        # process twist_msg
        twist_msg = ud.twist_msg
        twist = Twist()

        # Get estimated displacement over time step and apply limits
        twist.linear.x = clamp(twist_msg.linear.x * self.time_step, -self.lin_displace_lmt, self.lin_displace_lmt)
        twist.linear.y = clamp(twist_msg.linear.y * self.time_step, -self.lin_displace_lmt, self.lin_displace_lmt)
        twist.linear.z = clamp(twist_msg.linear.z * self.time_step, -self.lin_displace_lmt, self.lin_displace_lmt)
        twist.angular.x = clamp(twist_msg.angular.x * self.time_step, -self.ang_displace_lmt, self.ang_displace_lmt)
        twist.angular.y = clamp(twist_msg.angular.y * self.time_step, -self.ang_displace_lmt, self.ang_displace_lmt)
        twist.angular.z = clamp(twist_msg.angular.z * self.time_step, -self.ang_displace_lmt, self.ang_displace_lmt)

        # Get 4x4 homogeneous transform matrix
        transform = transforms.euler_matrix(twist.angular.x, twist.angular.y, twist.angular.z)
        transform[0,3] = twist.linear.x
        transform[1,3] = twist.linear.y
        transform[2,3] = twist.linear.z

        # Update userdata
        ud.transform_matrix = transform

        return self.exit(ud, 'done')

    def exit(self, ud, outcome):
        self.active = False
        return outcome

    def _twist_cb(self, msg):
        self._twist_msg = msg
