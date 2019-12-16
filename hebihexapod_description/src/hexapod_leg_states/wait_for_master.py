import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from smach import State

from hebiros_utils.hebiros_wrapper import HebirosWrapper
from hebihexapod_description.msg import LegCmd

NAN = float('nan')


def round_list(lst, num_decimal):
    return [round(num, num_decimal) for num in lst]


class WaitForMaster(State):
    def __init__(self, hebiros_wrapper, from_master_topic, to_master_topic):
        """SMACH State
        :type hebiros_wrapper: HebirosWrapper
        :param hebiros_wrapper: HebirosWrapper instance for Leg HEBI group

        :type from_master_topic: str
        :param from_master_topic:

        :type to_master_topic: str
        :param to_master_topic: str
        """
        State.__init__(self, outcomes=['exit', 'step', 'push', 'move_to_joint_state'],
                       input_keys=['prev_joint_pos', 'active_joints'],
                       output_keys=['prev_joint_pos','target_end_link_point','target_joint_state','execution_time'])
        self.hebi_wrap = hebiros_wrapper
        self.from_master_topic = from_master_topic
        self.to_master_topic = to_master_topic

        self._from_master_sub = rospy.Subscriber(from_master_topic, LegCmd, self._from_master_cb)
        self._to_master_pub = rospy.Publisher(to_master_topic, String, queue_size=1)

        self._rate = rospy.Rate(100)
        self._from_master_msg = None
        self._to_master_ready_msg = 'ready'

        self._active_joints = None

        self.active = False

        # hardware interface
        self._hold_leg_position = True
        self._hold_joint_angles = []
        self.hebi_wrap.add_feedback_callback(self._hold_leg_pos_cb)

        # joint state publisher
        while not rospy.is_shutdown() and len(self.hebi_wrap.get_joint_positions()) < len(self.hebi_wrap.hebi_mapping):
            rospy.sleep(0.1)
        self._joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.hebi_wrap.add_feedback_callback(self._joint_state_cb)

    def enter(self, ud):
        if self._hold_joint_angles is None:
            ud.prev_joint_pos = [0, 0, 0]
            self._hold_leg_position = False
        else:
            self._hold_joint_angles = ud.prev_joint_pos
            self._hold_leg_position = True
        self._active_joints = ud.active_joints
        self.active = True

    def execute(self, ud):
        self.enter(ud)

        self._to_master_pub.publish(self._to_master_ready_msg)
        while not rospy.is_shutdown():
            if self._from_master_msg is not None:
                # process msg from master
                self._update_userdata(ud)
                msg = self._from_master_msg
                self._from_master_msg = None
                if msg.behavior in self.get_registered_outcomes():
                    self.exit(ud)
                    return msg.behavior
                else:
                    rospy.loginfo("Unrecognized LegCmd.leg_behavior [str]: %s", msg.leg_behavior)
            self._rate.sleep()

        self.exit(ud)
        return 'exit'

    def _update_userdata(self, ud):
        ud.target_end_link_point = self._from_master_msg.target_eff_position
        ud.target_joint_state = self._from_master_msg.target_joint_state
        ud.execution_time = self._from_master_msg.time_to_execute

    def exit(self, ud):
        self.active = False
        self._hold_leg_position = False

    def _from_master_cb(self, msg):
        assert isinstance(msg, LegCmd)
        self._from_master_msg = msg

    def _hold_leg_pos_cb(self, msg):
        if not rospy.is_shutdown() and self.active and self._hold_leg_position:
            jointstate = JointState()
            jointstate.name = self.hebi_wrap.hebi_mapping
            jointstate.position = self._hold_joint_angles
            jointstate.velocity = []
            jointstate.effort = []
            self.hebi_wrap.joint_state_publisher.publish(jointstate)

    def _joint_state_cb(self, msg):
        if not rospy.is_shutdown() and self.active and self._active_joints is not None:
            jointstate = JointState()
            jointstate.header.stamp = rospy.Time.now()
            jointstate.name = self._active_joints
            jointstate.position = self.hebi_wrap.get_joint_positions()
            jointstate.velocity = []
            jointstate.effort = []
            self._joint_state_pub.publish(jointstate)
