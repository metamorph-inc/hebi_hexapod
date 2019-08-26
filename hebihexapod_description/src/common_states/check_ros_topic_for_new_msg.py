import rospy
from smach import State


class CheckRosTopicForNewMsg(State):
    def __init__(self, topic_name, data_type):
        """SMACH State
        :type topic_name: str
        :param topic_name: ROS Topic to subscribe to

        :type data_type: type
        :param data_type: Data type of messages published on ROS Topic. Note: This wouldn't work with a GUI like FlexBE

        :rtype: str
        :return: 'true' if new msg received; else 'false'
        """
        State.__init__(self, outcomes=['true', 'false'], output_keys=['new_msg'])

        self._sub = rospy.Subscriber(topic_name, data_type, self._sub_cb)
        self._sub_msg = None
        self.rate = rospy.Rate(50)

        self.active = False

    def enter(self, ud):
        self.active = True

    def execute(self, ud):
        self.enter(ud)

        new_msg = self._sub_msg
        self._sub_msg = None

        # Update userdata
        ud.new_msg = new_msg

        if new_msg is not None:
            return self.exit(ud, 'true')
        else:
            self.rate.sleep()
            return self.exit(ud, 'false')

    def exit(self, ud, outcome):
        self.active = False
        return outcome

    def _sub_cb(self, msg):
        self._sub_msg = msg
