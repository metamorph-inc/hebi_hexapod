import rospy
from std_msgs.msg import Bool
from smach import State


class Switch(State):
    def __init__(self):
        State.__init__(self, outcomes=['true','false'])

        switch_topic = "switch"
        self._switch_subscriber = rospy.Subscriber(switch_topic, Bool, self._switch_cb)
        self._switch = False

    def execute(self, ud):
        return str(self._switch).lower()

    def _switch_cb(self, msg):
        self._switch = msg.data
