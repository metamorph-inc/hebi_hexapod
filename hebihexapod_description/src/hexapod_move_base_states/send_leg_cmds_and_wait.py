import rospy
from smach import State
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from hebihexapod_description.msg import LegCmd


class SendLegCmdsAndWait(State):
    def __init__(self, leg_behaviors, time_to_execute, to_leg_topics, from_leg_topics, response_timeout=10.0):
        """SMACH
        :type leg_behaviors: list[str | None]
        :param leg_behaviors: LegCmd behavior, e.g. ['step', None, 'push', None, 'step']

        :type time_to_execute: float
        :param time_to_execute: LegCmd time_to_execute

        :type to_leg_topics: list[str]
        :param to_leg_topics:

        :type from_leg_topics: list[str]
        :param from_leg_topics:

        :type response_timeout: float
        :param response_timeout: Maximum time [s] to wait for leg response after sending leg commands
        """
        State.__init__(self, outcomes=['done','response_timeout'],
                       input_keys=['leg_eff_pos_current','leg_eff_pos_targets','target_joint_states'],
                       output_keys=['leg_eff_pos_current'])

        if len(leg_behaviors) != len(to_leg_topics):
            raise ValueError("len(leg_behaviors) != len(leg_pub_topics)")

        self.leg_behaviors = leg_behaviors
        self.time_to_execute = time_to_execute
        self.to_leg_topics = to_leg_topics
        self.from_leg_topics = from_leg_topics
        self.response_timeout = response_timeout

        self.behaviors_set = ("step","push","move_to_joint_state")

        # Create publishers
        self.to_leg_pubs = [rospy.Publisher(topic, LegCmd, tcp_nodelay=True, queue_size=1) for topic in to_leg_topics]

        # Create subscribers
        self.from_leg_msgs = [None] * len(self.from_leg_topics)
        self.from_leg_subs = [rospy.Subscriber(topic, String, self._from_leg_cb, callback_args=i) for i,topic in enumerate(from_leg_topics)]

        self.rate = rospy.Rate(100)

        self.active = False

    def enter(self, ud):
        self.active = True

    def execute(self, ud):
        self.enter(ud)

        if len(self.leg_behaviors) != len(ud.leg_eff_pos_targets):
            raise ValueError("len(self.leg_behaviors) != len(ud.leg_eff_pos_targets)")

        leg_eff_pos_targets = ud.leg_eff_pos_targets
        leg_joint_state_targets = ud.target_joint_states

        # Get Leg Commands
        legcmds = []
        for i,behavior in enumerate(self.leg_behaviors):

            legcmd = None
            if behavior is not None:
                if behavior in self.behaviors_set:
                    legcmd = LegCmd()
                    legcmd.behavior = behavior
                    if behavior in ['step', 'push']:
                        point = Point()
                        point.x = leg_eff_pos_targets[i][0]
                        point.y = leg_eff_pos_targets[i][1]
                        point.z = leg_eff_pos_targets[i][2]
                        legcmd.target_eff_position = point
                    elif behavior in ['move_to_joint_state']:
                        js = JointState()
                        js.position = leg_joint_state_targets[i]
                        legcmd.target_joint_state = js
                    legcmd.time_to_execute = self.time_to_execute

                    # Update userdata
                    ud.leg_eff_pos_current[i] = leg_eff_pos_targets[i]
                else:
                    rospy.logwarn("Unrecognized leg behavior: %s", behavior)

            legcmds.append(legcmd)

        # Send Leg Commands
        response_index = []
        for i, cmd in enumerate(legcmds):
            if cmd is not None:
                self.to_leg_pubs[i].publish(cmd)
                self.from_leg_msgs[i] = None
                response_index.append(i)

        # Wait for timeout or leg responses
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and len(response_index) > 0:
            elapsed_time = rospy.Time.now().to_sec() - start_time
            if elapsed_time > self.response_timeout:
                return self.exit(ud, 'response_timeout')
            else:
                for i in list(response_index):
                    if self.from_leg_msgs[i] is not None:
                        if self.from_leg_msgs[i] == "ready":
                            response_index.remove(i)  # these are unique so we can use remove
                        else:
                            rospy.logwarn("Unrecognized msg from leg: %s", self.from_leg_msgs[i])

            self.rate.sleep()

        return self.exit(ud, 'done')

    def exit(self, ud, outcome):
        self.active = False
        return outcome

    def _from_leg_cb(self, msg, i):
        print("_from_leg_cb initial {}".format(i))  # TODO: Why do we have so many duplicate msgs?
        if self.active:
            assert isinstance(msg, String)
            self.from_leg_msgs[i] = msg.data
