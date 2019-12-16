import rospy
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from smach import State

from trac_ik_python.trac_ik import IK

from pykdl_utils.kdl_kinematics import KDLKinematics
from hebiros.msg import WaypointMsg, TrajectoryGoal
from hebiros_utils.hebiros_wrapper import HebirosWrapper

NAN = float('nan')


def round_list(lst, num_decimal):
    return [round(num, num_decimal) for num in lst]


class MoveToJointState(State):
    def __init__(self, hebiros_wrapper, urdf_str, base_link, end_link):
        """SMACH State
        :type hebiros_wrapper: HebirosWrapper
        :param hebiros_wrapper: HebirosWrapper instance for Leg HEBI group

        :type urdf_str: str
        :param urdf_str: Serialized URDF str

        :type base_link: str
        :param base_link: Leg base link name in URDF

        :type end_link: str
        :param end_link: Leg end link name in URDF
        """
        State.__init__(self, outcomes=['success'],
                       input_keys=['prev_joint_pos','target_joint_state','execution_time'],
                       output_keys=['prev_joint_pos', 'active_joints'])
        self.hebi_wrap = hebiros_wrapper
        self.urdf_str = urdf_str
        self.base_link = base_link
        self.end_link = end_link

        self.active = False

        # hardware interface
        self._hold_leg_position = True
        self._hold_joint_angles = []
        self.hebi_wrap.add_feedback_callback(self._hold_leg_pos_cb)

        # pykdl
        self.kdl_fk = KDLKinematics(URDF.from_xml_string(urdf_str), base_link, end_link)
        self._active_joints = self.kdl_fk.get_joint_names()

        # joint state publisher
        while not rospy.is_shutdown() and len(self.hebi_wrap.get_joint_positions()) < len(self.hebi_wrap.hebi_mapping):
            rospy.sleep(0.1)
        self._joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.hebi_wrap.add_feedback_callback(self._joint_state_cb)

    def enter(self, ud):
        self._hold_joint_angles = ud.prev_joint_pos
        self.active = True

    def execute(self, ud):
        self.enter(ud)

        init_wp = WaypointMsg()
        end_wp = WaypointMsg()

        jt_init_pos = self.hebi_wrap.get_joint_positions()

        # init_wp
        init_wp.names = self.hebi_wrap.hebi_mapping
        init_wp.positions = jt_init_pos
        init_wp.velocities = [0.0]*self.hebi_wrap.hebi_count
        init_wp.accelerations = [0.0]*self.hebi_wrap.hebi_count

        # end_wp
        end_wp.names = self.hebi_wrap.hebi_mapping
        end_wp.positions = ud.target_joint_state.position
        end_wp.velocities = [0.0]*self.hebi_wrap.hebi_count
        end_wp.accelerations = [0.0]*self.hebi_wrap.hebi_count

        goal = TrajectoryGoal()
        goal.waypoints = [init_wp, end_wp]
        goal.times = [0.0, ud.execution_time]

        # send goal to trajectory action server
        self._hold_leg_position = False
        self.hebi_wrap.trajectory_action_client.send_goal(goal)
        self.hebi_wrap.trajectory_action_client.wait_for_result()
        self._hold_leg_position = True

        ud.prev_joint_pos = end_wp.positions
        self.exit(ud)
        return 'success'

    def exit(self, ud):
        ud.active_joints = self._active_joints
        self.active = False

    def _hold_leg_pos_cb(self, msg):
        if not rospy.is_shutdown() and self.active and self._hold_leg_position:
            jointstate = JointState()
            jointstate.name = self.hebi_wrap.hebi_mapping
            jointstate.position = self._hold_joint_angles
            jointstate.velocity = []
            jointstate.effort = []
            self.hebi_wrap.joint_state_publisher.publish(jointstate)

    def _joint_state_cb(self, msg):
        if not rospy.is_shutdown() and self.active:
            jointstate = JointState()
            jointstate.header.stamp = rospy.Time.now()
            jointstate.name = self._active_joints
            jointstate.position = self.hebi_wrap.get_joint_positions()
            jointstate.velocity = []
            jointstate.effort = []
            self._joint_state_pub.publish(jointstate)
