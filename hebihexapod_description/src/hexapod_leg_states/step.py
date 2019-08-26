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


class Step(State):
    def __init__(self, hebiros_wrapper, urdf_str, base_link, end_link, step_height):
        """SMACH State
        :type hebiros_wrapper: HebirosWrapper
        :param hebiros_wrapper: HebirosWrapper instance for Leg HEBI group

        :type urdf_str: str
        :param urdf_str: Serialized URDF str

        :type base_link: str
        :param base_link: Leg base link name in URDF

        :type end_link: str
        :param end_link: Leg end link name in URDF

        :type step_height: float
        :param step_height:
        """
        State.__init__(self, outcomes=['ik_failed','success'],
                       input_keys=['prev_joint_pos','target_end_link_point','execution_time'],
                       output_keys=['prev_joint_pos', 'active_joints'])
        self.hebi_wrap = hebiros_wrapper
        self.urdf_str = urdf_str
        self.base_link = base_link
        self.end_link = end_link
        self.step_height = step_height
        self.step_height_vector = [0, 0, step_height]  # FIXME: Make step height work regardless of leg base orientation

        self.active = False

        # hardware interface
        self._hold_leg_position = True
        self._hold_joint_angles = []
        self.hebi_wrap.add_feedback_callback(self._hold_leg_pos_cb)

        # pykdl
        self.kdl_fk = KDLKinematics(URDF.from_xml_string(urdf_str), base_link, end_link)
        self._active_joints = self.kdl_fk.get_joint_names()

        # trac-ik
        self.trac_ik = IK(base_link, end_link, urdf_string=urdf_str, timeout=0.01, epsilon=1e-4, solve_type="Distance")
        self.ik_pos_xyz_bounds = [0.01, 0.01, 0.01]
        self.ik_pos_wxyz_bounds = [31416.0, 31416.0, 31416.0]  # NOTE: This implements position-only IK

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
        lift_wp = WaypointMsg()
        end_wp = WaypointMsg()

        eff_prev_target_pos = self.kdl_fk.forward(ud.prev_joint_pos)[:3,3].reshape(1,3).tolist()[0]

        jt_init_pos = self.hebi_wrap.get_joint_positions()
        eff_init_pos = self.kdl_fk.forward(jt_init_pos)[:3,3].reshape(1,3).tolist()[0]

        eff_target_pos = [ud.target_end_link_point.x, ud.target_end_link_point.y, ud.target_end_link_point.z]

        # init_wp
        init_wp.names = self.hebi_wrap.hebi_mapping
        init_wp.positions = jt_init_pos
        init_wp.velocities = [0.0]*self.hebi_wrap.hebi_count
        init_wp.accelerations = [0.0]*self.hebi_wrap.hebi_count

        # lift_wp
        lift_wp.names = self.hebi_wrap.hebi_mapping
        eff_lift_pos = [(a + b + 2.0*c) / 2.0 for a,b,c in zip(eff_init_pos, eff_target_pos, self.step_height_vector)]
        success, lift_wp.positions = self._get_pos_ik(self.trac_ik, jt_init_pos, eff_lift_pos, seed_xyz=eff_prev_target_pos)
        lift_wp.velocities = [NAN]*self.hebi_wrap.hebi_count
        lift_wp.accelerations = [NAN]*self.hebi_wrap.hebi_count

        # end_wp
        end_wp.names = self.hebi_wrap.hebi_mapping
        success, end_wp.positions = self._get_pos_ik(self.trac_ik, lift_wp.positions, eff_target_pos, seed_xyz=eff_lift_pos)
        end_wp.velocities = [0.0]*self.hebi_wrap.hebi_count
        end_wp.accelerations = [0.0]*self.hebi_wrap.hebi_count

        goal = TrajectoryGoal()
        goal.waypoints = [init_wp, lift_wp, end_wp]
        goal.times = [0.0, ud.execution_time/2.0, ud.execution_time]

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

    def _get_pos_ik(self, ik_solver, seed_angles, target_xyz, target_wxyz=None, seed_xyz=None, recursion_depth_cnt=100):
        if recursion_depth_cnt < 0:
            rospy.logdebug("%s FAILURE. Maximum recursion depth reached", self._get_pos_ik.__name__)
            return False, seed_angles
        if target_wxyz is None:
            target_wxyz=[1,0,0,0]  # trak-ik seems a little more stable when given initial pose for pos-only ik
        target_jt_angles = ik_solver.get_ik(seed_angles,
                                            target_xyz[0], target_xyz[1], target_xyz[2],
                                            target_wxyz[0], target_wxyz[1], target_wxyz[2], target_wxyz[3],
                                            self.ik_pos_xyz_bounds[0],
                                            self.ik_pos_xyz_bounds[1],
                                            self.ik_pos_xyz_bounds[2],
                                            self.ik_pos_wxyz_bounds[0],
                                            self.ik_pos_wxyz_bounds[1],
                                            self.ik_pos_wxyz_bounds[2])
        if target_jt_angles is not None:
            rospy.logdebug("%s SUCCESS. Solution: %s to target xyz: %s from seed angles: %s",
                           self._get_pos_ik.__name__, round_list(target_jt_angles,4), round_list(target_xyz,4), round_list(seed_angles,4))
            return True, target_jt_angles
        else:  # ik_solver failed
            if seed_xyz is None:
                return False, seed_angles
            else:  # binary recursive search
                target_xyz_new = [(a + b)/2.0 for a,b in zip(target_xyz, seed_xyz)]
                success, recursive_jt_angles = self._get_pos_ik(ik_solver, seed_angles, target_xyz_new, target_wxyz,
                                                                seed_xyz, recursion_depth_cnt-1)
                if not success:
                    return False, seed_angles
                else:
                    return self._get_pos_ik(ik_solver, recursive_jt_angles, target_xyz, target_wxyz,
                                            target_xyz_new, recursion_depth_cnt-1)

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


