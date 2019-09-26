#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
# Description:
#   rospy adaption of HEBI Hexapod Matlab scripts
#   https://github.com/HebiRobotics/hebi-matlab-examples/tree/master/kits/hexapod
"""

from __future__ import print_function

import sys
import math as m
import numpy as np
import copy

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from actionlib.simple_action_client import SimpleActionClient
from hebiros.srv import AddGroupFromNamesSrv, SendCommandWithAcknowledgementSrv
from hebiros.msg import TrajectoryAction, TrajectoryGoal, WaypointMsg, CommandMsg
from urdf_parser_py.urdf import URDF
from trac_ik_python.trac_ik import IK
from pykdl_utils.kdl_kinematics import KDLKinematics

from common import transformations as transforms
sys.stdout.flush()

NAN = float('nan')

LEGS = 6
ACTUATORS_PER_LEG = 3
ACTUATORS_TOTAL = 18

STEP_HEIGHT = 0.04


def round_list(lst, num_decimal):
    return [round(num, num_decimal) for num in lst]


def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n


class Hexapod(object):
    """
    Attributes
        hebi_group_name     (str):
        hebi_mapping        (list of list of str):
        leg_base_links      (list of str):
        leg_end_links       (list of str):
    """
    def __init__(self, hebi_group_name, hebi_mapping, leg_base_links, leg_end_links):
        rospy.loginfo("Creating Hexapod instance...")
        hebi_families = []
        hebi_names = []
        for leg in hebi_mapping:
            for actuator in leg:
                family, name = actuator.split('/')
                hebi_families.append(family)
                hebi_names.append(name)
        rospy.loginfo("  hebi_group_name: %s", hebi_group_name)
        rospy.loginfo("  hebi_families: %s", hebi_families)
        rospy.loginfo("  hebi_names: %s", hebi_names)
        self.hebi_mapping = hebi_mapping
        self.hebi_mapping_flat = self._flatten(self.hebi_mapping)

        # jt information populated by self._feedback_cb
        self._current_jt_pos = {}
        self._current_jt_vel = {}
        self._current_jt_eff = {}
        self._joint_state_pub = None

        self._hold_leg_list = [False, False, False, False, False, False ]
        self._hold_leg_positions = self._get_list_of_lists()

        # Create a service client to create a group
        set_hebi_group = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)
        # Topic to receive feedback from a group
        self.hebi_group_feedback_topic = "/hebiros/"+hebi_group_name+"/feedback/joint_state"
        rospy.loginfo("  hebi_group_feedback_topic: %s", "/hebiros/"+hebi_group_name+"/feedback/joint_state")
        # Topic to send commands to a group
        self.hebi_group_command_topic = "/hebiros/"+hebi_group_name+"/command/joint_state"
        rospy.loginfo("  hebi_group_command_topic: %s", "/hebiros/"+hebi_group_name+"/command/joint_state")
        # Call the /hebiros/add_group_from_names service to create a group
        rospy.loginfo("  Waiting for AddGroupFromNamesSrv at %s ...", '/hebiros/add_group_from_names')
        rospy.wait_for_service('/hebiros/add_group_from_names')  # block until service server starts
        rospy.loginfo("  AddGroupFromNamesSrv AVAILABLE.")
        set_hebi_group(hebi_group_name, hebi_names, hebi_families)
        # Create a service client to set group settings
        change_group_settings = rospy.ServiceProxy("/hebiros/"+hebi_group_name+"/send_command_with_acknowledgement",
                                                   SendCommandWithAcknowledgementSrv)
        rospy.loginfo("  Waiting for SendCommandWithAcknowledgementSrv at %s ...", "/hebiros/"+hebi_group_name+"/send_command_with_acknowledgement")
        rospy.wait_for_service("/hebiros/"+hebi_group_name+"/send_command_with_acknowledgement")  # block until service server starts
        cmd_msg = CommandMsg()
        cmd_msg.name = self.hebi_mapping_flat
        cmd_msg.settings.name = self.hebi_mapping_flat
        cmd_msg.settings.position_gains.name = self.hebi_mapping_flat
        cmd_msg.settings.position_gains.kp = [5, 8, 2]*LEGS
        cmd_msg.settings.position_gains.ki = [0.001]*LEGS*ACTUATORS_PER_LEG
        cmd_msg.settings.position_gains.kd = [0]*LEGS*ACTUATORS_PER_LEG
        cmd_msg.settings.position_gains.i_clamp = [0.25]*LEGS*ACTUATORS_PER_LEG  # TODO: Tune this. Setting it low for testing w/o restarting Gazebo
        change_group_settings(cmd_msg)

        # Feedback/Command
        self.fbk_sub = rospy.Subscriber(self.hebi_group_feedback_topic, JointState, self._feedback_cb)
        self.cmd_pub = rospy.Publisher(self.hebi_group_command_topic, JointState, queue_size=1)
        self._hold_position = False
        self._hold_joint_states = {}
        # TrajectoryAction client
        self.trajectory_action_client = SimpleActionClient("/hebiros/"+hebi_group_name+"/trajectory", TrajectoryAction)
        rospy.loginfo("  Waiting for TrajectoryActionServer at %s ...", "/hebiros/"+hebi_group_name+"/trajectory")
        self.trajectory_action_client.wait_for_server()  # block until action server starts
        rospy.loginfo("  TrajectoryActionServer AVAILABLE.")
        # Twist Subscriber
        self._cmd_vel_sub = rospy.Subscriber("/hexapod/cmd_vel/", Twist, self._cmd_vel_cb)
        self.last_vel_cmd = None
        self.linear_displacement_limit = 0.075  # m
        self.angular_displacement_limit = 0.65  # rad

        # Check ROS Parameter server for robot_description URDF
        urdf_str = ""
        urdf_loaded = False
        while not rospy.is_shutdown() and not urdf_loaded:
            if rospy.has_param('/robot_description'):
                urdf_str = rospy.get_param('/robot_description')
                urdf_loaded = True
                rospy.loginfo("Pulled /robot_description from parameter server.")
            else:
                rospy.sleep(0.01)  # sleep for 10 ms of ROS time
        # pykdl_utils setup
        self.robot_urdf = URDF.from_xml_string(urdf_str)
        self.kdl_fk_base_to_leg_base = [KDLKinematics(self.robot_urdf, 'base_link', base_link)
                                        for base_link in leg_base_links]
        self.kdl_fk_leg_base_to_eff = [KDLKinematics(self.robot_urdf, base_link, end_link)
                                       for base_link, end_link in zip(leg_base_links, leg_end_links)]
        # trac-ik setup
        self.trac_ik_leg_base_to_end = [IK(base_link,
                                           end_link,
                                           urdf_string=urdf_str,
                                           timeout=0.01,  # TODO: Tune me
                                           epsilon=1e-4,
                                           solve_type="Distance")
                                        for base_link, end_link in zip(leg_base_links, leg_end_links)]
        self.ik_pos_xyz_bounds = [0.01, 0.01, 0.01]
        self.ik_pos_wxyz_bounds = [31416.0, 31416.0, 31416.0]  # NOTE: This implements position-only IK
        # Wait for connections to be set up
        rospy.loginfo("Wait for ROS connections to be set up...")
        while not rospy.is_shutdown() and len(self._current_jt_pos) < len(self.hebi_mapping_flat):
            rospy.sleep(0.1)
        # Set up joint state publisher
        self._joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

        self._loop_rate = rospy.Rate(20)

        # leg joint home pos     Hip,  Knee,  Ankle
        self.leg_jt_home_pos = [[0.0, +0.26, -1.57],  # Leg 1
                                [0.0, -0.26, +1.57],  # Leg 2
                                [0.0, +0.26, -1.57],  # Leg 3
                                [0.0, -0.26, +1.57],  # Leg 4
                                [0.0, +0.26, -1.57],  # Leg 5
                                [0.0, -0.26, +1.57]]  # Leg 6

        # leg end-effector home position
        self.leg_eff_home_pos = self._get_leg_base_to_eff_fk(self.leg_jt_home_pos)

        # leg step height relative to leg base link
        self.leg_eff_step_height = [[]]*LEGS  # relative to leg base
        for i, fk_solver in enumerate(self.kdl_fk_base_to_leg_base):
            base_to_leg_base_rot = fk_solver.forward([])[:3,:3]
            step_ht_chassis = np.array([0, 0, STEP_HEIGHT])
            step_ht_leg_base = np.dot(base_to_leg_base_rot, step_ht_chassis)
            self.leg_eff_step_height[i] = step_ht_leg_base.tolist()[0]

        self._odd_starts = True

        rospy.loginfo("Done creating Hexapod instance...")

    def stand_up(self):
        rospy.loginfo("Hexapod standing up...")
        current_leg_positions = self._get_joint_angles()

        goal = TrajectoryGoal()
        start_wp = WaypointMsg()
        start_wp.names = self.hebi_mapping_flat
        start_wp.positions = self._flatten(current_leg_positions)
        start_wp.velocities = [0.0]*ACTUATORS_TOTAL
        start_wp.accelerations = [0.0]*ACTUATORS_TOTAL
        goal.waypoints.append(start_wp)
        goal.times.append(0.0)
        end_wp = WaypointMsg()
        end_wp.names = self.hebi_mapping_flat
        end_wp.positions = self._flatten(self.leg_jt_home_pos)
        end_wp.velocities = [0.0]*ACTUATORS_TOTAL
        end_wp.accelerations = [0.0]*ACTUATORS_TOTAL
        goal.waypoints.append(end_wp)
        goal.times.append(4.0)

        self.trajectory_action_client.send_goal(goal)  # TODO: Add the various callbacks
        self.trajectory_action_client.wait_for_result()
        self.hold_pos([1,2,3,4,5,6])

    def loop(self):
        """Main Hexapod loop (distant - somewhat less accomplished - relative of HEBI algorithm)
            - Get chassis translation
            - Get leg end-effector translations (relative to leg base link)
            - side_alpha:odd, side_beta:even or side_alpha:even, side_beta:odd
            - side_alpha legs lift, plant to +transformation
            - side_alpha legs push to new home positions; side_beta legs push to -transformation
            - swap side_alpha and side_beta
        """
        rospy.loginfo("Hexapod entering main loop...")
        rospy.loginfo("  Waiting for initial velocity command on /hexapod/cmd_vel/ ...")
        while self.last_vel_cmd is None:
            self._loop_rate.sleep()

        # start main loop
        while not rospy.is_shutdown():

            chassis_pos_delta = None
            if self.last_vel_cmd is not None:
                dt = 1  # FIXME: Temporary for debugging
                lin_disp_lmt = self.linear_displacement_limit
                ang_disp_lmt = self.angular_displacement_limit
                chassis_pos_delta = Twist()
                chassis_pos_delta.linear.x = clamp(self.last_vel_cmd.linear.x*dt, -lin_disp_lmt, lin_disp_lmt)
                chassis_pos_delta.linear.y = clamp(self.last_vel_cmd.linear.y*dt, -lin_disp_lmt, lin_disp_lmt)
                chassis_pos_delta.linear.z = clamp(self.last_vel_cmd.linear.z*dt, -lin_disp_lmt, lin_disp_lmt)
                chassis_pos_delta.angular.x = clamp(self.last_vel_cmd.angular.x*dt, -ang_disp_lmt, ang_disp_lmt)
                chassis_pos_delta.angular.y = clamp(self.last_vel_cmd.angular.y*dt, -ang_disp_lmt, ang_disp_lmt)
                chassis_pos_delta.angular.z = clamp(self.last_vel_cmd.angular.z*dt, -ang_disp_lmt, ang_disp_lmt)
                self.last_vel_cmd = None

            if chassis_pos_delta is not None \
                    and not self._check_if_twist_msg_is_zero(chassis_pos_delta, linear_threshold=0.005, angular_threshold=0.01):
                # Get chassis position transformation
                chassis_pos_rot = transforms.euler_matrix(chassis_pos_delta.angular.x,
                                                          chassis_pos_delta.angular.y,
                                                          chassis_pos_delta.angular.z)[:3,:3]

                rospy.loginfo("chassis_pos_rot: %s", chassis_pos_rot)
                chassis_pos_trans = np.zeros([3])
                chassis_pos_trans[0] = chassis_pos_delta.linear.x
                chassis_pos_trans[1] = chassis_pos_delta.linear.y
                chassis_pos_trans[2] = chassis_pos_delta.linear.z
                chassis_translation = np.dot(chassis_pos_trans, chassis_pos_rot)
                rospy.loginfo("chassis_translation: %s", chassis_translation)

                leg_target_eff_translation = [[]]*LEGS
                # Get leg base positions relative to chassis
                leg_base_positions = self._get_base_to_leg_base_fk()
                for i, leg_base_pos in enumerate(leg_base_positions):
                    leg_base_pos_arr = np.array(leg_base_pos).reshape(3,1)
                    leg_base_pos_arr_new = np.dot(chassis_pos_rot, leg_base_pos_arr)
                    leg_base_pos_trans_4 = np.ones(4).reshape(4,1)
                    leg_base_pos_trans_4[:3,:] = leg_base_pos_arr_new
                    # get leg base translations relative to leg_base coordinate frame
                    relative_trans = np.dot(np.linalg.inv(self.kdl_fk_base_to_leg_base[i].forward([])), leg_base_pos_trans_4)
                    relative_trans = relative_trans.reshape(1,4).tolist()[0][:3]
                    leg_target_eff_translation[i] = relative_trans

                # Get leg target end-effector translations
                for i, q in enumerate(self.leg_jt_home_pos):
                    base_to_leg_base_rot = self.kdl_fk_base_to_leg_base[i].forward([])[:3,:3]
                    leg_target_eff_trans = np.dot(np.linalg.inv(base_to_leg_base_rot),chassis_translation).tolist()[0]
                    leg_target_eff_translation[i] = [x+y for x,y in zip(leg_target_eff_translation[i], leg_target_eff_trans)]  # TODO: FIXME: Technically incorrect

                # 1: side_alpha legs lift, plant to +transformation
                rospy.loginfo("1: side_alpha legs lift, plant to +transformation")
                if self._odd_starts:
                    active_legs = [1,2,5]
                else:  # even starts
                    active_legs = [0,3,4]

                init_wp = WaypointMsg()
                lift_wp = WaypointMsg()
                end_wp = WaypointMsg()

                legs_jt_init_pos = self._get_joint_angles()
                leg_eff_cur_pos = self._get_leg_base_to_eff_fk(legs_jt_init_pos)
                for i in range(LEGS):
                    motor_names = [name for name in self.hebi_mapping[i]]
                    # INITIAL POSITION
                    init_wp.names.extend(motor_names)
                    init_wp.positions.extend(legs_jt_init_pos[i])
                    init_wp.velocities.extend([0.0]*ACTUATORS_PER_LEG)
                    init_wp.accelerations.extend([0.0]*ACTUATORS_PER_LEG)
                    # LIFT
                    lift_wp.names.extend(motor_names)
                    if i in active_legs:
                        # apply translation
                        leg_lift_eff_target_pos = [(x + y + z) / 2.0 for x, y, z in zip(leg_eff_cur_pos[i], self.leg_eff_home_pos[i], leg_target_eff_translation[i])]
                        leg_lift_eff_target_pos = [x + y for x,y in zip(leg_lift_eff_target_pos, self.leg_eff_step_height[i])]
                        # get ik
                        leg_lift_jt_target_pos = self._get_pos_ik(self.trac_ik_leg_base_to_end[i], legs_jt_init_pos[i],
                                                                  leg_lift_eff_target_pos, seed_xyz=self.leg_eff_home_pos[i])
                        lift_wp.positions.extend(leg_lift_jt_target_pos)
                        lift_wp.velocities.extend([NAN]*ACTUATORS_PER_LEG)
                        lift_wp.accelerations.extend([NAN]*ACTUATORS_PER_LEG)
                    else:
                        lift_wp.positions.extend(legs_jt_init_pos[i])
                        lift_wp.velocities.extend([0.0]*ACTUATORS_PER_LEG)
                        lift_wp.accelerations.extend([0.0]*ACTUATORS_PER_LEG)
                    # PLANT
                    end_wp.names.extend(motor_names)
                    if i in active_legs:
                        # apply translation
                        leg_plant_eff_target_pos = [x + y for x,y in zip(self.leg_eff_home_pos[i], leg_target_eff_translation[i])]
                        leg_plant_eff_target_pos[2] = self.leg_eff_home_pos[i][2]  # end eff z-position should match home z-position
                        # get ik
                        leg_plant_jt_target_pos = self._get_pos_ik(self.trac_ik_leg_base_to_end[i], leg_lift_jt_target_pos,
                                                                   leg_plant_eff_target_pos, seed_xyz=leg_lift_eff_target_pos)
                        end_wp.positions.extend(leg_plant_jt_target_pos)
                        end_wp.velocities.extend([0.0]*ACTUATORS_PER_LEG)
                        end_wp.accelerations.extend([0.0]*ACTUATORS_PER_LEG)
                    else:
                        end_wp.positions.extend(legs_jt_init_pos[i])
                        end_wp.velocities.extend([0.0]*ACTUATORS_PER_LEG)
                        end_wp.accelerations.extend([0.0]*ACTUATORS_PER_LEG)

                goal = TrajectoryGoal()
                goal.waypoints.append(init_wp)
                goal.waypoints.append(lift_wp)
                goal.waypoints.append(end_wp)
                goal.times.extend([0.0, 0.4, 0.8])

                self.release_pos([1,2,3,4,5,6])
                self.trajectory_action_client.send_goal(goal)
                self.trajectory_action_client.wait_for_result()
                self.hold_pos([1,2,3,4,5,6])

                # 2: side_alpha legs push to new home positions; side_beta legs push to -transformation
                rospy.loginfo("2: side_alpha legs push to new home positions; side_beta legs push to -transformation")
                if self._odd_starts:
                    active_legs = [0,3,4]
                else:  # even starts
                    active_legs = [1,2,5]

                init_wp = WaypointMsg()
                end_wp = WaypointMsg()

                legs_jt_init_pos = self._get_joint_angles()
                for i in range(LEGS):
                    motor_names = [name for name in self.hebi_mapping[i]]
                    # INITIAL POSITION
                    init_wp.names.extend(motor_names)
                    init_wp.positions.extend(legs_jt_init_pos[i])
                    init_wp.velocities.extend([0.0]*ACTUATORS_PER_LEG)
                    init_wp.accelerations.extend([0.0]*ACTUATORS_PER_LEG)
                    # PUSH
                    end_wp.names.extend(motor_names)
                    if i in active_legs:
                        # apply -translation
                        leg_plant_eff_target_pos = [x + y for x,y in zip(self.leg_eff_home_pos[i], [-val for val in leg_target_eff_translation[i]])]
                        leg_plant_eff_target_pos[2] = self.leg_eff_home_pos[i][2]  # end eff z-position should match home z-position
                        # get ik
                        leg_plant_jt_target_pos = self._get_pos_ik(self.trac_ik_leg_base_to_end[i], legs_jt_init_pos[i],
                                                                   leg_plant_eff_target_pos, seed_xyz=self.leg_eff_home_pos[i])
                        end_wp.positions.extend(leg_plant_jt_target_pos)
                        end_wp.velocities.extend([0.0]*ACTUATORS_PER_LEG)
                        end_wp.accelerations.extend([0.0]*ACTUATORS_PER_LEG)
                    else:
                        end_wp.positions.extend(self.leg_jt_home_pos[i])
                        end_wp.velocities.extend([0.0]*ACTUATORS_PER_LEG)
                        end_wp.accelerations.extend([0.0]*ACTUATORS_PER_LEG)

                goal = TrajectoryGoal()
                goal.waypoints.append(init_wp)
                goal.waypoints.append(end_wp)
                goal.times.extend([0.0, 0.4])

                self.release_pos([1,2,3,4,5,6])
                self.trajectory_action_client.send_goal(goal)
                self.trajectory_action_client.wait_for_result()
                self.hold_pos([1,2,3,4,5,6])

                self._odd_starts = not self._odd_starts

            self._loop_rate.sleep()  # FIXME: Doesn't make sense to use this unless re-planning trajectories
        # end main loop

    def _get_pos_ik(self, ik_solver, seed_angles, target_xyz, target_wxyz=None, seed_xyz=None, recursion_depth_cnt=100):
        if recursion_depth_cnt < 0:
            rospy.logdebug("%s FAILURE. Maximum recursion depth reached", self._get_pos_ik.__name__)
            return None
        rospy.logdebug("recursion depth = %s", recursion_depth_cnt)
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
        if target_jt_angles is not None:  # ik_solver succeeded
            rospy.logdebug("%s SUCCESS. Solution: %s to target xyz: %s from seed angles: %s",
                           self._get_pos_ik.__name__, round_list(target_jt_angles,4), round_list(target_xyz,4), round_list(seed_angles,4))
            return target_jt_angles
        else:  # ik_solver failed
            if seed_xyz is None:
                rospy.logdebug("%s FAILURE. Solution: %s to target_xyz: %s from seed_angles: %s",
                               self._get_pos_ik.__name__, ['NA', 'NA', 'NA'], target_xyz, seed_angles)
                return target_jt_angles
            else:
                # binary recursive search
                target_xyz_new = [(x + y)/2.0 for x,y in zip(target_xyz, seed_xyz)]
                recursive_jt_angles = self._get_pos_ik(ik_solver, seed_angles, target_xyz_new, target_wxyz,
                                                       seed_xyz, recursion_depth_cnt-1)
                if recursive_jt_angles is None:
                    rospy.logdebug("%s FAILURE. Solution: %s to target_xyz: %s from seed_angles: %s",
                                   self._get_pos_ik.__name__, ['NA', 'NA', 'NA'], round_list(target_xyz,4), round_list(seed_angles,4))
                else:
                    return self._get_pos_ik(ik_solver, recursive_jt_angles, target_xyz, target_wxyz,
                                            target_xyz_new, recursion_depth_cnt-1)

    def _get_base_to_leg_base_fk(self):
        leg_base_pos = [[]]*LEGS
        for i, fk_solver in enumerate(self.kdl_fk_base_to_leg_base):
            base_to_leg_base_tf = fk_solver.forward([])
            leg_base_pos[i] = base_to_leg_base_tf[:3,3].reshape(1,3).tolist()[0]
        return leg_base_pos

    def _get_leg_base_to_eff_fk(self, jt_angles):
        leg_eff_cur_pos = [[]]*LEGS  # relative to leg base
        for i, (fk_solver, angles) \
                in enumerate(zip(self.kdl_fk_leg_base_to_eff, jt_angles)):
            leg_base_to_eff_tf = fk_solver.forward(angles)
            leg_eff_cur_pos[i] = leg_base_to_eff_tf[:3,3].reshape(1,3).tolist()[0]
        return leg_eff_cur_pos

    def _get_base_to_leg_eff_fk(self, jt_angles):
        leg_eff_cur_pos = [[]]*LEGS  # relative to base
        for i, (fk_solver_1, fk_solver_2, angles) \
                in enumerate(zip(self.kdl_fk_base_to_leg_base, self.kdl_fk_leg_base_to_eff, jt_angles)):
            base_to_leg_base_tf = fk_solver_1.forward([])
            leg_base_to_eff_tf = fk_solver_2.forward(angles)
            base_to_eff_tf = np.dot(base_to_leg_base_tf, leg_base_to_eff_tf)
            leg_eff_cur_pos[i] = base_to_eff_tf[:3,3].reshape(1,3).tolist()[0]
        return leg_eff_cur_pos

    def hold_pos(self, leg_nums):
        released_leg = False
        for num in leg_nums:
            num -= 1  # Convert from 1-based leg numbering to 0-based indexing
            if not self._hold_leg_list[num]:
                released_leg = True
                break
        if released_leg:
            self._hold_leg_positions = self._get_joint_angles()
            for num in leg_nums:
                num -= 1  # Convert from 1-based leg numbering to 0-based indexing
                self._hold_leg_list[num] = True

    def release_pos(self, leg_nums):
        for num in leg_nums:
            num -= 1  # Convert from 1-based leg numbering to 0-based indexing
            self._hold_leg_list[num] = False

    def _get_joint_angles(self):
        return [[self._current_jt_pos[motor] for motor in leg] for leg in self.hebi_mapping]

    def _get_joint_velocities(self):
        return [[self._current_jt_vel[motor] for motor in leg] for leg in self.hebi_mapping]

    def _get_joint_efforts(self):
        return [[self._current_jt_eff[motor] for motor in leg] for leg in self.hebi_mapping]

    @staticmethod
    def _get_list_of_lists(item=None):
        return [[item]*ACTUATORS_PER_LEG]*LEGS

    @staticmethod
    def _flatten(listoflists):
        return [item for lst in listoflists for item in lst]

    def _feedback_cb(self, msg):
        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            if name not in self.hebi_mapping_flat:
                print("WARNING: arm_callback - unrecognized name!!!")
            else:
                self._current_jt_pos[name] = pos
                self._current_jt_vel[name] = vel
                self._current_jt_eff[name] = eff
                # Publish JointState() for RViz
                if not rospy.is_shutdown() and self._joint_state_pub is not None:
                    jointstate = JointState()
                    jointstate.header.stamp = rospy.Time.now()
                    jointstate.name = self.hebi_mapping_flat
                    jointstate.position = self._flatten(self._get_joint_angles())
                    jointstate.velocity = [0.0] * len(jointstate.name)
                    jointstate.effort = [0.0] * len(jointstate.name)
                    self._joint_state_pub.publish(jointstate)

        # Publish JointState() for held legs
        # TODO: Probably better to put in its own callback
        # TODO: Trigger at regular intervals and when self._hold_leg_list changes
        # TODO: Smooth transition from trajectory to cmd
        if not rospy.is_shutdown() and self._joint_state_pub is not None and any(self._hold_leg_list):
            jointstate = JointState()
            jointstate.header.stamp = rospy.Time.now()
            for i, leg in enumerate(self.hebi_mapping):
                if self._hold_leg_list[i]:
                    jointstate.name.extend(leg)
                    jointstate.position.extend(self._hold_leg_positions[i])
                    jointstate.velocity = []
                    jointstate.effort = []  # TODO: Gravity compensation?
            self.cmd_pub.publish(jointstate)

    def _cmd_vel_cb(self, msg):
        if isinstance(msg, Twist):
            if self.last_vel_cmd is None:
                self.last_vel_cmd = Twist()
            self.last_vel_cmd.linear.x = msg.linear.x
            self.last_vel_cmd.linear.y = msg.linear.y
            self.last_vel_cmd.linear.z = msg.linear.z
            self.last_vel_cmd.angular.x = msg.angular.x
            self.last_vel_cmd.angular.y = msg.angular.y
            self.last_vel_cmd.angular.z = msg.angular.z

    @staticmethod
    def _check_if_twist_msg_is_zero(twist_msg, linear_threshold, angular_threshold):
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


if __name__ == '__main__':

    rospy.init_node('run_hebi_hexapod')

    hebi_group_name = 'hexapod'
    hebi_mapping = [
                    ['Leg1/Hip', 'Leg1/Knee', 'Leg1/Ankle'],
                    ['Leg2/Hip', 'Leg2/Knee', 'Leg2/Ankle'],
                    ['Leg3/Hip', 'Leg3/Knee', 'Leg3/Ankle'],
                    ['Leg4/Hip', 'Leg4/Knee', 'Leg4/Ankle'],
                    ['Leg5/Hip', 'Leg5/Knee', 'Leg5/Ankle'],
                    ['Leg6/Hip', 'Leg6/Knee', 'Leg6/Ankle']
                   ]
    leg_base_links = ['a_2039_02_2Z', 'a_2039_02_7Z', 'a_2039_02_8Z', 'a_2039_02_12Z', 'a_2039_02_13Z', 'a_2039_02_10Z']  # Hardcoded for now
    leg_end_links = ['pp_2057_01_15Z', 'pp_2057_01_14Z', 'pp_2057_01_34Z', 'pp_2057_01_45Z', 'pp_2057_01_55Z', 'pp_2057_01_44Z']  # Hardcoded for now
    hexapod = Hexapod(hebi_group_name=hebi_group_name,
                      hebi_mapping=hebi_mapping,
                      leg_base_links=leg_base_links,
                      leg_end_links=leg_end_links)

    userinput = raw_input("Press Enter to stand up")
    hexapod.stand_up()
    userinput = raw_input("Press Enter to start main loop")
    hexapod.loop()
    userinput = raw_input("Press Enter to exit")
    hexapod.release_pos([1,2,3,4,5,6])
    rospy.signal_shutdown("I'm sorry, Dave. I'm afraid I can't do that.")
