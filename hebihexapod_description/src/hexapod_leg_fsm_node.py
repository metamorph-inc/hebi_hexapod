#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import argparse

import rospy
from smach import StateMachine
import smach_ros

from hebiros.msg import CommandMsg
from hebiros_utils.hebiros_wrapper import HebirosWrapper
from hexapod_leg_states.wait_for_master import WaitForMaster
from hexapod_leg_states.step import Step
from hexapod_leg_states.push import Push


def parse_args(args):
    """
    :type args: list
    :param args:

    :rtype:
    :return:
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-hebi_group_name', type=str, required=True, help="")
    parser.add_argument('-hebi_mapping_hip', type=str, required=True, help="")
    parser.add_argument('-hebi_mapping_knee', type=str, required=True, help="")
    parser.add_argument('-hebi_mapping_ankle', type=str, required=True, help="")
    parser.add_argument('-base_link_name', type=str, required=True, help="")
    parser.add_argument('-end_link_name', type=str, required=True, help="")
    parser.add_argument('-from_master_topic', type=str, required=True, help="")
    parser.add_argument('-to_master_topic', type=str, required=True, help="")
    parser.add_argument('-step_height', type=float, default=0.04, help="")
    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def main():
    # ROS stuff
    rospy.init_node('hexapod_leg_fsm_node', anonymous=True)

    # Parse node args
    parser = parse_args(sys.argv[1:])
    hebi_group_name = parser.hebi_group_name
    hebi_mapping_hip = parser.hebi_mapping_hip
    hebi_mapping_knee = parser.hebi_mapping_knee
    hebi_mapping_ankle = parser.hebi_mapping_ankle
    base_link_name = parser.base_link_name
    end_link_name = parser.end_link_name
    from_master_topic = parser.from_master_topic
    to_master_topic = parser.to_master_topic
    step_height = parser.step_height

    # Hardware interface
    hebi_mapping = [hebi_mapping_hip, hebi_mapping_knee, hebi_mapping_ankle]
    hebi_families = []
    hebi_names = []
    for actuator in hebi_mapping:
        family, name = actuator.split('/')
        hebi_families.append(family)
        hebi_names.append(name)
    hebi_wrap = HebirosWrapper(hebi_group_name, hebi_families, hebi_names)
    #   Set PID Gains
    cmd_msg = CommandMsg()
    cmd_msg.name = hebi_mapping
    cmd_msg.settings.name = hebi_mapping
    cmd_msg.settings.position_gains.name = hebi_mapping
    cmd_msg.settings.position_gains.kp = [5, 8, 2]
    cmd_msg.settings.position_gains.ki = [0.001, 0.001, 0.001]
    cmd_msg.settings.position_gains.kd = [0, 0, 0]
    cmd_msg.settings.position_gains.i_clamp = [0.25, 0.25, 0.25]  # TODO: Tune me.
    hebi_wrap.send_command_with_acknowledgement(cmd_msg)

    # Get URDF
    urdf_str = ""
    urdf_loaded = False
    while not rospy.is_shutdown() and not urdf_loaded:
        if rospy.has_param('robot_description'):
            urdf_str = rospy.get_param('robot_description')
            urdf_loaded = True
            rospy.loginfo("Pulled {} from parameter server.".format(rospy.resolve_name('robot_description')))
        else:
            rospy.sleep(0.01)  # sleep for 10 ms of ROS time

    ### CREATE LEG STATE INSTANCES ###
    wait_for_master = WaitForMaster(hebiros_wrapper=hebi_wrap, from_master_topic=from_master_topic, to_master_topic=to_master_topic)
    step = Step(hebiros_wrapper=hebi_wrap, urdf_str=urdf_str, base_link=base_link_name, end_link=end_link_name, step_height=step_height)
    push = Push(hebiros_wrapper=hebi_wrap, urdf_str=urdf_str, base_link=base_link_name, end_link=end_link_name)

    ### CREATE TOP SM ###
    top = StateMachine(outcomes=['exit','success'])

    ### INITIALIZE USERDATA ###
    top.userdata.prev_joint_pos = [0,0,0]
    top.userdata.target_end_link_point = None
    top.userdata.execution_time = None
    top.userdata.active_joints = None

    remapping_list = ['prev_joint_pos', 'prev_joint_pos', 'target_end_link_point', 'execution_time', 'active_joints']
    remapping_dict = {userdata: userdata for userdata in remapping_list}

    with top:
        StateMachine.add('WAIT_FOR_MASTER', wait_for_master,
                         transitions={'exit':'exit', 'step':'STEP', 'push':'PUSH'},
                         remapping=remapping_dict)

        StateMachine.add('STEP', step,
                         transitions={'ik_failed':'WAIT_FOR_MASTER', 'success':'WAIT_FOR_MASTER'},
                         remapping=remapping_dict)

        StateMachine.add('PUSH', push,
                         transitions={'ik_failed':'WAIT_FOR_MASTER', 'success':'WAIT_FOR_MASTER'},
                         remapping=remapping_dict)

    sis = smach_ros.IntrospectionServer(str(rospy.get_name()), top, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    #user_input = raw_input("Please press the 'Return/Enter' key to start executing \
    #     - pkg: hexapod_leg_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    print("Input received. Executing...")
    top.execute()

    rospy.spin()
    sis.stop()
    print("\nExiting " + str(rospy.get_name()))


if __name__ == '__main__':
    main()
