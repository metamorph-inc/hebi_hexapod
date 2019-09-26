#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import argparse

import rospy
from geometry_msgs.msg import Twist
from smach import StateMachine
import smach_ros

# SMACH Debug States
from common_states.breakpoint import Breakpoint

# SMACH Common States
from common_states.get_urdf_str_from_parameter_server import GetURDFStrFromParameterServer
from common_states.check_ros_topic_for_new_msg import CheckRosTopicForNewMsg
from common_states.get_transform_matrix_from_twist_msg import GetTransformMatrixFromTwistMsg
from common_states.check_if_twist_msg_is_zero import CheckIfTwistMsgIsZero
from common_states.get_static_transforms_from_urdf_str import GetStaticTransformsFromURDFStr
from common_states.toggle import Toggle

# SMACH Hexapod States
from hexapod_move_base_states.get_leg_eff_translations import GetLegEffTranslations
from hexapod_move_base_states.get_leg_eff_pos_targets_from_translations import GetLegEffPosTargetsFromTranslations
from hexapod_move_base_states.send_leg_cmds_and_wait import SendLegCmdsAndWait


def parse_args(args):
    """
    :type args: list
    :param args:

    :rtype:
    :return:
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-base_link_name', type=str, required=True, help="")
    parser.add_argument('-to_leg_1_topic', type=str, required=True, help="")
    parser.add_argument('-from_leg_1_topic', type=str, required=True, help="")
    parser.add_argument('-leg_1_base_link_name', type=str, required=True, help="")
    parser.add_argument('-to_leg_2_topic', type=str, required=True, help="")
    parser.add_argument('-from_leg_2_topic', type=str, required=True, help="")
    parser.add_argument('-leg_2_base_link_name', type=str, required=True, help="")
    parser.add_argument('-to_leg_3_topic', type=str, required=True, help="")
    parser.add_argument('-from_leg_3_topic', type=str, required=True, help="")
    parser.add_argument('-leg_3_base_link_name', type=str, required=True, help="")
    parser.add_argument('-to_leg_4_topic', type=str, required=True, help="")
    parser.add_argument('-from_leg_4_topic', type=str, required=True, help="")
    parser.add_argument('-leg_4_base_link_name', type=str, required=True, help="")
    parser.add_argument('-to_leg_5_topic', type=str, required=True, help="")
    parser.add_argument('-from_leg_5_topic', type=str, required=True, help="")
    parser.add_argument('-leg_5_base_link_name', type=str, required=True, help="")
    parser.add_argument('-to_leg_6_topic', type=str, required=True, help="")
    parser.add_argument('-from_leg_6_topic', type=str, required=True, help="")
    parser.add_argument('-leg_6_base_link_name', type=str, required=True, help="")
    parser.add_argument('-from_cmd_vel_twist_topic', type=str, required=True, help="")

    parser.add_argument('-linear_step_distance', type=float, default=0.075, help="[m]")
    parser.add_argument('-plant_height', type=float, default=0.217,  help="[m]")
    parser.add_argument('-step_period', type=float, default=1, help="[s]")
    parser.add_argument('-push_period', type=float, default=0.5, help="[s]")

    parser.add_argument('ros_args_name', type=str, nargs='?', default='no_ros_args')
    parser.add_argument('ros_args_log', type=str, nargs='?', default='no_ros_args')
    return parser.parse_args()


def main():
    # ROS stuff
    rospy.init_node('hexapod_move_base_fsm_node', anonymous=True)

    # Parse node args
    parser = parse_args(sys.argv[1:])
    base_link_name = parser.base_link_name
    to_leg_1_topic = parser.to_leg_1_topic
    from_leg_1_topic = parser.from_leg_1_topic
    leg_1_base_link_name = parser.leg_1_base_link_name
    to_leg_2_topic = parser.to_leg_2_topic
    from_leg_2_topic = parser.from_leg_2_topic
    leg_2_base_link_name = parser.leg_2_base_link_name
    to_leg_3_topic = parser.to_leg_3_topic
    from_leg_3_topic = parser.from_leg_3_topic
    leg_3_base_link_name = parser.leg_3_base_link_name
    to_leg_4_topic = parser.to_leg_4_topic
    from_leg_4_topic = parser.from_leg_4_topic
    leg_4_base_link_name = parser.leg_4_base_link_name
    to_leg_5_topic = parser.to_leg_5_topic
    from_leg_5_topic = parser.from_leg_5_topic
    leg_5_base_link_name = parser.leg_5_base_link_name
    to_leg_6_topic = parser.to_leg_6_topic
    from_leg_6_topic = parser.from_leg_6_topic
    leg_6_base_link_name = parser.leg_6_base_link_name
    from_cmd_vel_twist_topic = parser.from_cmd_vel_twist_topic

    linear_step_distance = parser.linear_step_distance
    plant_height = parser.plant_height
    step_period = parser.step_period
    push_period = parser.push_period
    print(linear_step_distance)
    print(plant_height)
    print(step_period)
    print(push_period)

    ### CREATE STATE INSTANCES ###
    # Either here or while populating state containers

    ### CREATE TOP SM ###
    top = StateMachine(outcomes=['exit'])

    ### INITIALIZE USERDATA ###
    #                                                        ODD LEGS                       EVEN LEGS
    #                                               X       Y       Z               X       Y       Z
    top.userdata.leg_eff_pos_setup =   [[plant_height, -0.042, +0.000], [plant_height, +0.042, +0.000]] * 3
    top.userdata.leg_eff_pos_home =    [[plant_height, -0.042, -0.226], [plant_height, +0.042, -0.226]] * 3
    top.userdata.leg_eff_pos_current = [[plant_height, -0.042, -0.226], [plant_height, +0.042, -0.226]] * 3
    top.userdata.leg_eff_translations = None

    # Only used for bookkeeping right now
    remapping_list = ['urdf_str', 'cmd_twist_msg',
                      'move_base_to_leg_base_transforms', 'move_base_transform',
                      'leg_eff_pos_home', 'leg_eff_pos_current', 'leg_eff_translations']

    with top:

        ## SETUP ##
        get_urdf_str_from_parameter_server = GetURDFStrFromParameterServer()
        StateMachine.add('GET_URDF_STR_FROM_PARAM_SERVER', get_urdf_str_from_parameter_server,
                         transitions={'done':'GET_MOVE_BASE_TO_LEG_BASE_TRANSFORMS'},
                         remapping={'urdf_str':'urdf_str'})

        get_move_base_to_leg_base_transforms = GetStaticTransformsFromURDFStr(base_links=[base_link_name],
                                                                              end_links=[leg_1_base_link_name,leg_2_base_link_name,leg_3_base_link_name,leg_4_base_link_name,leg_5_base_link_name,leg_6_base_link_name])
        StateMachine.add('GET_MOVE_BASE_TO_LEG_BASE_TRANSFORMS', get_move_base_to_leg_base_transforms,
                         transitions={'done':'GET_LEG_POS_FROM_TRANSLATIONS_SETUP'},
                         remapping={'urdf_str':'urdf_str',
                                    'transforms':'move_base_to_leg_base_transforms'})

        ## STAND UP ##
        get_leg_eff_pos_from_translations_setup = GetLegEffPosTargetsFromTranslations(translation_signs=['home', 'home', 'home', 'home', 'home', 'home'])
        StateMachine.add('GET_LEG_POS_FROM_TRANSLATIONS_SETUP', get_leg_eff_pos_from_translations_setup,
                         transitions={'done':'SEND_LEG_CMDS_AND_WAIT_SETUP'},
                         remapping={'leg_eff_translations':'leg_eff_translations',
                                    'leg_eff_pos_home':'leg_eff_pos_setup',
                                    'leg_eff_pos_current':'leg_eff_pos_current',
                                    'leg_eff_pos_targets':'leg_eff_pos_targets'})

        send_leg_cmds_and_wait_setup = SendLegCmdsAndWait(
            leg_behaviors=['step', 'step', 'step', 'step', 'step', 'step'], time_to_execute=3.0,
            to_leg_topics=[to_leg_1_topic, to_leg_2_topic, to_leg_3_topic, to_leg_4_topic, to_leg_5_topic,
                           to_leg_6_topic],
            from_leg_topics=[from_leg_1_topic, from_leg_2_topic, from_leg_3_topic, from_leg_4_topic, from_leg_5_topic,
                             from_leg_6_topic])
        StateMachine.add('SEND_LEG_CMDS_AND_WAIT_SETUP', send_leg_cmds_and_wait_setup,
                         transitions={'done': 'GET_LEG_POS_FROM_TRANSLATIONS_STANDUP', 'response_timeout': 'GET_LEG_POS_FROM_TRANSLATIONS_STANDUP'},
                         remapping={'leg_eff_pos_targets':'leg_eff_pos_targets',
                                    'leg_eff_pos_current':'leg_eff_pos_current'})

        get_leg_eff_pos_from_translations_standup = GetLegEffPosTargetsFromTranslations(translation_signs=['home', 'home', 'home', 'home', 'home', 'home'])
        StateMachine.add('GET_LEG_POS_FROM_TRANSLATIONS_STANDUP', get_leg_eff_pos_from_translations_standup,
                         transitions={'done':'SEND_LEG_CMDS_AND_WAIT_STANDUP'},
                         remapping={'leg_eff_translations':'leg_eff_translations',
                                    'leg_eff_pos_home':'leg_eff_pos_home',
                                    'leg_eff_pos_current':'leg_eff_pos_current',
                                    'leg_eff_pos_targets':'leg_eff_pos_targets'})

        send_leg_cmds_and_wait_standup = SendLegCmdsAndWait(
            leg_behaviors=['push', 'push', 'push', 'push', 'push', 'push'], time_to_execute=2.5,
            to_leg_topics=[to_leg_1_topic, to_leg_2_topic, to_leg_3_topic, to_leg_4_topic, to_leg_5_topic,
                           to_leg_6_topic],
            from_leg_topics=[from_leg_1_topic, from_leg_2_topic, from_leg_3_topic, from_leg_4_topic, from_leg_5_topic,
                             from_leg_6_topic])
        StateMachine.add('SEND_LEG_CMDS_AND_WAIT_STANDUP', send_leg_cmds_and_wait_standup,
                         transitions={'done': 'CHECK_FOR_TWIST_MSG', 'response_timeout': 'CHECK_FOR_TWIST_MSG'},
                         remapping={'leg_eff_pos_targets':'leg_eff_pos_targets',
                                    'leg_eff_pos_current':'leg_eff_pos_current'})

        ## LOOP ##
        check_for_new_twist_msg = CheckRosTopicForNewMsg(topic_name=from_cmd_vel_twist_topic, data_type=Twist)
        StateMachine.add('CHECK_FOR_TWIST_MSG', check_for_new_twist_msg,
                         transitions={'true':'CHECK_IF_TWIST_MSG_IS_ZERO', 'false':'CHECK_FOR_TWIST_MSG'},
                         remapping={'new_msg':'cmd_twist_msg'})

        check_if_twist_msg_is_zero = CheckIfTwistMsgIsZero(lin_threshold=0.005, ang_threshold=0.01)
        StateMachine.add('CHECK_IF_TWIST_MSG_IS_ZERO', check_if_twist_msg_is_zero,
                         transitions={'true':'CHECK_FOR_TWIST_MSG', 'false':'GET_TRANSFORM_FROM_TWIST_MSG'},
                         remapping={'twist_msg':'cmd_twist_msg'})

        get_transform_mat_from_twist_msg = GetTransformMatrixFromTwistMsg(lin_displace_lmt=linear_step_distance, ang_displace_lmt=0.3, time_step=1)
        StateMachine.add('GET_TRANSFORM_FROM_TWIST_MSG', get_transform_mat_from_twist_msg,
                         transitions={'done':'GET_LEG_EFF_TRANSLATIONS'},
                         remapping={'twist_msg':'cmd_twist_msg',
                                    'transform_matrix':'move_base_transform'})

        get_leg_eff_translations = GetLegEffTranslations()
        StateMachine.add('GET_LEG_EFF_TRANSLATIONS', get_leg_eff_translations,
                         transitions={'done':'TOGGLE'},
                         remapping={'move_base_to_leg_base_transforms':'move_base_to_leg_base_transforms',
                                    'move_base_transform':'move_base_transform',
                                    'leg_eff_pos_current':'leg_eff_pos_current',
                                    'leg_eff_translations':'leg_eff_translations'})

        #StateMachine.add("BREAKPOINT", Breakpoint(), transitions={'done':'TOGGLE'})

        StateMachine.add('TOGGLE', Toggle(), transitions={'true':'GET_LEG_POS_FROM_TRANSLATIONS_ODD_STEP',
                                                          'false':'GET_LEG_POS_FROM_TRANSLATIONS_EVEN_STEP'})

        ## ODD ##
        get_leg_eff_pos_from_translations_odd_step = GetLegEffPosTargetsFromTranslations(translation_signs=['pos', None, None, 'pos', 'pos', None])
        StateMachine.add('GET_LEG_POS_FROM_TRANSLATIONS_ODD_STEP', get_leg_eff_pos_from_translations_odd_step,
                         transitions={'done':'SEND_LEG_CMDS_AND_WAIT_ODD_STEP'},
                         remapping={'leg_eff_translations':'leg_eff_translations',
                                    'leg_eff_pos_home':'leg_eff_pos_home',
                                    'leg_eff_pos_current':'leg_eff_pos_current',
                                    'leg_eff_pos_targets':'leg_eff_pos_targets'})

        send_leg_cmds_and_wait_odd_step = SendLegCmdsAndWait(leg_behaviors=['step', None, None, 'step', 'step', None], time_to_execute=step_period,
                                                             to_leg_topics=[to_leg_1_topic, to_leg_2_topic, to_leg_3_topic, to_leg_4_topic, to_leg_5_topic, to_leg_6_topic],
                                                             from_leg_topics=[from_leg_1_topic, from_leg_2_topic, from_leg_3_topic, from_leg_4_topic, from_leg_5_topic, from_leg_6_topic])
        StateMachine.add('SEND_LEG_CMDS_AND_WAIT_ODD_STEP', send_leg_cmds_and_wait_odd_step,
                         transitions={'done':'GET_LEG_POS_FROM_TRANSLATIONS_ODD_PUSH', 'response_timeout':'GET_LEG_POS_FROM_TRANSLATIONS_ODD_PUSH'},
                         remapping={'leg_eff_pos_targets':'leg_eff_pos_targets',
                                    'leg_eff_pos_current':'leg_eff_pos_current'})

        get_leg_eff_pos_from_translations_odd_push = GetLegEffPosTargetsFromTranslations(translation_signs=['home', 'neg', 'neg', 'home', 'home', 'neg'])
        StateMachine.add('GET_LEG_POS_FROM_TRANSLATIONS_ODD_PUSH', get_leg_eff_pos_from_translations_odd_push,
                         transitions={'done':'SEND_LEG_CMDS_AND_WAIT_ODD_PUSH'},
                         remapping={'leg_eff_translations':'leg_eff_translations',
                                    'leg_eff_pos_home':'leg_eff_pos_home',
                                    'leg_eff_pos_current':'leg_eff_pos_current',
                                    'leg_eff_pos_targets':'leg_eff_pos_targets'})

        send_leg_cmds_and_wait_odd_push = SendLegCmdsAndWait(leg_behaviors=['push', 'push', 'push', 'push', 'push', 'push'], time_to_execute=push_period,
                                                             to_leg_topics=[to_leg_1_topic, to_leg_2_topic, to_leg_3_topic, to_leg_4_topic, to_leg_5_topic, to_leg_6_topic],
                                                             from_leg_topics=[from_leg_1_topic, from_leg_2_topic, from_leg_3_topic, from_leg_4_topic, from_leg_5_topic, from_leg_6_topic])
        StateMachine.add('SEND_LEG_CMDS_AND_WAIT_ODD_PUSH', send_leg_cmds_and_wait_odd_push,
                         transitions={'done':'CHECK_FOR_TWIST_MSG', 'response_timeout':'CHECK_FOR_TWIST_MSG'},
                         remapping={'leg_eff_pos_targets':'leg_eff_pos_targets',
                                    'leg_eff_pos_current':'leg_eff_pos_current'})

        ## EVEN ##
        get_leg_eff_pos_from_translations_even_step = GetLegEffPosTargetsFromTranslations(translation_signs=[None, 'pos', 'pos', None, None, 'pos'])
        StateMachine.add('GET_LEG_POS_FROM_TRANSLATIONS_EVEN_STEP', get_leg_eff_pos_from_translations_even_step,
                         transitions={'done':'SEND_LEG_CMDS_AND_WAIT_EVEN_STEP'},
                         remapping={'leg_eff_translations':'leg_eff_translations',
                                    'leg_eff_pos_home':'leg_eff_pos_home',
                                    'leg_eff_pos_current':'leg_eff_pos_current',
                                    'leg_eff_pos_targets':'leg_eff_pos_targets'})

        send_leg_cmds_and_wait_even_step = SendLegCmdsAndWait(leg_behaviors=[None, 'step', 'step', None, None, 'step'], time_to_execute=step_period,
                                                              to_leg_topics=[to_leg_1_topic, to_leg_2_topic, to_leg_3_topic, to_leg_4_topic, to_leg_5_topic, to_leg_6_topic],
                                                              from_leg_topics=[from_leg_1_topic, from_leg_2_topic, from_leg_3_topic, from_leg_4_topic, from_leg_5_topic, from_leg_6_topic])
        StateMachine.add('SEND_LEG_CMDS_AND_WAIT_EVEN_STEP', send_leg_cmds_and_wait_even_step,
                         transitions={'done':'GET_LEG_POS_FROM_TRANSLATIONS_EVEN_PUSH', 'response_timeout':'GET_LEG_POS_FROM_TRANSLATIONS_EVEN_PUSH'},
                         remapping={'leg_eff_pos_targets':'leg_eff_pos_targets',
                                    'leg_eff_pos_current':'leg_eff_pos_current'})

        get_leg_eff_pos_from_translations_even_push = GetLegEffPosTargetsFromTranslations(translation_signs=['neg','home','home','neg','home','neg'])
        StateMachine.add('GET_LEG_POS_FROM_TRANSLATIONS_EVEN_PUSH', get_leg_eff_pos_from_translations_even_push,
                         transitions={'done':'SEND_LEG_CMDS_AND_WAIT_EVEN_PUSH'},
                         remapping={'leg_eff_translations':'leg_eff_translations',
                                    'leg_eff_pos_home':'leg_eff_pos_home',
                                    'leg_eff_pos_current':'leg_eff_pos_current',
                                    'leg_eff_pos_targets':'leg_eff_pos_targets'})

        send_leg_cmds_and_wait_even_push = SendLegCmdsAndWait(leg_behaviors=['push','push','push','push','push','push'], time_to_execute=push_period,
                                                              to_leg_topics=[to_leg_1_topic, to_leg_2_topic, to_leg_3_topic, to_leg_4_topic, to_leg_5_topic, to_leg_6_topic],
                                                              from_leg_topics=[from_leg_1_topic, from_leg_2_topic, from_leg_3_topic, from_leg_4_topic, from_leg_5_topic, from_leg_6_topic])
        StateMachine.add('SEND_LEG_CMDS_AND_WAIT_EVEN_PUSH', send_leg_cmds_and_wait_even_push,
                         transitions={'done':'CHECK_FOR_TWIST_MSG', 'response_timeout':'CHECK_FOR_TWIST_MSG'},
                         remapping={'leg_eff_pos_targets':'leg_eff_pos_targets',
                                    'leg_eff_pos_current':'leg_eff_pos_current'})

    sis = smach_ros.IntrospectionServer(str(rospy.get_name()), top, '/SM_ROOT')# + str(rospy.get_name()))
    sis.start()

    rospy.sleep(3)
    #user_input = raw_input("Please press the 'Return/Enter' key to start executing \
    #     - pkg: hexapod_move_base_fsm_node.py | node: " + str(rospy.get_name()) + "\n")
    #print("Input received. Executing...")
    top.execute()

    rospy.spin()
    sis.stop()
    print("\nExiting " + str(rospy.get_name()))


if __name__ == '__main__':
    main()
