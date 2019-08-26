import rospy
from smach import State


class GetURDFStrFromParameterServer(State):
    def __init__(self, urdf_parameter_name='robot_description'):
        State.__init__(self, outcomes=['done'], output_keys=['urdf_str'])

        self.urdf_parameter_name = urdf_parameter_name

    def execute(self, ud):

        urdf_str = ""
        urdf_loaded = False
        while not rospy.is_shutdown() and not urdf_loaded:
            if rospy.has_param(self.urdf_parameter_name):
                urdf_str = rospy.get_param(self.urdf_parameter_name)
                urdf_loaded = True
                rospy.loginfo("Pulled {} from parameter server.".format(rospy.resolve_name(self.urdf_parameter_name)))
            else:
                rospy.sleep(0.01)  # sleep for 10 ms of ROS time

        ud.urdf_str = urdf_str

        return 'done'
