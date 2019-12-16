from smach import State


class SetTargetJointStates(State):
    def __init__(self, target_joint_states):
        State.__init__(self, outcomes=['done'],
                       output_keys=['target_joint_states'])

        self.active = False
        self.target_joint_states = target_joint_states

    def enter(self, ud):
        self.active = True

    def execute(self, ud):
        self.enter(ud)

        ud.target_joint_states = self.target_joint_states

        return self.exit(ud, 'done')

    def exit(self, ud, outcome):
        self.active = False
        return outcome