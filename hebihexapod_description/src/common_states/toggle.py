from smach import State


class Toggle(State):
    def __init__(self):
        State.__init__(self, outcomes=['true','false'])

        self.toggle = False

    def execute(self, ud):
        self.toggle = not self.toggle
        return str(self.toggle).lower()
