from smach import State


class Breakpoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])

    def execute(self, ud):
        userinput = raw_input("Breakpoint reached. Press Enter to continue.")
        return 'done'
