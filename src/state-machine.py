#!/usr/bin/env python
import smach
import rospy


# adding states to a state machine
# here we make the state machine and define the possible
# outcomes as outcome4 and outcome5
class BotSetup(smach.State):
    def __init__(self):
        # state initialization
        smach.State.__init__(self,
                             outcomes=['success', 'fail']
                             )
        
    def execute(self, userdata):
        # state execution
        rospy.loginfo("Initializing...")
        rospy.loginfo("X location: ")
        rospy.loginfo("Y location: ")
        if 1:
            return 'success'
        else:
            return 'fail'


class GotoMiningLocation(smach.State):
    def __init__(self):
        # state initialization
        smach.State.__init__(self,
                             outcomes=['success', 'fail']
                             )

    def execute(self, userdata):
        # state execution
        rospy.loginfo("Moving to mine...")
        rospy.loginfo("X location: ")
        rospy.loginfo("Y location: ")
        return 'success'


class GotoDumpingLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('Going to dump...')
        rospy.loginfo("X location: ")
        rospy.loginfo("Y location: ")
        return 'success'


class Mine(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('Mining...')
        rospy.loginfo("X location: ")
        rospy.loginfo("Y location: ")
        rospy.loginfo("Depth: ")
        rospy.loginfo("Tool RPM: ")
        rospy.loginfo("Elapsed time: ")
        return 'success'


class Dump(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('Dumping...')
        return 'success'


class Localize(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('Localizing...')
        return 'success'

    
class Safe(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('In Safe Mode...')
        return 'success'
    
    
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('In Safe Mode...')
        return 'success'


def main():
    rospy.loginfo('Starting Main...')
    rospy.init_node('state_machine')

    # creating a state
    sm = smach.StateMachine(outcomes=['sm_success', 'sm_fail'])
    with sm:
        # add the state to the state machine sm
        # first arguement is the label, FOO
        # second argument is the state, "Foo" as defined above
        # transistions states the possible outcomes
        # if outcome is outcome1, we transistion to state BAR.
        # if outcome is outcome2, the machine will exit with 'outcome4'
        smach.StateMachine.add('SETUP',
                               BotSetup(),
                               transitions={
                                   'success': 'IDLE',
                                   'fail': 'sm_fail'
                               })
        
        smach.StateMachine.add('GOTO_MINE_LOCATION',
                               GotoMiningLocation(),
                               transitions={
                                   'success': 'MINE',
                                   'fail': 'SELF_LOC'
                               })

        smach.StateMachine.add('GOTO_DUMP_LOCATION',
                               GotoDumpingLocation(),
                               transitions={
                                   'success': 'DUMP',
                                   'fail': 'SELF_LOC'
                               })

        smach.StateMachine.add('DUMP',
                               Dump(),
                               transitions={
                                   'success': 'GOTO_MINE_LOCATION',
                                   'fail': 'SAFE'
                               })

        smach.StateMachine.add('MINE',
                               Mine(),
                               transitions={
                                   'success': 'GOTO_DUMP_LOCATION',
                                   'fail': 'SAFE'
                               })

        smach.StateMachine.add('SELF_LOC',
                               Localize(),
                               transitions={
                                   'success': 'IDLE',
                                   'fail': 'SAFE'
                               })
        
        smach.StateMachine.add('SAFE',
                               Safe(),
                               transitions={
                                   'success': 'IDLE',
                                   'fail': 'sm_fail'
                               })
        
        smach.StateMachine.add('IDLE',
                               Idle(),
                               transitions={
                                   'success': 'MINE',
                                   'fail': 'sm_fail'
                               })

    outcome = sm.execute()
    rospy.loginfo('State Machine Done')
    rospy.loginfo(outcome)


if __name__=='__main__':
    main()