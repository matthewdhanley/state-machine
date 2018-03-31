#!/usr/bin/env python
import smach
import smach_ros
import rospy
import motor_commands
import time
import robot_actions.msg


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
        time.sleep(1)
        return 'success'


class Dump(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('Dumping...')
        time.sleep(1)
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

        # define where we want to mine
        mining_goal = robot_actions.msg.move_baseGoal()
        mining_goal.location.y = 10
        mining_goal.location.x = 0

        # define where we want to mine
        dumping_goal = robot_actions.msg.move_baseGoal()
        mining_goal.location.y = 0
        mining_goal.location.x = 0

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

        # call move_base action!
        smach.StateMachine.add('GOTO_MINE_LOCATION',
                               smach_ros.SimpleActionState('move_base',
                                                           robot_actions.msg.move_baseAction,
                                                           goal=mining_goal),
                               transitions={
                                   'succeeded': 'MINE',
                                   'aborted': 'SELF_LOC',
                                   'preempted': 'SELF_LOC'
                               })

        # call move_base action!
        smach.StateMachine.add('GOTO_DUMP_LOCATION',
                               smach_ros.SimpleActionState('move_base',
                                                           robot_actions.msg.move_baseAction,
                                                           goal=dumping_goal),
                               transitions={
                                   'succeeded': 'MINE',
                                   'aborted': 'SELF_LOC',
                                   'preempted': 'SELF_LOC'
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