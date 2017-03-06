#!/usr/bin/env python

import roslib; roslib.load_manifest('valve_task')
import rospy
import smach
import smach_ros
import time

class SearchForPanel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
    def execute(self, userdata):
        # Your state execution goes here
        print "Entered SearchForPanel state"
        time.sleep(2)
        if True:
            print "SearchForPanel succeeded" 
            return 'succeeded'
        else:
            print "SearchForPanel failed"
            return 'failed'
            

class ApproachPanel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
    def execute(self, userdata):
        print "Entered ApproachPanel state"
        time.sleep(2)
        # Your state execution goes here
        if True:
            print "ApproachPanel succeeded"
            return 'succeeded'
        else:
            print "ApproachPanel failed"
            return 'failed'

            
class FindWrenchSide(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        # Your state initialization goes here
        
    def execute(self, userdata):
        print "Entered FindWrenchSide state"
        time.sleep(2)
        # Your state execution goes here
        if True:
            print "FindWrenchSide succeeded"
            return 'succeeded'
        else:
            print "FindWrenchSide failed"
            return 'failed'

class AlignWithPanelFront(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        # Your state initialization goes here
        
    def execute(self, userdata):
        print "Entered AlignWithPanelFront state"
        time.sleep(2)
        # Your state execution goes here
        if True:
            print "AlignWithPanelFront succeeded"
            return 'succeeded'
        else:
            print "AlignWithPanelFront failed"
            return 'failed'
            
class PickWrench(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        # Your state initialization goes here
        
    def execute(self, userdata):
        print "Entered PickWrench state"
        time.sleep(2)
        # Your state execution goes here
        if True:
            print "PickWrench succeeded"
            return 'succeeded'
        else:
            print "PickWrench failed"
            return 'failed'
            
class AlignWithShaft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        # Your state initialization goes here
        
    def execute(self, userdata):
        print "Entered AlignWithShaft state"
        time.sleep(2)
        # Your state execution goes here
        if True:
            print "AlignWithShaft succeeded"
            return 'succeeded'
        else:
            print "AlignWithShaft failed"
            return 'failed'

class TurnShaft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        # Your state initialization goes here
        
    def execute(self, userdata):
        # Your state execution goes here
        print "Entered TurnShaft state"
        time.sleep(2)
        if True:
            print "TurnShaft succeeded"
            return 'succeeded'
        else:
            print "TurnShaft failed"
            return 'failed'

class GoPos(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        # Your state initialization goes here
        
    def execute(self, userdata):
        print "Entered GoPos state"
        time.sleep(2)
        # Your state execution goes here
        if True:
            print "GoPos succeeded"
            return 'succeeded'
        else:
            print "GoPos failed"
            return 'failed'




########################### states for panel detection ###########################

class DetectPanelLongRange(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        # Your state initialization goes here
        
    def execute(self, userdata):
        # Your state execution goes here
        print "Entered DetectPanelLongRange state"
        time.sleep(2)
        if True:
            print "DetectPanelLongRange succeeded"
            return 'succeeded'
        else:
            print "DetectPanelLongRange aborted"
            return 'aborted'

class DetectPanelMidRange(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        # Your state initialization goes here
        
    def execute(self, userdata):
        # Your state execution goes here
        print "Entered DetectPanelMidRange state"
        time.sleep(2)
        if True:
            print "DetectPanelFarRange succeeded"
            return 'succeeded'
        else:
            print "DetectPanelFarRange aborted"
            return 'aborted'

class DetectPanelShortRange(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        # Your state initialization goes here
        
    def execute(self, userdata):
        print "Entered DetectPanelShortRange state"
        time.sleep(2)
        # Your state execution goes here
        if True:
            print "DetectPanelFarRange succeeded"
            return 'succeeded'
        else:
            print "DetectPanelFarRange aborted"
            return 'aborted'


def main():
    rospy.init_node('task2_smach')


    # main state machine for task2
    task2_sm = smach.StateMachine(outcomes=['succeeded','failed'])
    with task2_sm:



        # Create the sub SMACH state machine 
        approach_panel_sub_sm = smach.StateMachine(outcomes=['succeeded','failed'])

        # Open the container 
        with approach_panel_sub_sm:

            # Add states to the container 
            smach.StateMachine.add('Detect Panel Long Range', DetectPanelLongRange(),
                                   transitions={'succeeded':'Go Pos Long Range',
                                                'preempted':'failed',
                                                'aborted':'failed'})
            smach.StateMachine.add('Go Pos Long Range', GoPos(),
                                   transitions={'succeeded':'Detect Panel Mid Range', 
                                                'failed':'failed'})
            smach.StateMachine.add('Detect Panel Mid Range', DetectPanelMidRange(),
                                   transitions={'succeeded':'Go Pos Mid Range',
                                                'preempted':'failed',
                                                'aborted':'failed'})
            smach.StateMachine.add('Go Pos Mid Range', GoPos(),
                                   transitions={'succeeded':'Detect Panel Short Range', 
                                                'failed':'failed'})
            smach.StateMachine.add('Detect Panel Short Range', DetectPanelShortRange(),
                                   transitions={'succeeded':'Adjust Pos in Front of Panel',
                                                'preempted':'failed',
                                                'aborted':'failed'})
            smach.StateMachine.add('Adjust Pos in Front of Panel', GoPos(),
                                   transitions={'succeeded':'succeeded', 
                                                'failed':'failed'})





        ################# top state machine starts here! ################
        smach.StateMachine.add('Search for panel', SearchForPanel(),
                               transitions={'succeeded':'Approach panel',
                                            'failed':'failed'})
        smach.StateMachine.add('Approach panel', approach_panel_sub_sm,
                               transitions={'succeeded':'Pick Wrench',
                                            'failed':'failed'})
        smach.StateMachine.add('Pick Wrench', PickWrench(),
                               transitions={'succeeded':'Turn Shaft',
                                            'failed':'failed'})
        smach.StateMachine.add('Turn Shaft', TurnShaft(),
                               transitions={'succeeded':'succeeded'})


        ################# end of top state machine! #####################





    # Execute SMACH plan
    outcome = task2_sm.execute()
        

if __name__ == '__main__':
    main()
