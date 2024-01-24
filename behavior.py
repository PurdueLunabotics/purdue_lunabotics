import rospy
import smach

class ascent(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                outcomese=['init mapping', 'traversal to mining', 'traversal to berm', 'fail'])
                
    
    def execute (self, behavior):
        # extend linear actuators
        if True:
            return 'pass'
        else:
            return 'fail'
        
class init_mapping(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                outcomes=['pass', 'fail'])
    
    def execute (self, behavior):
        # sping 360 & find april tag & set target

        # generatte trajectory

        if True:
            return 'pass'
        else:
            return 'fail'

class traversal_to_mining(smach.State):
    def __init__(self):
        smach.State.__inti__(self,
                outcomes=['pass', 'fail'])
        
    def execute (self, behavior):
        # generate trajectory

        # move along trajectory + recieve sensor input(imu, uwb, images) + update map

        if True:
            return 'pass'
        else:
            return 'fail'
        
class traversal_to_berm(smach.State):
    def __init__(self):
        smach.State.__inti__(self,
                outcomes=['target reached', 'detect robot stuck'])
        
    def execute (self, behavior):
        # generate trajectory

        # move along trajectory + recieve sensor input(imu, uwb, images) + update map

        if True:
            return 'pass'
        else:
            return 'fail'
        
class plunging(smach.Sate):
    def __init__(self):
        smach.State.__init__(self,
                outcomes=['pass', 'fail'])
    
    def execute (self, behavior):
        # Set excavation to full speed + lower 90% of distance

        # reduce speed to 25%

        if True:
            return 'pass'
        else:
            return 'fail'

class trenching(smach.Sate):
    def __init__(self):
        smach.State.__init__(self,
                outcomes=['load cell full', 'Obstacle reached', 'fail'])
    
    def execute (self, behavior):
        # drive forward + adjust speed

        if True:
            return 'pass'
        else:
            return 'fail'
        
class deposit(smach.Sate):
    def __init__(self):
        smach.State.__init__(self,
                outcomes=['load cell empty', 'fail':'FUCKED'])
    
    def execute (self, behavior):
        # spin auger

        # stop spinning auger

        if True:
            return 'pass'
        else:
            return 'fail'
        
class post_deposit(smach.Sate):
    def __init__(self):
        smach.State.__init__(self,
                outcomes=['pass', 'fail'])
    
    def execute (self, behavior):
        # shift future deposition location

        if True:
            return 'pass'
        else:
            return 'fail'
        
class escape(smach.Sate):
    def __init__(self):
        smach.State.__init__(self,
                outcomes=['back to traversal to mining', 'back to traversal to berm', 'detect robot stuck':'ESCAPE'])
    
    def execute (self, behavior):
        # drive stright at 100% speed

        if True:
            return 'pass'
        else:
            return 'fail'


def main():
    rospy.init_node('smach_example_state_machine')

    sm = smach.StateMachine(outcomes = ['PASS', 'FAIL'])

    with sm:
        smach.StateMachine.add('ASCENT', ascent(), 
                               transitions={'init mapping':'INIT_MAPPING',
                                            'traversal to mining':'TRAVERSAL_TO_MINING',
                                            'traversal to berm':'TRAVERSAL_TO_BERM',
                                             'fail':'FUCKED'})
        smach.StateMachine.add('INIT_MAPPING', init_mapping(), 
                               transitions={'pass':'TRAVERSAL_TO_MINING', 
                                             'fail':'FUCKED'})
        smach.StateMachine.add('TRAVERSAL_TO_MINING', traversal_to_mining(),
                               transitions={'pass':'PLUGING',
                                             'fail':'ESCAPE'})
        smach.StateMachine.add('PLUGING', plunging(), 
                               transitions={'pass':'TRENCHING', 
                                             'fail':'FUCKED'})
        smach.StateMachine.add('TRENCHING', trenching(), 
                               transitions={'load cell full':'ASCENT',
                                            'Obstacle reached':'ASCENT', 
                                             'fail':'FUCKED'})
        smach.StateMachine.add('TRAVERSAL_TO_BERM', traversal_to_berm(), 
                               transitions={'target reached':'DEPOSIT', 
                                            'detect robot stuck':'ESCAPE'})
        smach.StateMachine.add('DEPOSIT', deposit(), 
                               transitions={'load cell empty':'POST_DEPOSIT', 
                                            'fail':'FUCKED'})
        smach.StateMachine.add('POST_DEPOSIT', post_deposit(), 
                               transitions={'pass':'ASCENT', 
                                            'fail':'FUCKED'})
        smach.StateMachine.add('ESCAPE', escape(), 
                               transitions={'back to traversal to mining':'TRAVERSAL_TO_MINING',
                                            'back to traversal to berm':'TRAVERSAL_TO_BERM', 
                                            'detect robot stuck':'ESCAPE'})
        
        
        