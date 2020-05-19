# BEGIN ALL
#!/usr/bin/env python

import rospy
# BEGIN PART_1
from smach import State,StateMachine
# END PART_1

from time import sleep


# BEGIN PART_2
class One(State):
    def __init__(self):
        State.__init__(self, outcomes=['go','back'])

    def execute(self, userdata):
        print("one")
	conditions=input("input cmd:")
        sleep(1)
        return conditions
# END PART_2

class Two(State):
    def __init__(self):
        State.__init__(self, outcomes=['go','back'])

    def execute(self, userdata):
        print("two")
	conditions=input("input cmd:")
        sleep(1)
        return conditions


class Three(State):
    def __init__(self):
        State.__init__(self, outcomes=['go','back'])

    def execute(self, userdata):
        print("three")
	conditions=input("input cmd:")
        sleep(1)
        return conditions


if __name__ == '__main__':
# BEGIN PART_3
    sm = StateMachine(outcomes=['go','back'])
    with sm:
        StateMachine.add('ONE', One(), transitions={'go' : 'TWO' ,'back':'THREE'})
        StateMachine.add('TWO', Two(), transitions={'go' : 'TWO' ,'back':'THREE'})
	StateMachine.add('THREE', Three(), transitions={'go' : 'TWO' ,'back':'THREE'})
        
    sm.execute()
# END PART_3
# END ALL
