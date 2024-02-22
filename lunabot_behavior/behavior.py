import rospy

import ascent
import escape
import init_mapping


def main():
    rospy.init_node('behavior_machine')

    #create interrupt logic here!
    # - rospy.is_shutdown():
    # stuck
    # map change
    # overcurrent

    #startup stuff here
    ascent.main()
    init_mapping.main()

    while(True):
        #traverse to mining zone

        #plunge

        #trench

        ascent.main()

        #berm plan

        #go to berm

        #deposit

        #post_deposit

        ascent.main() #why?

        #plan to mining zone