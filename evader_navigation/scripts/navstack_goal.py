#!/usr/bin/env python
import numpy as np
import rogata_library as rgt
import rospy
import tf_conversions
import time

from geometry_msgs.msg import PoseStamped, Quaternion


class NavigationStack:
    def __init__(self):
        # nav_stack goal pos publisher
        self.pub_nav_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped)
    
        self.rogata = rgt.rogata_helper()
        
        # get bounty position
        bounty_pixel_pos = self.rogata.get_pos('goal_obj')
        self.bounty_pos = (bounty_pixel_pos - np.array([500,500])) / 100 * np.array([1, -1])
        
        # publish bounty position
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal.pose.position.x = self.bounty_pos[0]
        self.goal.pose.position.y = self.bounty_pos[1]
        # orientation
        self.goal.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 1.57))
        
        
        while True:
            self.pub_nav_goal.publish(self.goal)
            evader = self.rogata.get_pos('evader_obj')
            # check if the evader got the bounty
            if self.rogata.inside("goal_obj", evader):
                # set the new goal position to entry_object
                exit_pixel_pos = self.rogata.get_pos('entry_obj')
                self.exit_pos = (exit_pixel_pos - np.array([500,500])) / 100 * np.array([1, -1])
                self.goal.pose.position.x = self.exit_pos[0]
                self.goal.pose.position.y = self.exit_pos[1]
            time.sleep(1)
    

if __name__ == "__main__":
    rospy.init_node("navigation_stack")
    try:
        node = NavigationStack()
    except rospy.ROSInterruptException:
        pass

