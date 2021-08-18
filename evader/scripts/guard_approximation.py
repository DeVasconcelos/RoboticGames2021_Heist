#!/usr/bin/env python3
import numpy as np
import rospy

from nav_msgs.msg import Odometry


class GuardApproximation:
    def __init__(self):
        # Subscriber
        rospy.Subscriber("/evader/perception_of_guard", Odometry, self.source_callback)
        
        # ring buffer to store guard positions
        self.old_percepted_guard_positions = list()
        self.max_array_size = 10
        self.index = 0
        
        # used for circle based approximation of the guards position
        # -> approximate the position in the intersect area of circles
        #    around the percepted guard positions
        self.guard_circle_radius = 1.5
        
        # Publisher
        self.pub = rospy.Publisher("/evader/corrected_perception_of_guard", Odometry, queue_size=1)
        self.new_corrected_guard_pos = None

        while not rospy.is_shutdown():
            pass
          

    def source_callback(self, odom):
        # current percepted guard position
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        current_percepted_guard_pos = np.array([x, y])
        
        
        if self.new_corrected_guard_pos is None:
            self.new_corrected_guard_pos = current_percepted_guard_pos
        
        # update old_percepted_guard_positions
        if len(self.old_percepted_guard_positions) <= self.max_array_size:
            self.old_percepted_guard_positions.append(current_percepted_guard_pos)
        else:
            self.old_percepted_guard_positions[self.index] = current_percepted_guard_pos
        
        
        i = (self.index + 1) % len(self.old_percepted_guard_positions)
        # calculation of new_corrected_guard_pos
        for i in range(len(self.old_percepted_guard_positions)):
            idx = (self.index + i + 1) % len(self.old_percepted_guard_positions)
            pos = self.old_percepted_guard_positions[idx]
            dist = np.linalg.norm(pos - self.new_corrected_guard_pos) * (1 + 0.4 * (len(self.old_percepted_guard_positions) - i))
            if dist > self.guard_circle_radius:
                delta = 1 - (self.guard_circle_radius / dist)
                self.new_corrected_guard_pos = (pos - self.new_corrected_guard_pos) * delta + self.new_corrected_guard_pos
                   
        # clip new_corrected_guard_pos
        self.new_corrected_guard_pos = np.clip(self.new_corrected_guard_pos, - 4.8, 4.8)
        
        # publish new_corrected_guard_pos to /evader/corrected_perception_of_guard    
        odom.pose.pose.position.x = self.new_corrected_guard_pos[0]
        self.pub.publish(odom)
        
        
        self.index = (self.index + 1) % (self.max_array_size + 1)
     
if __name__ == '__main__':
    rospy.init_node("guard_approximation")
    try:
        node = GuardApproximation()
    except rospy.ROSInterruptException:
        pass
