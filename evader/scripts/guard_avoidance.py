#!/usr/bin/env python3
import numpy as np
import rogata_library as rgt
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion


class GuardAvoidance:
    def __init__(self):
        # lookup_table with points around the guards minmax position 
        # for the guard avoidance utility metric
        self.lookup_table = [np.array([0, 0])]
        # point circle 1 and 3
        for i in np.arange(0., 2. * np.pi, 2. * np.pi / 8.):
            for j in np.arange(0.5, 2.0, 1):
                new_point_x = j * np.cos(i)
                new_point_y = j * np.sin(i)
                new_point = np.array([new_point_x, new_point_y])
                self.lookup_table.append(new_point)
        # point circle 2        
        for i in np.arange(np.pi / 8, 2. * np.pi, 2. * np.pi / 8.):
            new_point_x = np.cos(i)
            new_point_y = np.sin(i)
            new_point = np.array([new_point_x, new_point_y])
            self.lookup_table.append(new_point)
            
        
        self.rogata = rgt.rogata_helper()   
        
        # evader state
        self.evader_pos = None
        self.evader_pose = None
        self.evader_twist = None
        # evader cmd_vels for treeplanner
        self.evader_cmd_vels = [np.array([0., 0.]), np.array([0., 0.]), np.array([0.22, -0.2]), np.array([0.22, 0.]), np.array([0.22, 0.2])]
        
        # nav_stack
        # twist
        self.nav_stack_movement = None
        self.nav_stack_cmd_vel = None  
        
        # threshold for the nav_stack guard avoidance utility
        self.utility_threshold = 0.8
        
        # flag for cmd_vel to be published
        self.drive_after_nav_stack = True
        
        # Publisher
        self.pub_cmd_vel = rospy.Publisher("/evader/cmd_vel", Twist, queue_size=1) 
                
        # Subscriber
        rospy.Subscriber("/evader/corrected_perception_of_guard", Odometry, self.corrected_perception_callback, queue_size=1)
        
        # uncomment this instead of /evader/corrected_perception_of_guard above to test behavior without perception noise
        # rospy.Subscriber("/guard/odom", Odometry, self.corrected_perception_callback, queue_size=1) 
        
        rospy.Subscriber("/evader/odom", Odometry, self.evader_odom_callback)
        rospy.Subscriber("/evader/nav_stack_cmd_vel", Twist, self.nav_stack_callback)       
        
        while not rospy.is_shutdown():
            # publish movement dependig on the utility of the nav_stack movement
            if self.drive_after_nav_stack:
                if self.nav_stack_movement is not None:
                    self.pub_cmd_vel.publish(self.nav_stack_movement)
            else:
                if self.evader_twist is not None:    
                    self.pub_cmd_vel.publish(self.evader_twist) 
   
   
    def evader_odom_callback(self, odom):
        # store evader pose and position
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        orientation = euler_from_quaternion([odom.pose.pose.orientation.x,
                                             odom.pose.pose.orientation.y,
                                             odom.pose.pose.orientation.z,
                                             odom.pose.pose.orientation.w])[2]
        self.evader_pose = np.array([x, y, orientation])
        self.evader_pos = np.array([x, y])   
      
        
    def nav_stack_callback(self, twist):
        # store nav_stack cmd_vel
        self.nav_stack_cmd_vel = np.array([twist.linear.x, twist.angular.z])
        self.nav_stack_movement = twist
    
    
    def corrected_perception_callback(self, odom):
        # save evader_pos to avoid different values during calculation
        evader_pos = self.evader_pos
        evader_pose = self.evader_pose
        
        if evader_pos is None or evader_pose is None:
            return
        
        # calculate best position of guard with minmax, assuming the guard knows 
        # the evaders correct position
        best_guard_pos, best_guard_pos_utility = self.min_max(odom, evader_pos)
        
        
        if self.nav_stack_cmd_vel is not None:
            # save nav_stack_cmd_vel to avoid different values during calculation
            self.evader_cmd_vels[0] = self.nav_stack_cmd_vel
        
        # calculate best possible movement for evader and its utility given the 
        # guards minmax position
        evader_cmd_vel = self.plan_evader_movement(evader_pose, best_guard_pos, best_guard_pos_utility)
        
        # set new evader cmd_vel to publish
        self.evader_twist = Twist()
        self.evader_twist.linear.x = evader_cmd_vel[0]
        self.evader_twist.angular.z = evader_cmd_vel[1]
        
        
    def min_max(self, odom, evader_pos):
        # save approximated guard position
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        app_guard_pos = np.array([x, y])
        
        # calculate a position above, right, below and left the guard
        guard_positions = [app_guard_pos,
                           app_guard_pos - np.array([0.5, 0]),
                           app_guard_pos + np.array([0.5, 0]),
                           app_guard_pos - np.array([0, 0.5]),
                           app_guard_pos + np.array([0, 0.5])]
        
        
        # utility and index of the best guard position
        max_utility = 0
        max_idx = 0
        
        # calculate the utility for the calculated guard positions 
        # and select the best point for the guard
        for idx, guard_pos in enumerate(guard_positions):
            utility = self.visibility_utility(guard_pos, evader_pos)
            if(max_utility < utility):
                max_utility = utility
                max_idx = idx
                
        best_guard_pos = guard_positions[max_idx]
        
        return best_guard_pos, max_utility   
   
   
    def plan_evader_movement(self, evader_pos, best_guard_pos, best_guard_pos_utility):
        # calculate possible evader movements using a treeplanner               
        evader_positions = self.treeplanner(evader_pos, self.evader_cmd_vels)           
        
        # utility and index of the best evader position given the guards min max position
        max_utility = 0
        max_idx = 0
        
        # evader position for rogata
        evader_pixel_pos = np.array([evader_pos[0], - evader_pos[1]]) * 100 + np.array([500,500])
        
        # calculate the utility for the calculated evader positions
        for idx, possible_evader_pos in enumerate(evader_positions):
            # nav_stack cmd_vel
            if idx == 0:
                # UTILITY COMBINE (reversed prevail)
                # SELECT THE NAVSTACK MOVEMENT/CMD_VEL IF THE POSSIBILITY
                # OF BEING SEEN THERE IS LOW / THE GUARD AVOIDANCE UTILITY IS HIGH
                utility = 1 - self.visibility_utility(best_guard_pos, possible_evader_pos)
                if utility > self.utility_threshold:
                    self.drive_after_nav_stack = True
                    return self.evader_cmd_vels[0]
                else:
                    self.drive_after_nav_stack = False
                    
            # current_position / standing still
            elif idx == 1: 
                utility = 1 - best_guard_pos_utility
            # utility calculation for selected cmd_vels
            else:
                # calculate the rogata pos for the treeplanner position
                possible_evader_pixel_pos = np.array([possible_evader_pos[0], - possible_evader_pos[1]]) * 100 + np.array([500,500])
                
                # simple COLLISION AVOIDANCE with line of sight
                if self.visibility(possible_evader_pixel_pos, evader_pixel_pos, ["walls_obj"], 1000, self.rogata):
                    utility = 1 - self.visibility_utility(best_guard_pos, possible_evader_pos)
                else:
                    utility = 0    
            
            # select the best point for the evader given the guards minmax position
            if(max_utility < utility):
                max_utility = utility
                max_idx = idx
        
        # best evader_cmd vel / evader cmd_vel with highest utility
        evader_cmd_vel = self.evader_cmd_vels[max_idx]
        
        return evader_cmd_vel
     
        
    # calculate points out of cmd_vels and current positions with Euler Method    
    def treeplanner(self, start_position, cmd_vels): 
        possible_positions = []
        for cmd_vel in cmd_vels:
            possible_positions.append(self.update_pose(start_position, cmd_vel))
        return possible_positions
    
    
    # implementation of Euler Method
    def update_pose(self, position, cmd_vel, iterations=50):
        dt = 2.0 / iterations
        u = cmd_vel
        x = position  
          
        # calculation of the new turtlebot position (Euler method)  
        for i in range(iterations):
            A = np.identity(3)
            B = np.array([[dt * np.cos(x[2]), 0],
                        [dt * np.sin(x[2]), 0],
                        [0.0, dt]])
            x = A.dot(x) + B.dot(u)
        
        return x


    # determine by how many points around the guard the evader can be seen
    # zero sum game for evader and guard 1 = good for guard
    #                                    0 = good for evader
    def visibility_utility(self, guard_pos, evader_pos):
        evader_pixel_pos = np.array([evader_pos[0], - evader_pos[1]]) * 100 + np.array([500,500])
        
        visible_points = 0
        # count at how many points around the guard the evader can see
        # using the line of sight calculation of the rogata engine
        for idx, point in enumerate(self.lookup_table):
            coord_pos = guard_pos + point
            pixel_pos = np.array([coord_pos[0], - coord_pos[1]]) * 100 + np.array([500,500])
            visible_points += self.visibility(pixel_pos, evader_pixel_pos, ["walls_obj"], 1000, self.rogata)
                
        utility = float(visible_points / len(self.lookup_table))
        return utility


    # line of sight function from the rogata engine tutorial
    def visibility(self, guard, thief, wall_objects, max_seeing_distance, rogata):
        distance   = np.linalg.norm(thief-guard)
        direction  = (thief-guard)/distance
        direction  = np.arctan2(direction[1],direction[0])
        
        min_intersect = guard + max_seeing_distance * np.array([np.cos(direction),np.sin(direction)])
        
        for walls in wall_objects:
            intersection = rogata.intersect(walls,guard,direction,max_seeing_distance)
            
            if np.linalg.norm(intersection-guard) <= np.linalg.norm(min_intersect-guard):
                min_intersect = intersection

        if np.linalg.norm(min_intersect-guard) >= distance:
            return 1
        else:
            return 0
            

if __name__ == '__main__':
    rospy.init_node("guard_avoidance")
    try:
        node = GuardAvoidance()
    except rospy.ROSInterruptException:
        pass

