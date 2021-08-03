#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None    
        self.stopline_wp_index = -1 # wp Index of the point where velocity has to go to zero

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()
        
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints :
                close_waypoint_index = self.get_close_waypoint_id()
                self.publish_waypoints(close_waypoint_index)
            rate.sleep()
                
    
    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d : 
            self.waypoints_2d = [[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
    
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_index = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
        
    def get_close_waypoint_id(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        
        close_id_x = self.waypoint_tree.query([x,y],k=1)[1]
        close_coord = self.waypoints_2d[close_id_x]
        prev_close_coord = self.waypoints_2d[close_id_x - 1]
        
        # Equation of hyper plane
        cl_vect = np.array (close_coord)
        prev_cl_vect = np.array (prev_close_coord)
        current_vect = np.array([x,y])
        
        dot_op_result = np.dot(cl_vect - prev_cl_vect, current_vect - cl_vect)
        
        if dot_op_result > 0 :
            close_id_x = (close_id_x + 1 ) % len(self.waypoints_2d)
        return close_id_x
        
        
        
    def publish_waypoints(self, close_id):
        lane = Lane()
        #lane.header = self.base_waypoints.header
        #lane.waypoints = self.base_waypoints.waypoints[close_id : close_id + LOOKAHEAD_WPS]
        lane = self.generate_waypoints_with_velocity()
        self.final_waypoints_pub.publish(lane)
     
    def generate_waypoints_with_velocity(self):
        # Get relevant base points
        close_id = self.get_close_waypoint_id()
        base_pts = self.base_waypoints.waypoints[close_id : close_id + LOOKAHEAD_WPS] # Slice base waypoints before updating velocity
        
        # Create new lane with new velocities
        lane = Lane()
        lane.header = self.base_waypoints.header
        
        
        if (self.stopline_wp_index >= (close_id + LOOKAHEAD_WPS)) or (self.stopline_wp_index == -1):
            # Condition for normal travel
            lane.waypoints = base_pts
        else:
            # Condition when Traffic signal is red 
            lane.waypoints = self.decelerate_waypoints(base_pts,close_id)
        
        return lane

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self,waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
        
    def decelerate_waypoints(self, waypoints, close_id_idx):
        new_waypoints = []
        for i, wp in enumerate(waypoints):
            
            stopline_idx = max(self.stopline_wp_index - close_id_idx - 2, 0) # car stops 2 Waypts before the stop line
            
            dist = self.distance(waypoints, i, close_id_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            
            wp_new = Waypoint()
            wp_new.pose = wp.pose
            
            new_velocity = min (vel, self.get_waypoint_velocity(wp.new_pose)) # Calculated velocity is too high , then use default velocity
            wp_new.twist.twist.linear.x = new_velocity
            
            new_waypoints.append(wp_new)
            
        return new_waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
