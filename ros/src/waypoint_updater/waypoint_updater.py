#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np

import math

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0

class WaypointUpdater(object):
    def __init__(self):
        # initialise this node
        rospy.init_node('waypoint_updater')

        # define the subscribers and the corresponding callback function
        # /current_pose represents the EGO position
        # /base_waypoints represent
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Initialize the members
        self.car_pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = 0

        # loop to suspends itself every 50ms TODO calibrate
        self.loop()

    def loop(self):
        # publish next waypoints every 50 ms TODO 

        # define the frequency of publish
        execution_freq = rospy.Rate(50)  # TODO 

        # check if roscore is still active
        while not rospy.is_shutdown():
            # asynchronous thread execution: check if variables have been initialized
            # publish closest waypoint
            if not self.car_pose is None and not self.base_waypoints is None and not self.waypoint_tree is None:
                next_wpt_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(next_wpt_idx)
            # got to sleep for 20ms
            execution_freq.sleep()

    def get_closest_waypoint_idx(self):
        x = self.car_pose.pose.position.x
        y = self.car_pose.pose.position.y

        # Check if closest waypoint in ahead or behind the car
        closest_index = self.waypoint_tree.query([x, y], 1)[1]
        closest_waypoint_in_2d = self.waypoints_2d[closest_index]
        prev_waypoint_in_2d = self.waypoints_2d[closest_index - 1]

        # vectors using 2d coordinates
        cl_vector = np.array([x, y])
        previous_waypoint_vector = np.array(prev_waypoint_in_2d)
        closest_waypoint_vector = np.array(closest_waypoint_in_2d)

        # hyperplane intersection
        intersection = np.dot(closest_waypoint_vector - previous_waypoint_vector, cl_vector - closest_waypoint_vector)

        # check if found waypoint is behind the vehicle
        if intersection > 0:
            # use next waypoint
            closest_index = (closest_index + 1) % len(self.waypoints_2d)
        return closest_index

    def pose_cb(self, msg):
        self.car_pose = msg

    def publish_waypoints(self, nxt_idx):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()
        # header retained
        lane.header = self.base_waypoints.header
        # get 200 waypoints starting from closest
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        decel_waypoints = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            # Stop 2 waypoints back from line so front of car stops at line
            stop_idx = max(self.stopline_wp_idx - closest_idx - 3, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel == 0.

            # make sure not to go faster than speed limit (already considered in wp.twist.twist.linear.x)
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            decel_waypoints.append(p)

        return decel_waypoints

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
