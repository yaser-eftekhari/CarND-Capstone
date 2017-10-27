#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from copy import copy,deepcopy
import sys

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32 , self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.waypoints_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_wp_list = None
        self.curr_pose = None
        self.curr_speed = 30

        rospy.logdebug('WaypointUpdater debug')
        rospy.loginfo('WaypointUpdater info')
        rospy.logwarn('WaypointUpdater warn')
        print('WaypointUpdater print')
        sys.stdout.flush()
        rospy.spin()

    def get_base_idx(self, curr_pose):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        mindl = 100000000
        minidx = 0
        #optimize to binary seach TBD
        for idx in range(len(self.base_wp_list)):
            dist = dl(curr_pose.pose.position, self.base_wp_list[idx].pose.pose.position)
            if mindl > dist:
                minidx = idx
                mindl = dist
        minidx = (minidx + 1) % len(self.base_wp_list)
        return minidx

    def get_base_off_idx(self, curr_i, dist):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        d = 0
        i = (curr_i + 1) % len(self.base_wp_list)
        while d < dist:
            d += dl(self.base_wp_list[curr_i].pose.pose.position, self.base_wp_list[i].pose.pose.position)
            i = (i + 1) % len(self.base_wp_list)
        return i        

    def get_rough_path(self, curr_i, off_i):
        path = Lane()
        if off_i < curr_i:
            off_i += len(self.base_wp_list)

        for i in range(curr_i, off_i): 
            i = i % len(self.base_wp_list)
            wp = deepcopy(self.base_wp_list[i])
            path.waypoints.append(wp)
        # rospy.logwarn('pos {} \norient {}'.format(path.waypoints[0].pose.pose.position, path.waypoints[0].pose.pose.orientation))
        return path

    def pose_cb(self, msg):
        # TODO: Implement
        # rospy.logwarn('Got pose {}'.format(msg))
        self.curr_pose = msg
        if self.base_wp_list is None:
            return
        '''
        curr_i = self.get_curr_idx(curr_pose, dist)
        get_pts for 30m
        calc spline
        calc WPS

        #tck = interpolate.splrep(xpts, ypts, s=1)
        #based on speed TBD store prev pts and 
        #xnew =  
        #ynew = interpolate.splev(xnew, tck, der=0)
        self.final_waypoints_pub.publish(final_wp)
        '''
        curr_i = self.get_base_idx(self.curr_pose)
        off_i = self.get_base_off_idx(curr_i, 30)
        final_wp = self.get_rough_path(curr_i, off_i)
        self.final_waypoints_pub.publish(final_wp)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.logwarn('Got waypoints_cb of size {}'.format(len(waypoints.waypoints)))
        self.base_wp_list = deepcopy(waypoints.waypoints)
        #rospy.logwarn ('{} : {} '.format(self.base_wp_count, waypoints))

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
