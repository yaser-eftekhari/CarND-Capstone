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
    def _log(self, msg):
        if self.logEnable:
            rospy.logwarn(msg)

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32 , self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.waypoints_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.logEnable = False
        self.base_wp_list = None
        self.traffic_wp_list = None
        self.curr_pose = None
        self.curr_pose_wp = -1
        self.curr_speed = 30
        self.red_light_wp = -1
        self.prev_red_light_wp = -1
        self.loopEnable = False

        sys.stdout.flush()
        rospy.spin()

    # optimized search used in tl_detector
    def get_closest_waypoint(self, pose, start_idx=0, max_dist=0):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        mindl = 10000000
        minidx = -1
        idx = start_idx
        dist_sum = 0
        #optimize to binary search TBD
        for a_idx in range(len(self.base_wp_list)):
            last_idx = idx
            idx = (a_idx + start_idx) % len(self.base_wp_list) 
            dist = dl(pose.pose.position, self.base_wp_list[idx].pose.pose.position)
            dist_sum += dl(self.base_wp_list[last_idx].pose.pose.position, self.base_wp_list[idx].pose.pose.position)
            if mindl > dist:
                minidx = idx
                mindl = dist
                if mindl<2.:
                    break
            if max_dist>0 and dist_sum>max_dist:
                return -1
        minidx = (minidx + 1) % len(self.base_wp_list)
        return minidx    

    def get_base_idx(self, curr_pose):
        """Finds the waypoint index corresponding to the current car location

        Args:
            curr_pose (Current Position): current position of the car

        """
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        mindl = 100000000
        minidx = 0
        # optimize to binary seach TBD
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
        last_i = curr_i    
        while d < dist:
            d += dl(self.base_wp_list[last_i].pose.pose.position, self.base_wp_list[i].pose.pose.position)
            last_i = i
            i = (i + 1) % len(self.base_wp_list)
        return i        

    def get_rough_path(self, curr_pose, curr_i, off_i):
        """Set the speed on a range of waypoints

        Args:
            curr_i (Current Position): current position of the car in waypoint scale
            off_i : offset of waypoints from current position

        """
        path = Lane()
        if off_i < curr_i:
            off_i += len(self.base_wp_list)

        for idx in range(curr_i, off_i): 
            i = idx % len(self.base_wp_list)
            wp = deepcopy(self.base_wp_list[i])
            if self.loopEnable and self.base_wp_list[i].twist.twist.linear.x==0:
                self.base_wp_list[i].twist.twist.linear.x = 2. # avoid stopping the car with 0 target velocity if loop enabled
            if self.red_light_wp != -1:
                if idx >= self.red_light_wp:
                    wp.twist.twist.linear.x = 0
                else:
                    # set minimum speed to 4 to avoid slowly down too much before TL and avoid 2 stops.
                    wp.twist.twist.linear.x = max(4, self.base_wp_list[self.detect_red_wp].twist.twist.linear.x - 
                                                    (i - self.detect_red_wp )* self.base_wp_list[self.detect_red_wp].twist.twist.linear.x
                                                                /max(1, (self.red_light_wp -1 - self.detect_red_wp )) )

            path.waypoints.append(wp)
        # self._log('pos {} orient {}'.format(path.waypoints[0].pose.pose.position,path.waypoints[0].pose.pose.orientation))
        return path
    

    def pose_cb(self, msg):
        """Callback function for when a car position is received

        Args:
            msg (current_pose): position of the car

        """
        # Implement
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
        #curr_i = self.get_base_idx(self.curr_pose)
        curr_i = self.get_closest_waypoint(self.curr_pose, start_idx=self.curr_pose_wp)
        self.curr_pose_wp = curr_i
        lookahead_dist = max(1.5* self.get_waypoint_velocity(self.base_wp_list[curr_i]), 15)
        off_i = self.get_base_off_idx(curr_i, lookahead_dist) # 15m gets more points. TBD Spline
        final_wp = self.get_rough_path(self.curr_pose, curr_i, off_i)
        self.final_waypoints_pub.publish(final_wp)

    def waypoints_cb(self, waypoints):
        """Callback function for when all waypoints are received at the beginning of time

        Args:
            waypoints (base_waypoints): array of all waypoints on the track

        """
        # Implement
        self._log('Got waypoints_cb of size {}'.format(len(waypoints.waypoints)))
        self.base_wp_list = waypoints.waypoints
        self.traffic_wp_list = deepcopy(waypoints.waypoints)

    def traffic_cb(self, msg):
        """Callback function for when a traffic signal is received

        Args:
            msg (traffic_waypoint): location and status of the upcoming traffic light

        """
        # TODO: Callback for /traffic_waypoint message. Implement
        if self.red_light_wp == -1 and msg.data != -1:
            if self.prev_red_light_wp == msg.data: # dont stop at same stop twice
                return
            # do following check in tl_detector.py
            #dist = self.distance(self.curr_pose_wp, self.red_light_wp)
            #if dist < self.base_wp_list[self.curr_pose_wp].twist.twist.linear.x: #if stop less than threshold, dont stop
            #    return
            self.detect_red_wp = self.curr_pose_wp
            self.red_light_wp = int(msg.data)
        elif self.red_light_wp != -1 and msg.data == -1:
            self.prev_red_light_wp = self.red_light_wp
            self.red_light_wp = -1

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        if wp2 < wp1:
            r = range(wp1, len(self.base_wp_list))
            r.extend(range(0,wp2+1))
        else:
            r = range(wp1, wp2+1)
        for i in r:
            dist += dl(self.base_wp_list[wp1].pose.pose.position, self.base_wp_list[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
