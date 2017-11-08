#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from copy import deepcopy
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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def _log(self, msg, force_enable = False):
        if force_enable or self.logEnable:
            rospy.logwarn(msg)

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32 , self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.waypoints_cb)

        # Adding Current velocity as well to be used to calculate the speed for final waypoints
        rospy.Subscriber('/current_velocity', TwistStamped, self.curr_vel_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.logEnable = False
        self.base_wp_list = None
        # self.traffic_wp_list = None
        self.curr_pose = None
        self.curr_pose_wp = -1
        self.curr_speed = 30
        self.red_light_wp = -1
        self.prev_red_light_wp = -1
        self.loopEnable = True

        self.total_base_wp = None
        self.prev_base_wp = 0
        self.current_velocity = None
        self.detect_red_wp = -1

        sys.stdout.flush()
        rospy.spin()
    

    def get_base_idx(self, curr_pose, prev_idx):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        mindl = None
        minidx = None

        for idx in range(prev_idx, prev_idx + self.total_base_wp):
            idx = idx % self.total_base_wp

            dist = dl(curr_pose.pose.position, self.base_wp_list[idx].pose.pose.position)

            if mindl is None:
                mindl = dist
                minidx = idx
            elif mindl > dist:
                minidx = idx
                mindl = dist

        minidx = (minidx + 1) % self.total_base_wp

        return minidx

    # def get_base_off_idx(self, curr_i, dist):
    #     dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
    #     d = 0
    #     i = (curr_i + 1) % len(self.base_wp_list)
    #     last_i = curr_i
    #     while d < dist:
    #         d += dl(self.base_wp_list[last_i].pose.pose.position, self.base_wp_list[i].pose.pose.position)
    #         last_i = i
    #         i = (i + 1) % len(self.base_wp_list)
    #     return i

    def clone_waypoint(self, wp):
        output = Waypoint()

        output.pose.pose.position.x = wp.pose.pose.position.x
        output.pose.pose.position.y = wp.pose.pose.position.y
        output.pose.pose.position.z = wp.pose.pose.position.z

        output.pose.pose.orientation.x = wp.pose.pose.orientation.x
        output.pose.pose.orientation.y = wp.pose.pose.orientation.y
        output.pose.pose.orientation.z = wp.pose.pose.orientation.z
        output.pose.pose.orientation.w = wp.pose.pose.orientation.w

        output.twist.twist.linear.x = wp.twist.twist.linear.x
        output.twist.twist.linear.y = wp.twist.twist.linear.y
        output.twist.twist.linear.z = wp.twist.twist.linear.z

        output.twist.twist.angular.x = wp.twist.twist.angular.x
        output.twist.twist.angular.y = wp.twist.twist.angular.y
        output.twist.twist.angular.z = wp.twist.twist.angular.z

        return output

    def clone_waypoints(self, start_idx, end_idx):
        output = []
        for idx in range(start_idx, end_idx):
            i = idx % self.total_base_wp
            output.append(self.clone_waypoint(self.base_wp_list[i]))

        return output


    def get_rough_path(self, curr_i):
        path = Lane()
        off_i = (curr_i + LOOKAHEAD_WPS)

        self._log('current index {} end index {}'.format(curr_i, off_i))

        prev_speed = self.base_wp_list[curr_i].twist.twist.linear.x

        # wp = self.clone_waypoints(curr_i, off_i)
        for idx in range(curr_i, off_i):
            i = idx % self.total_base_wp
            wp = self.clone_waypoint(self.base_wp_list[i])
            # if self.loopEnable and self.base_wp_list[i].twist.twist.linear.x==0:
            #     self.base_wp_list[i].twist.twist.linear.x = 1. # avoid stopping the car with 0 target velocity if loop enabled
            if self.red_light_wp != -1:
                if idx >= self.red_light_wp:
                    wp.twist.twist.linear.x = 0
                else:
                    wp.twist.twist.linear.x = max(2, self.base_wp_list[self.detect_red_wp].twist.twist.linear.x -
                                                    (i - self.detect_red_wp )* self.base_wp_list[self.detect_red_wp].twist.twist.linear.x
                                                                /max(1, (self.red_light_wp -1 - self.detect_red_wp )) )

        # self.adjust_speed(wp)

            path.waypoints.append(wp)

        # self._log('light {} length {} prev speed {} speed {}'.format(self.red_light_wp, len(wp), prev_speed, wp[0].twist.twist.linear.x), True)
        # self._log('pos {} orient {}'.format(path.waypoints[0].pose.pose.position,path.waypoints[0].pose.pose.orientation))
        return path
    
    def adjust_speed(self, original_wp):
        if self.red_light_wp == -1:
            return original_wp

        distance_wp = self.curr_pose_wp - self.red_light_wp
        speed_delta = self.current_velocity*1.0/max(distance_wp, 1)

        for idx in range(LOOKAHEAD_WPS):
            wp = original_wp[idx]
            if idx <= distance_wp:
                wp.twist.twist.linear.x = self.current_velocity - idx*speed_delta
            else:
                wp.twist.twist.linear.x = 0


    def pose_cb(self, msg):
        # Implement
        self.curr_pose = msg
        self._log('Got pose {}'.format(msg))

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

        curr_i = self.get_base_idx(self.curr_pose, self.prev_base_wp)
        self.prev_base_wp = curr_i

        self.curr_pose_wp = curr_i
        # lookahead_dist = max(1.5* self.get_waypoint_velocity(self.base_wp_list[curr_i]), 15)
        # off_i = self.get_base_off_idx(curr_i, lookahead_dist) # 60m gets more points. TBD Spline
        final_wp = self.get_rough_path(curr_i)
        self.final_waypoints_pub.publish(final_wp)

    def waypoints_cb(self, waypoints):
        # Implement
        self.base_wp_list = waypoints.waypoints
        self.total_base_wp = len(self.base_wp_list)
        self._log('Got waypoints_cb of size {}'.format(self.total_base_wp))
        # self.traffic_wp_list = deepcopy(waypoints.waypoints)

    def traffic_cb(self, msg):
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

        self._log('red_light_wp {} detect_red_wp {} light {}'.format(self.red_light_wp, self.detect_red_wp, msg.data))


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def curr_vel_cb(self, msg):
        self.current_velocity = msg.twist.linear.x


    # def get_waypoint_velocity(self, waypoint):
    #     return waypoint.twist.twist.linear.x
    #
    # def set_waypoint_velocity(self, waypoints, waypoint, velocity):
    #     waypoints[waypoint].twist.twist.linear.x = velocity
    #
    # def distance(self, wp1, wp2):
    #     dist = 0
    #     dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    #     if wp2 < wp1:
    #         r = range(wp1, len(self.base_wp_list))
    #         r.extend(range(0,wp2+1))
    #     else:
    #         r = range(wp1, wp2+1)
    #     for i in r:
    #         dist += dl(self.base_wp_list[wp1].pose.pose.position, self.base_wp_list[i].pose.pose.position)
    #         wp1 = i
    #     return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
