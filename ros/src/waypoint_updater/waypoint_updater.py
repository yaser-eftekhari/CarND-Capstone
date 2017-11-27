#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
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

LOOKAHEAD_WPS = 20 # Number of waypoints we will publish. You can change this number


def clone_waypoint(wp, vel):
    output = Waypoint()

    output.pose.pose.position.x = wp.pose.pose.position.x
    output.pose.pose.position.y = wp.pose.pose.position.y
    output.pose.pose.position.z = wp.pose.pose.position.z

    output.twist.twist.linear.x = vel
    output.twist.twist.linear.y = wp.twist.twist.linear.y
    output.twist.twist.linear.z = wp.twist.twist.linear.z

    return output


class WaypointUpdater(object):
    def _log(self, msg, force_enable = False):
        if force_enable or self.logEnable:
            rospy.logwarn(msg)

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', TrafficLight , self.traffic_cb)
        # rospy.Subscriber('/traffic_waypoint', Int32 , self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        # Adding Current velocity as well to be used to calculate the speed for final waypoints
        rospy.Subscriber('/current_velocity', TwistStamped, self.curr_vel_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.logEnable = False
        self.base_wp_list = None
        self.curr_pose = None
        self.curr_pose_wp = -1
        self.red_light_wp = -1
        self.red_light_state = TrafficLight.UNKNOWN

        self.total_base_wp = None
        self.prev_base_wp = None
        self.curr_speed = None

        self.new_pose = False
        self.new_traffic = False

        self.stop_zone_wp = 3

        self.loop()

    def loop(self):
        rate = rospy.Rate(10) # 20Hz
        while not rospy.is_shutdown():
            if self.new_pose or self.new_traffic:
                self.new_pose = False
                self.new_traffic = False

                final_wp = self.get_rough_path(self.curr_pose_wp)
                self.final_waypoints_pub.publish(final_wp)

            rate.sleep()

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
            else:
                break

        minidx = (minidx + 1) % self.total_base_wp

        return minidx

    def get_rough_path(self, curr_i):
        path = Lane()
        off_i = (curr_i + LOOKAHEAD_WPS)

        self._log('current index {} end index {}'.format(curr_i, off_i))

        final_wps = self.adjust_speed(self.base_wp_list[curr_i:off_i])

        path.waypoints = final_wps

        return path

    def adjust_speed(self, original_wp):
        if self.red_light_wp == -1:
            return original_wp

        distance_wp = self.red_light_wp - self.curr_pose_wp

        # We have passed the light
        if distance_wp <= 0:
            return original_wp

        # Traffic light is too far from now
        if distance_wp >= LOOKAHEAD_WPS:
            return original_wp

        # Traffic light is fast approaching
        # Light is green
        if self.red_light_state == TrafficLight.GREEN:
            return original_wp

        # self._log('light {} distance {}'.format(self.red_light_state, distance_wp), True)

        if distance_wp <= self.stop_zone_wp:
            target_velocity = 0
        # elif distance_wp <= 3*self.stop_zone_wp:
        #     target_velocity = 2
        # else:
        #     target_velocity = original_wp[0].twist.twist.linear.x / 4.
        else:
            target_velocity = 2

        output = []
        for idx in range(LOOKAHEAD_WPS):
            wp = clone_waypoint(original_wp[idx], target_velocity)
            output.append(wp)

        return output

        # for idx in range(LOOKAHEAD_WPS):
        #     wp = original_wp[idx]
        #     wp.twist.twist.linear.x = target_velocity

            # if distance_wp - idx <= self.stop_zone_wp:
            #     wp.twist.twist.linear.x = 0
            # elif distance_wp - idx <= 3*self.stop_zone_wp:
            #     wp.twist.twist.linear.x = 2
            # else:
            #     wp.twist.twist.linear.x /= 4.

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

        if self.prev_base_wp is None:
            curr_i = self.get_base_idx(self.curr_pose, 0)
        else:
            curr_i = self.get_base_idx(self.curr_pose, self.prev_base_wp)

        self.prev_base_wp = (curr_i - 2 + self.total_base_wp) % self.total_base_wp

        self.curr_pose_wp = curr_i

        self.new_pose = True

    def waypoints_cb(self, waypoints):
        # Implement
        self.base_wp_list = waypoints.waypoints
        self.total_base_wp = len(self.base_wp_list)
        self._log('Got waypoints_cb of size {}'.format(self.total_base_wp))

    def traffic_cb(self, msg):
        self.red_light_state = int(msg.state)
        self.red_light_wp = int(msg.waypoint_idx)
        self.new_traffic = True

    def obstacle_cb(self, msg):
        # Callback for /obstacle_waypoint message. We will implement it later
        pass

    def curr_vel_cb(self, msg):
        self.curr_speed = msg.twist.linear.x


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
