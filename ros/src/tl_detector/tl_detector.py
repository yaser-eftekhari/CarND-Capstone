#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import yaml
import math

class TLDetector(object):
    def _log(self, msg, force=False):
        if self.logEnable or force:
            rospy.logwarn(msg)

    def __init__(self):
        rospy.init_node('tl_detector')

        self.waypoints = None
        self.camera_image = None
        self.cv_image = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', TrafficLight, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = None

        self.logEnable = False

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_car_position = 0
        self.stop_zone_wp = 40

        self.traffic_light_wps = None
        self.waypoints_cached = False

        self.total_waypoints = None

        self.log_flag = True

        self.new_pose = False
        self.new_image = False

        self.loop()

    def loop(self):
        rate = rospy.Rate(7) # 7Hz
        while not rospy.is_shutdown():
            if self.new_pose and self.new_image:
                self.new_pose = False
                self.new_image = False
                self.process_traffic_lights()

            rate.sleep()

    def pose_cb(self, msg):
        """Callback function for when a car position is received

        Args:
            msg (current_pose): position of the car

        """
        pose = msg
        if self.waypoints_cached:
            self.last_car_position = self.get_closest_waypoint(pose, start_idx=self.last_car_position)
            self._log('Car pos {}'.format(self.last_car_position))

            self.new_pose = True

    def waypoints_cb(self, waypoints):
        """Callback function for when all waypoints are received at the beginning of time.
        This function also starts the classifier on tensorflow. It also gets the waypoints associated to the traffic lights

        Args:
            waypoints (base_waypoints): array of all waypoints on the track

        """
        self.waypoints = waypoints.waypoints
        self.total_waypoints = len(self.waypoints)
        self.traffic_light_wps = self.get_traffic_light_waypoints(self.config['stop_line_positions'])
        self.light_classifier = TLClassifier()
        self.waypoints_cached = True
        self._log('{} Waypoints received'.format(len(self.waypoints)))

    def image_cb(self, msg):
        """Callback function for when an image is received

        Args:
            msg (Image): image from car-mounted camera

        """
        self.camera_image = msg
        self.new_image = True

    def get_traffic_light_waypoints(self, stop_line_positions):
        """Find the closest waypoint to each traffic light

        Args:
            stop_line_positions : coordinates of the lights provided by the simulator

        """
        waypoints = []
        stop_pose = PoseStamped()
        for i in range(len(stop_line_positions)):
            stop_pose.pose.position.x = stop_line_positions[i][0]
            stop_pose.pose.position.y = stop_line_positions[i][1]
            stop_pose.pose.position.z = 0
            waypoint_idx = self.get_closest_waypoint(stop_pose, force_search=True)
            # compensate for the +1 in the get_closes_waypoint method
            waypoint_idx = (waypoint_idx - 1 + self.total_waypoints) % self.total_waypoints
            waypoints.append(waypoint_idx)

        self._log('light waypoints {}'.format(waypoints))
        return waypoints

    def get_closest_waypoint(self, pose, start_idx=0, force_search=False):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            start_idx : index of the previously found waypoint. This is used to speed up the search
            force_search: to force the search over all waypoints

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        mindl = 10000000
        minidx = -1

        start_idx_search = (start_idx - 2 + self.total_waypoints) % self.total_waypoints
        for a_idx in range(self.total_waypoints):
            idx = (a_idx + start_idx_search) % self.total_waypoints
            dist = dl(pose.pose.position, self.waypoints[idx].pose.pose.position)
            if mindl > dist:
                minidx = idx
                mindl = dist
            else:
                if not force_search:
                    break

        minidx = (minidx + 1) % self.total_waypoints
        return minidx

    # def get_closest_tl_stop(self, curr_i, stop_line_positions):
    def get_closest_tl_stop(self, curr_i):
        """Identify the closest traffic light and its state based on the distance to the car

        Args:
            curr_i : current position of the car in waypoint scale

        """
        for i in range(len(self.traffic_light_wps)):
            d_wp = self.traffic_light_wps[i] - curr_i
            self._log('Stop Light {} d {} curr_wp {}'.format(i, d_wp, curr_i))
            if 0 <= d_wp < self.stop_zone_wp:
                if self.log_flag:
                    self._log('In Zone Stop Light {} light wp {} car wp {}'.format(i, self.traffic_light_wps[i], curr_i))
                    self.log_flag = False
                return self.traffic_light_wps[i], True
            self.log_flag = True
        return -1, False

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        """
        # List of positions that correspond to the line to stop in front of for a given intersection
        self.cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        # find the closest visible traffic light (if one exists)
        light_wp, light_is_close = self.get_closest_tl_stop(self.last_car_position)

        state = TrafficLight.UNKNOWN
        if light_is_close:
            state = self.light_classifier.get_classification(self.cv_image)
            self._log('state {}'.format(state))
            if state == TrafficLight.YELLOW:
                state = TrafficLight.RED

        '''
        Publish upcoming red lights at camera frequency.
        '''
        light_output = TrafficLight()
        light_output.state = state
        light_output.waypoint_idx = light_wp
        self.upcoming_red_light_pub.publish(light_output)


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
