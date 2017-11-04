#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from os.path import exists  
from os import makedirs, remove
from glob import glob
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def _log(self, msg):
        if self.logEnable:
            rospy.logwarn(msg)

    def _getNextRecordName(self):
        name = self.imgDir+"/"+"img_"+"%05d"%(self.saveRecCount)+".jpg"
        self.saveRecCount += 1
        return name

    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.logEnable = False
        self.useTrafficLightsDebugEnable = True
        self.saveImgEnable = False
        self.saveImgCount = self.saveRecCount = 0
        self.saveImgRate = 10 # images are sent 10 times a second. rate=10 saves 1 per second.    
        self.imgDir = './img'
        if self.saveImgEnable:
            if not exists(self.imgDir):
                makedirs(self.imgDir)
            else:
                for name in glob(self.imgDir+"/*"):
                   remove(name)
        
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.last_car_position = 0
        self.stop_zone = 50.
        self._log('Stop Line Positions: {}'.format(self.config['stop_line_positions']))
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

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
        for a_idx in range(len(self.waypoints)):
            last_idx = idx
            idx = (a_idx + start_idx) % len(self.waypoints) 
            dist = dl(pose.pose.position, self.waypoints[idx].pose.pose.position)
            dist_sum += dl(self.waypoints[last_idx].pose.pose.position, self.waypoints[idx].pose.pose.position)
            if mindl > dist:
                minidx = idx
                mindl = dist
                if mindl<2.:
                    break
            if max_dist>0 and dist_sum>max_dist:
                return -1
        minidx = (minidx + 1) % len(self.waypoints)
        return minidx

    def get_base_off_idx(self, curr_i, dist):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        d = 0
        i = (curr_i + 1) % len(self.waypoints)
        last_i = curr_i
        while d < dist:
            d += dl(self.waypoints[last_i].pose.pose.position, self.waypoints[i].pose.pose.position)
            last_i = i
            i = (i + 1) % len(self.waypoints)
        return i   

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        if wp2 < wp1:
            r = range(wp1, len(self.waypoints))
            r.extend(range(0,wp2+1))
        else:
            r = range(wp1, wp2+1)
        for i in r:
            dist += dl(self.waypoints[wp1].pose.pose.position, self.waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_tl_stop(self, curr_i, stop_line_positions):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        
        step = 20
        stop_pose = PoseStamped()
        for i in range(len(stop_line_positions)):
            stop_pose.pose.position.x = stop_line_positions[i][0]
            stop_pose.pose.position.y = stop_line_positions[i][1]
            stop_pose.pose.position.z = 0
            d = dl(self.waypoints[curr_i].pose.pose.position, stop_pose.pose.position )
            self._log('Stop Light {} d {} curr_wp {} x {} y {}'.format(i, d, curr_i, stop_pose.pose.position.x, stop_pose.pose.position.y))
            if d < self.stop_zone:
                #curr_i = (curr_i -3 + len(self.waypoints)) % len(self.waypoints)
                stop_i = self.get_closest_waypoint(stop_pose, start_idx=curr_i, max_dist=self.stop_zone)
                if stop_i >= 0:
                    self._log('In Zone Stop Light {} wp {} x {} y {}'.format(i, stop_i, stop_pose.pose.position.x, stop_pose.pose.position.y))
                    return stop_i, True
        return -1, False

    def get_lookup_traffic_lights(self, light_wp):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        lights = self.lights
        stop_pose = Pose()
        mindl = 10000
        minidx = -1
        for i in range(len(lights)):
            stop_pose.position.x = lights[i].pose.pose.position.x
            stop_pose.position.y = lights[i].pose.pose.position.y
            stop_pose.position.z = 0
            d = dl(self.waypoints[light_wp].pose.pose.position, stop_pose.position)
            if mindl > d:
                mindl = d
                minidx = i
        if minidx < 0:
            return TrafficLight.UNKNOWN
        else:
            return lights[minidx].state

    def get_light_state(self, light_wp):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        if self.saveImgEnable:
            if self.saveImgCount % self.saveImgRate == 0:
                self._log('img size {}'.format(cv_image.shape))    
                cv2.imwrite(self._getNextRecordName(), cv_image)
            self.saveImgCount += 1
        if self.useTrafficLightsDebugEnable:
            return self.get_lookup_traffic_lights(light_wp)
        else:
            #Get classification
            return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose and self.waypoints):
            car_position = self.get_closest_waypoint(self.pose, start_idx=self.last_car_position)
            self.last_car_position = car_position
            self._log('Car pos {} x {} y {}'.format(car_position, self.waypoints[car_position].pose.pose.position.x, self.waypoints[car_position].pose.pose.position.y))
            #TODO find the closest visible traffic light (if one exists)
            # light = True
            light_wp, light = self.get_closest_tl_stop(car_position, stop_line_positions)

        if light:
            state = self.get_light_state(light_wp)
            if state == TrafficLight.YELLOW:
                state = TrafficLight.RED
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
