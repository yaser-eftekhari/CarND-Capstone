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
from os.path import exists, isdir  
from os import makedirs, remove, rmdir
from glob import glob
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def _log(self, msg):
        if self.logEnable:
            rospy.logwarn(msg)

    def _getNextRecordName(self, classtype='',prefix='img_'):
        name = self.imgDir+"/"+classtype+"/"+prefix+"%05d"%(self.saveRecCount)+".jpg"
        self.saveRecCount += 1
        return name

    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier((len(self.stop_line_positions)==1))
        self.listener = tf.TransformListener()

        self.logEnable = False
        self.useTrafficLightsDebugEnable = False
        self.saveImgEnable = False
        self.saveImgCount = self.saveRecCount = 0
        self.saveImgRate = 10 # images are sent 10 times a second. rate=10 saves 1 per second.    
        self.imgDir = './img'
        self.classlist = ['red','yellow', 'green', 'error', 'unknown']
        if self.saveImgEnable:
            if not exists(self.imgDir):
                makedirs(self.imgDir)
            else:
                for d in glob(self.imgDir+"/*"):
                    if isdir(d):
                        for name in glob(d+"/*"):
                           remove(name)
                        rmdir(d)
                    else:
                        remove(d)
            for d in self.classlist:
                makedirs(self.imgDir+"/"+d)        
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.last_car_position = 0
        if not self.useTrafficLightsDebugEnable:
            self.stop_zone = 60.
        else:
            self.stop_zone = 50.
        self._log('Stop Line Positions: {}'.format(self.config['stop_line_positions']))

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        """Callback function for when a car position is received

        Args:
            msg (current_pose): position of the car

        """
        self.pose = msg

    def waypoints_cb(self, waypoints):
        """Callback function for when all waypoints are received at the beginning of time.
        This function also gets the waypoints associated to the traffic lights

        Args:
            waypoints (base_waypoints): array of all waypoints on the track

        """
        self.waypoints = waypoints.waypoints
        self.stop_line_wp = []
        stop_pose = PoseStamped()
        for i in range(len(self.stop_line_positions)):
            stop_pose.pose.position.x = self.stop_line_positions[i][0]
            stop_pose.pose.position.y = self.stop_line_positions[i][1]
            stop_pose.pose.position.z = 0
            self.stop_line_wp.append(self.get_closest_waypoint(stop_pose))
        self._log('Stop line wps: {}'.format(self.stop_line_wp))

    def traffic_cb(self, msg):
        """Callback function for traffic lights. This information is only used in the debug state

        Args:
            msg (current_pose): position of the car

        """
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
            self._log('No Publish state_count {}'.format(self.state_count))
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            self._log('Publish light_wp {} state_count {}'.format(light_wp, self.state_count))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self._log('Publish light_wp {} state_count {}'.format(self.last_wp, self.state_count))
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

    def check_wp_ahead(self, wp, from_i):
        """Determine if wp is ahead of from_i in waypoints scale
        Args:
            wp : First waypoint
            from_i : Second waypoint

        Returns:
            boolean: true if wp is ahead of from_i

        """
        diff = wp - from_i
        if (diff > 0 and diff < (len(self.waypoints)/2)) or (diff < 0 and abs(diff) > (len(self.waypoints)/2)):
            return True # wp is ahead of from_i
        return False

    def get_closest_tl_stop(self, curr_i):
        """Finds the closest traffic light to the car and trigger the classification process only if the traffic light is close enough to the car
        Args:
            curr_i : location of the car

        Returns:
            int: waypoint index of the closest traffic light
            flag: true if traffic light is close enough

        """
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        
        stop_pose = PoseStamped()
        for i in range(len(self.stop_line_positions)):
            stop_pose.pose.position.x = self.stop_line_positions[i][0]
            stop_pose.pose.position.y = self.stop_line_positions[i][1]
            stop_pose.pose.position.z = 0
            d = dl(self.waypoints[curr_i].pose.pose.position, stop_pose.pose.position )
            self._log('Stop Light {} d {} curr_wp {} x {} y {}'.format(i, d, curr_i, stop_pose.pose.position.x, stop_pose.pose.position.y))
            if d < self.stop_zone:
                #curr_i = (curr_i -3 + len(self.waypoints)) % len(self.waypoints)
                wp = self.stop_line_wp[i]
                tl_crossed = self.check_wp_ahead(curr_i, wp)
                if tl_crossed and d < 5.:
                    stop_i = wp
                elif not tl_crossed:
                    stop_i = wp
                else:
                    stop_i = -1
                if stop_i >= 0:
                    self._log('In Zone Stop Light {} wp {} x {} y {}'.format(i, stop_i, stop_pose.pose.position.x, stop_pose.pose.position.y))
                    return stop_i, True
        return -1, False

    def get_lookup_traffic_lights(self, light_wp):
        """Finds the status of the traffic light ahead. Only used in debug mode
        Args:
            light_wp : waypoint of the traffic light ahead

        Returns:
            uint8: status of the light

        """
        if light_wp == -1:
            return TrafficLight.UNKNOWN
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

        
        pil_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        if self.useTrafficLightsDebugEnable:
            state = self.get_lookup_traffic_lights(light_wp)
            if self.saveImgEnable:
                if self.saveImgCount % self.saveImgRate == 0:
                    cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                    self._log('img size {}'.format(cv_image.shape))    
                    cv2.imwrite(self._getNextRecordName(self.classlist[state]), cv_image)
                self.saveImgCount += 1
            return state
        else:
            #Get classification
            correct_state = self.get_lookup_traffic_lights(light_wp)
            classifier_state = self.light_classifier.get_classification(pil_image)
            if correct_state != classifier_state and self.saveImgEnable:
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                prefix = 'err'+str(correct_state)+'_'+str(classifier_state)+'_'
                filename = self._getNextRecordName(self.classlist[3], prefix=prefix)
                self._log('Incorrect classification!! Correct {} filename {}'.format(correct_state, filename))
                cv2.imwrite(filename, cv_image)
            return classifier_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        if(self.pose and self.waypoints):
            car_position = self.get_closest_waypoint(self.pose, start_idx=self.last_car_position)
            self.last_car_position = car_position
            self._log('Car pos {} x {} y {}'.format(car_position, self.waypoints[car_position].pose.pose.position.x, self.waypoints[car_position].pose.pose.position.y))
            #TODO find the closest visible traffic light (if one exists)
            # light = True
            light_wp, light = self.get_closest_tl_stop(car_position)

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
