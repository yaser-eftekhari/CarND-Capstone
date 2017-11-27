import rospy
import math
import tensorflow as tf
import numpy as np
import cv2
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def _log(self, msg, force=False):
        if self.logEnable or force:
            rospy.logwarn(msg)

    def __init__(self, site=False):
        self.logEnable = False

        self.siteFlag = site
        if self.siteFlag:
            self._log('Selecting Model for classification at Site', force=True)
            PATH_TO_MODEL = 'light_classification/ssd-site.pb'
        else:
            self._log('Selecting Model for classification in simulator', force=True)
            PATH_TO_MODEL = 'light_classification/ssd-sim.pb'
        # PATH_TO_MODEL = 'light_classification/rfcn.pb' #slow
        # PATH_TO_MODEL = 'light_classification/faster_rcnn.pb' #too slow
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            # Works up to here.
            with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.det_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.det_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.det_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_det = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(graph=self.detection_graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Bounding Box Detection.
        with self.detection_graph.as_default():
            #image_np = self.load_image_into_numpy_array(image)
            # Expand dimension since the model expects image to have shape [1, None, None, 3].
            img_expanded = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num) = self.sess.run(
                [self.det_boxes, self.det_scores, self.det_classes, self.num_det],
                feed_dict={self.image_tensor: img_expanded})

        s_scores = np.squeeze(scores)
        s_classes = np.squeeze(classes).astype(np.int32)
        s_boxes = np.squeeze(boxes)

        self._log('TLClassifier {} {}'.format(s_scores, s_classes))
        best_class = -1
        max_score = -1.
        max_idx = -1
        left_most_x = 1.
        left_idx = -1
        for idx in range(len(s_scores)):
            if s_scores[idx] > 0.1 and s_scores[idx] > max_score:
                max_score = s_scores[idx]
                max_idx = idx
            # rely on the left most match to avoid confusion from T junctions
            if s_scores[idx] > 0.1 and s_boxes[idx][1] < left_most_x:
                left_most_x = s_boxes[idx][1]
                left_idx = idx

        if self.siteFlag and max_score > .2:
            best_class = s_classes[max_idx]
        elif not self.siteFlag and max_score > .8:
            best_class = s_classes[max_idx]
        elif not self.siteFlag and left_idx != -1:
            best_class = s_classes[left_idx]
        
        if best_class == 1:
            self._log('___GREEN___')
            return TrafficLight.GREEN
        if best_class == 2:
            self._log('___RED___')
            return TrafficLight.RED
        if best_class == 3:
            self._log('___YELLOW___')
            return TrafficLight.YELLOW

        self._log('___UNKNOWN___')

        return TrafficLight.UNKNOWN

