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

    def __init__(self):
        self.logEnable = False

        PATH_TO_MODEL = 'light_classification/ssd.pb'
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
            # Expand dimension since the model expects image to have shape [1, None, None, 3].
            img_expanded = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num) = self.sess.run(
                [self.det_boxes, self.det_scores, self.det_classes, self.num_det],
                feed_dict={self.image_tensor: img_expanded})

        s_scores = np.squeeze(scores)
        s_classes = np.squeeze(classes)
        s_boxes = np.squeeze(boxes)

        useful_idx = np.where((s_scores > 0.5) & (s_classes == 10))
        if len(useful_idx) == 0:
            return TrafficLight.UNKNOWN

        useful_box = s_boxes[useful_idx]

        h, w, _ = image.shape

        # rospy.logwarn('w {}, h {}'.format(w, h))

        sums = np.zeros(3, np.uint8)
        mask = np.zeros([h, w], np.uint8)
        for box in useful_box:
            y_lt = int(math.floor(h * box[0]))
            x_lt = int(math.floor(w * box[1]))
            y_rb = int(math.floor(h * box[2]))
            x_rb = int(math.floor(w * box[3]))
            mask[y_lt:y_rb, x_lt:x_rb] = 255

        color = ('b', 'g', 'r')
        for i, col in enumerate(color):
            hist_mask = cv2.calcHist([image], [i], mask, [256], [0, 256])
            sums[i] = sum(hist_mask[200:256])

        # rospy.logwarn('sums {}'.format(sums))

        # This is the ratio of green over red
        ratio = sums[1] * 1.0 / max(sums[2], 1.0)

        if ratio > 10:
            self._log('___GREEN___ {}'.format(ratio))
            return TrafficLight.GREEN

        if ratio < 0.3:
            self._log('___RED___ {}'.format(ratio))
            return TrafficLight.RED

        if 0.6 < ratio < 1.5:
            self._log('___YELLOW___ {}'.format(ratio))
            return TrafficLight.YELLOW

        self._log('___UNKNOWN___ {}'.format(ratio))

        return TrafficLight.UNKNOWN
