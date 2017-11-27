import rospy
import tensorflow as tf
import numpy as np
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def _log(self, msg, force=False):
        if self.logEnable or force:
            rospy.logwarn(msg)

    def __init__(self):
        self.logEnable = False

        PATH_TO_MODEL = 'light_classification/ssd-sim.pb'
        # PATH_TO_MODEL = 'light_classification/ssd-site.pb'
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
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

        useful_idx = np.where(s_scores > 0.5)

        if len(useful_idx) == 0:
            return TrafficLight.UNKNOWN

        for idx in range(len(useful_idx)):
            self._log('idx {}, class {}, score {}'.format(idx, s_classes[idx], s_scores[idx]))
            if s_classes[idx] == 1:
                self._log('___GREEN___')
                return TrafficLight.GREEN

            if s_classes[idx] == 2:
                self._log('___RED___')
                return TrafficLight.RED

            if s_classes[idx] == 3:
                self._log('___YELLOW___')
                return TrafficLight.YELLOW

        self._log('___UNKNOWN___')
        return TrafficLight.UNKNOWN
