#!/usr/bin/env python
import numpy as np
import rospy
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from label_extraction.msg import Labels

import message_filters

from dynamic_reconfigure.server import Server
from label_extraction.cfg import VisualizationConfig

class LabelVisualizer:
    # some helper members
    bridge = CvBridge()
    video_fname = None
    video_writer = None

    image_source = 0


    def callback(self, labels_msg, ambient_img, intensity_img, range_img):
        rospy.loginfo_throttle(1.0, "Getting messages")

        if self.video_writer is None and self.video_fname is not None:
            self.video_writer = cv2.VideoWriter(self.video_fname, cv2.VideoWriter_fourcc(*"MJPG"), 10.0, (self.imw,self.imh))

        # Prepare the visualization window
        cvim = np.zeros((ambient_img.height, ambient_img.width, 3), dtype=np.uint8)
        if self.image_source == 0:
            cvim[:, :, 0] = self.bridge.imgmsg_to_cv2(ambient_img)
            cvim[:, :, 1] = self.bridge.imgmsg_to_cv2(ambient_img)
            cvim[:, :, 2] = self.bridge.imgmsg_to_cv2(ambient_img)
        elif self.image_source == 1:
            cvim[:, :, 0] = self.bridge.imgmsg_to_cv2(intensity_img)
            cvim[:, :, 1] = self.bridge.imgmsg_to_cv2(intensity_img)
            cvim[:, :, 2] = self.bridge.imgmsg_to_cv2(intensity_img)
        elif self.image_source == 2:
            cvim[:, :, 0] = self.bridge.imgmsg_to_cv2(range_img)
            cvim[:, :, 1] = self.bridge.imgmsg_to_cv2(range_img)
            cvim[:, :, 2] = self.bridge.imgmsg_to_cv2(range_img)
        else:
            cvim[:, :, 0] = self.bridge.imgmsg_to_cv2(ambient_img)
            cvim[:, :, 1] = self.bridge.imgmsg_to_cv2(intensity_img)
            cvim[:, :, 2] = self.bridge.imgmsg_to_cv2(range_img)

        # Process the labels
        for label in labels_msg.labels:
            color = (255,0,0)
            if label.masked_out:
                color = (0,0,255)
            tl = (label.bounding_box.x_offset, label.bounding_box.y_offset)
            br = (tl[0] + label.bounding_box.width, tl[1] + label.bounding_box.height)
            cv2.rectangle(cvim, (tl[0], tl[1]), (br[0], br[1]), color)

        ros_img = self.bridge.cv2_to_imgmsg(cvim)
        ros_img.header = ambient_img.header
        self.pub.publish(ros_img)

        cv2.imshow("labels_vis", cvim)
        cv2.waitKey(1)

        # Optionally also write the image to the video
        if self.video_writer is not None:
            self.video_writer.write(cvim)


    def dynrec_cbk(self, config, level):
        self.image_source = config.image_source
        return config


    def __init__(self):

        # Create publishers
        self.pub = rospy.Publisher("labels_vis", Image, queue_size=2)

        sub_lbl = message_filters.Subscriber("labels_in", Labels)
        sub_ambient = message_filters.Subscriber("os_img_nodelet/ambient_image", Image)
        sub_intensity = message_filters.Subscriber("os_img_nodelet/intensity_image", Image)
        sub_range = message_filters.Subscriber("os_img_nodelet/range_image", Image)

        ts = message_filters.TimeSynchronizer([sub_lbl, sub_ambient, sub_intensity, sub_range], 10)
        ts.registerCallback(self.callback)

        srv = Server(VisualizationConfig, self.dynrec_cbk)

        cv2.namedWindow("labels_vis", cv2.WINDOW_GUI_EXPANDED)
        rospy.loginfo_throttle(1.0, "Initialzied visualier, waiting for messages")


    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("label_visualizer")
    lv = LabelVisualizer()
    lv.spin()
