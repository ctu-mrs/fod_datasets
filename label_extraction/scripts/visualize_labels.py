#!/usr/bin/env python
import numpy as np
import rospy
import struct
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
import cv2

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from lidar_tracker.msg import Tracks
from label_extraction.msg import Labels
from label_extraction.msg import Label

import message_filters


# just a helper class, all coordinates in pixels
class BoundingBox:
    # coordinates of the bounding box's center
    u = None
    v = None
    # dimensions of the bounding box's
    width = None
    height = None
    # top-left and bottom-right corners
    tl = None
    br = None
    # distance of the target from the camera
    range = None


class LabelVisualizer:
    # the main parameters of this script - the vertical and horizontal field of view of the sensor
    vfov = np.pi/2.0
    hfov = 2.0*np.pi
    hoffset = np.pi + 0.19 # empirically determined, seems to match well
    tgtr = 0.3 # radius of the target (metres)

    imw = None # image width, loaded from the first received image
    imh = None # image height, loaded from the first received image

    use_single_channel = True # used only for visualization
    ignore_secondary = True # if true, only the track that is marked as selected will be used, others will be ignored
    ignore_masked = True # if true, tracks that are masked out (hidden by the observer's body) will be ignored
    mask = None # used to mask out detections covered by body of the observer, loaded at initialization

    # some helper members
    tf_buffer = None
    bridge = CvBridge()
    video_fname = None
    video_writer = None


    def project3Dto2D(self, point3d):
        yaw = -np.arctan2(point3d.y, point3d.x) + self.hoffset
        # normalize yaw to be from 0 to 2*pi
        if yaw < 0.0:
            yaw = yaw + 2.0*np.pi
        dist_xy = np.sqrt(point3d.x**2.0 + point3d.y**2.0)
        pitch = -np.arctan2(point3d.z, dist_xy)
        u = int(np.round(yaw/self.hfov*self.imw))
        v = int(np.round((pitch/self.vfov + 0.5)*self.imh))
        return (u, v)


    def getBoundingBox(self, track, transformation):
        pst = PointStamped()
        pst.point = track.position
        tfd = tf2_geometry_msgs.do_transform_point(pst, transformation).point

        # Project the center of the detection to the image
        u, v = self.project3Dto2D(tfd)

        # Calculate the bounding rectangle
        # I was lazy to think, so I just brute-forced this :D
        pts3d = list()
        for x in (-1, 1):
            for y in (-1, 1):
                for z in (-1, 1):
                    pt3d = Point()
                    pt3d.x = tfd.x + x*self.tgtr
                    pt3d.y = tfd.y + y*self.tgtr
                    pt3d.z = tfd.z + z*self.tgtr
                    # print(pt3d)
                    pts3d.append(pt3d)
          
        pts = list()
        for pt3d in pts3d:
            pt = self.project3Dto2D(pt3d)
            pts.append(pt)
          
        tl_pt = [pts[0][0], pts[0][1]]
        br_pt = [pts[1][0], pts[1][1]]
        for pt in pts:
            if pt[0] < tl_pt[0]:
                tl_pt[0] = pt[0]
            if pt[1] < tl_pt[1]:
                tl_pt[1] = pt[1]
            if pt[0] > br_pt[0]:
                br_pt[0] = pt[0]
            if pt[1] > br_pt[1]:
                br_pt[1] = pt[1]
        width = br_pt[0] - tl_pt[0]
        height = br_pt[1] - tl_pt[1]

        ret = BoundingBox()
        ret.u = u
        ret.v = v
        ret.width = width
        ret.height = height
        ret.tl = tl_pt
        ret.br = br_pt
        ret.range = np.linalg.norm(np.array((tfd.x, tfd.y, tfd.z)))
        return ret

    def callback(self, labels_msg, ambient_img, intensity_img, range_img):
        rospy.loginfo_throttle(1.0, "Getting messages")

        if self.video_writer is None and self.video_fname is not None:
            self.video_writer = cv2.VideoWriter(self.video_fname, cv2.VideoWriter_fourcc(*"MJPG"), 10.0, (self.imw,self.imh))

        # Prepare the visualization window
        cvim = np.zeros((ambient_img.height, ambient_img.width, 3), dtype=np.uint8)
        if self.use_single_channel:
            cvim[:, :, 0] = self.bridge.imgmsg_to_cv2(ambient_img, desired_encoding='passthrough')
            cvim[:, :, 1] = self.bridge.imgmsg_to_cv2(ambient_img, desired_encoding='passthrough')
            cvim[:, :, 2] = self.bridge.imgmsg_to_cv2(ambient_img, desired_encoding='passthrough')
        else:
            cvim[:, :, 0] = self.bridge.imgmsg_to_cv2(ambient_img, desired_encoding='passthrough')
            cvim[:, :, 1] = self.bridge.imgmsg_to_cv2(intensity_img, desired_encoding='passthrough')
            cvim[:, :, 2] = self.bridge.imgmsg_to_cv2(range_img, desired_encoding='passthrough')

        # Process the labels
        for label in labels_msg.labels:
            if not track.selected and self.ignore_secondary:
                continue

            color = (255,0,0)
            if label.masked_out:
                color = (0,0,255)
            tl = (label.bounding_box.x_offset, label.bounding_box.y_offset)
            br = (tl[0] + label.bounding_box.width, tl[1] + label.bounding_box.height)
            cv2.rectangle(cvim, (tl[0], tl[1]), (br[0], br[1]), color)

        self.pub.publish(labels_msg)

        cv2.imshow("label_vis", cvim)
        cv2.waitKey(1)

        # Optionally also write the image to the video
        if self.video_writer is not None:
            self.video_writer.write(cvim)


    def __init__(self):

        # Create publishers
        self.pub = rospy.Publisher("label_vis", Image, queue_size=2)

        sub_lbl = message_filters.Subscriber("labels_in", Tracks)
        sub_ambient = message_filters.Subscriber("os_img_nodelet/ambient_image", Image)
        sub_intensity = message_filters.Subscriber("os_img_nodelet/intensity_image", Image)
        sub_range = message_filters.Subscriber("os_img_nodelet/range_image", Image)

        ts = message_filters.TimeSynchronizer([sub_lbl, sub_ambient, sub_intensity, sub_range], 10)
        ts.registerCallback(self.callback)
        cv2.namedWindow("tgt_vis", cv2.WINDOW_GUI_EXPANDED)
        rospy.loginfo_throttle(1.0, "Initialzied, waiting for messages")


    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("label_visualizer")
    lv = LabelVisualizer()
    lv.spin()
