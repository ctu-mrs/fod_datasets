#!/usr/bin/env python
import numpy as np
import rospy
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


class LabelExtractor:
    # the main parameters of this script - the vertical and horizontal field of view of the sensor
    vfov = np.pi/2.0
    hfov = 2.0*np.pi
    hoffset = np.pi + 0.19 # empirically determined, seems to match well
    tgtr = 0.3 # radius of the target (metres)

    imw = None # image width, loaded from the first received image
    imh = None # image height, loaded from the first received image

    ignore_secondary = False # if true, only the track that is marked as selected will be used, others will be ignored
    ignore_masked = False # if true, tracks that are masked out (hidden by the observer's body) will be ignored
    mask = None # used to mask out detections covered by body of the observer, loaded at initialization

    # some helper members
    tf_buffer = None
    bridge = CvBridge()


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

    def callback(self, tracks_msg, ambient_img, intensity_img, range_img):
        rospy.loginfo_throttle(1.0, "Getting messages")
        self.imw = ambient_img.width
        self.imh = ambient_img.height

        # find the transformation from the tracks' frame to the images' frame
        try:
            transformation = self.tf_buffer.lookup_transform(ambient_img.header.frame_id, tracks_msg.header.frame_id, tracks_msg.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Couldn't transform msg from frame {} to frame {} at {}".format(ambient_img.header.frame_id, tracks_msg.header.frame_id, tracks_msg.header.stamp))
            return

        # Process the tracks
        labels_msg = Labels()
        labels_msg.header = ambient_img.header
        track_processed = False
        for track in tracks_msg.tracks:
            if not track.selected and self.ignore_secondary:
                continue

            bb = self.getBoundingBox(track, transformation)
            if bb.u < 0 or bb.u >= self.imw or bb.v < 0 or bb.v >= self.imh:
                rospy.logwarn("Target is outside the bounds of the image: [{}, {}] not within [{}, {}]".format(bb.u, bb.v, self.imw, self.imh))
                continue

            masked_out = False
            if self.mask is not None and not self.mask[bb.v,bb.u].any():
                masked_out = True

            if masked_out and self.ignore_masked:
                continue

            label = Label()
            label.id = track.id
            label.range = bb.range
            label.bounding_box.x_offset = bb.tl[0]
            label.bounding_box.y_offset = bb.tl[1]
            label.bounding_box.width = bb.width
            label.bounding_box.height = bb.height
            label.masked_out = masked_out
            label.primary = track.selected
            labels_msg.labels.append(label)

            track_processed = True

        if not track_processed:
            rospy.logwarn("No valid track out of {} tracks!".format(len(tracks_msg.tracks)))

        try:
            self.pub.publish(labels_msg)
        except:
            rospy.logwarn("Could not publish labels!")


    def __init__(self):

        # Load parameters
        mask_fname = rospy.get_param("~mask_filename")

        # Load mask
        if len(mask_fname) > 0:
            self.mask = cv2.imread(mask_fname)
            if self.mask is None:
                rospy.logwarn("Could not load mask from file {}".format(mask_fname))

        # Create publishers
        self.pub = rospy.Publisher("labels", Labels, queue_size=2)

        # Create subscribers
        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)

        sub_tgt = message_filters.Subscriber("lidar_tracker/tracks", Tracks)
        sub_ambient = message_filters.Subscriber("os_img_nodelet/ambient_image", Image)
        sub_intensity = message_filters.Subscriber("os_img_nodelet/intensity_image", Image)
        sub_range = message_filters.Subscriber("os_img_nodelet/range_image", Image)

        ts = message_filters.TimeSynchronizer([sub_tgt, sub_ambient, sub_intensity, sub_range], 10)
        ts.registerCallback(self.callback)
        rospy.loginfo_throttle(1.0, "Initialzied extractor, waiting for messages")


    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("label_extractor")
    le = LabelExtractor()
    le.spin()
