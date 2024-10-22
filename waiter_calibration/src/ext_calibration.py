#!/usr/bin/env python3
"""
    This node is used for extrinsic calibration.
    Get the Forward kinemetics from Map to Baselink.
    Put the apriltag (id: 0) on Mir's back, so we know Baselink -> tag_0 and Map -> tag_0.
    Next, by apriltag api we also know Camera -> tag_0.
    Inverse it we will get tag_0 -> Camera.
    Finally, we can get Map -> tag_0 -> Camera and return Map -> Camera as it is extrinsic.  
"""

import rospy
import rospkg
import numpy as np
import tf
import tf2_ros
import yaml
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from tf.transformations import translation_matrix, quaternion_matrix, quaternion_from_matrix

class ExtCalibrator:
    def __init__(self):
        rospy.init_node('extrinsic_calibrator', anonymous=True)
        rospy.on_shutdown(self.save_calibration)
        self.prefix = rospy.get_param('~prefix', "")

        rospy.Subscriber(self.prefix+"/tag_detections", AprilTagDetectionArray, self.tag_callback)

        path = rospkg.RosPack().get_path('waiter_calibration') + "/config/extrinsic.yaml"
        self.save_file = open(path, "w")
        self.listener = tf.TransformListener()

        self.result = {}
        rospy.spin()

    def tag_callback(self, data):
        if data.detections != []:
            tag_pose = data.detections[0].pose.pose.pose
            cam_to_tag = self.pose_to_np(tag_pose)
            self.calibrate(cam_to_tag)

    def save_calibration(self):
        rospy.loginfo("Saving last data...")
        rospy.loginfo(self.result)
        yaml.dump(self.result, self.save_file, default_flow_style=False)
        self.save_file.close()
        rospy.loginfo("Calibration saved")

    def pose_to_np(self, pose):
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = np.array([pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w])

        trans = translation_matrix(pos)
        rot = quaternion_matrix(quat)
        return np.dot(trans,rot)

    def calibrate(self, cam_to_tag):
        tag_to_cam = np.linalg.inv(cam_to_tag)
        trans = tag_to_cam[:3,3]
        quat = quaternion_from_matrix(tag_to_cam)
        br = tf2_ros.TransformBroadcaster()

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "tag_0"
        msg.child_frame_id = "global_cam"
        msg.transform.translation.x = trans[0]
        msg.transform.translation.y = trans[1]
        msg.transform.translation.z = trans[2]
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]
        br.sendTransform(msg)
        
        try:
            trans,rot = self.listener.lookupTransform('/map','/global_cam', rospy.Time(0))
            self.result = dict(x = trans[0], y = trans[1], z = trans[2],
                        qx = rot[0], qy = rot[1], qz = rot[2], qw = rot[3],
                        frame_id = "map", child_frame_id="global_cam")
        except:
            return

if __name__ == '__main__':
    ExtCalibrator()




