#!/usr/bin/env python3
import cv2
import os
import rospy
import numpy as np

from cv_bridge import CvBridge

from std_srvs.srv import Trigger, TriggerResponse
from waiter_msgs.msg import PersonData, SvmData


class RobotMemory:
    def __init__(self):
        rospy.init_node('robot_memory_module', anonymous=True)
        rospy.loginfo("Node initialized")

        # Set up node parameters
        prefix = rospy.get_param('~prefix', "")
        save_global = rospy.get_param('~save_global', True)
        save_local = rospy.get_param('~save_local', True)
        rospy.loginfo(f"Save global camera : {save_global}")
        rospy.loginfo(f"Save local camera : {save_local}")

        # Set up node attributes
        root_path = os.path.dirname(os.path.realpath(__file__))
        self.capture_path = os.path.join(root_path, "image-capture")
        self.features_path = os.path.join(root_path, "image-features")
        self.data_path = os.path.join(root_path, "data","data.npy")
        self.bridge = CvBridge()
       
        # Set up publish topics
        self.svm_pub = rospy.Publisher(prefix+"/svm_data", SvmData, queue_size=10)

        # Set up service servers
        self.svm_service = rospy.Service(prefix+"/updateSVM", Trigger, self.send_svm)

        # Set up subscribe topics
        if(save_global):
            rospy.Subscriber(prefix+"/global_tracking/person_data", PersonData,
                             self.global_saver,queue_size=10)
        
        if(save_local):
            rospy.Subscriber(prefix+"/local_tracking/person_data", PersonData,
                            self.local_saver,queue_size=10)

    def send_svm(self, trig):
        rospy.loginfo("SVM service called!")
        svm_data = SvmData()
        rospy.sleep(1)
        try:
            svm_np = np.load(self.data_path, allow_pickle=True)
            svm_np = svm_np.item()
            svm_data.x.data = np.array(svm_np['x']).flatten().tolist()
            svm_data.y.data = svm_np['y']
        except:
            pass

        self.svm_pub.publish(svm_data)
        return [True, '']

    def global_saver(self, msg):
        header = msg.header.stamp
        cropped = self.bridge.compressed_imgmsg_to_cv2(msg.image, "bgr8")
        features = msg.features.data
        cv2.imwrite(self.capture_path+"/img_%s.png"%header , cropped)
        dict_save = {'name': header, 'data':features}
        np.save(self.features_path+"/fe_%s"%header, dict_save)

    def local_saver(self, msg):
        header = msg.header.stamp
        cropped = self.bridge.compressed_imgmsg_to_cv2(msg.image, "bgr8")
        features = msg.features.data
        cv2.imwrite(self.capture_path+"/img_%s.png"%header , cropped)
        dict_save = {'name': header, 'data':features}
        np.save(self.features_path+"/fe_%s"%header, dict_save)


if __name__ == "__main__":
    node = RobotMemory()
    rospy.spin()