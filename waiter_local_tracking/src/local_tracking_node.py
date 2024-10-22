#!/usr/bin/env python3
import cv2
import rospy
import tf2_ros
import time
import numpy as np

from cv_bridge import CvBridge

from geometry_msgs.msg import Vector3, TransformStamped
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Bool
from waiter_msgs.msg import PersonData, SvmData

from classify import SVM
from detection import *
from extraction import FtNetExtractor, FtNetExtractor_TRT

BATCH_SIZE = 16
detector = YoloDetector()
extractor_trt_16 = FtNetExtractor_TRT(USE_FP16=True, BATCH_SIZE=BATCH_SIZE)


class LocalTracking:
    def __init__(self):
        rospy.init_node('local_tracking_node')
        
        # Set up node parameters
        prefix = rospy.get_param('~prefix', "")
        self.update_interval = rospy.get_param('~update_interval', 2)  # 2 second

        # Set up node attributes
        self.classifier = SVM()
        self.bridge = CvBridge()
        self.broadcaster = tf2_ros.TransformBroadcaster()
        #self.listener = tf.TransformListener()
        self.mtx = []
        self.inv_mtx = []
        self.dist = []
        self.has_info = False
        self.last_update = time.time()
        self.predict_th = 0.60
        self.prev_frame_time = 0
        self.new_frame_time = 0
        self.depth_img = None

        # Set up publish topics
        self.visualizer_rw = rospy.Publisher(prefix+'/local_tracking/image_raw', Image, queue_size=1)
        self.visualizer_cp = rospy.Publisher(prefix+'/local_tracking/image_raw/compressed', CompressedImage, queue_size=1)
        self.person_pub = rospy.Publisher(prefix+'/local_tracking/person_data', PersonData, queue_size=10)
        self.center_pub = rospy.Publisher(prefix+'/target/center', Vector3, queue_size=10)

        # Set up subscribe topics
        rospy.Subscriber(prefix+"/local_cam/color/image_raw/compressed", CompressedImage, self.image_callback,queue_size=1,buff_size=2**24)
        rospy.Subscriber(prefix+"/local_cam/aligned_depth_to_color/image_raw/compressedDepth", CompressedImage, self.compressedDepth_cb,queue_size=1,buff_size=2**24)
        rospy.Subscriber(prefix+"/local_cam/color/camera_info", CameraInfo, self.build_camera_info)
        rospy.Subscriber(prefix+"/svm_data", SvmData, self.svm_callback)

    def compressedDepth_cb(self,msg):
        depth_header_size = 12
        raw_data = msg.data[depth_header_size:]

        cv_img = cv2.imdecode(np.frombuffer(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)
        img_scaled = cv2.normalize(cv_img, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX)
        self.depth_img = cv_img

    def build_camera_info(self, msg):
        if(not self.has_info):
            self.mtx = np.array(msg.K).reshape([3, 3])
            self.inv_mtx = np.linalg.inv(self.mtx)
            self.dist = np.array(msg.D)
            self.has_info = True

    def svm_callback(self, msg):
        rospy.loginfo("svm_callback!")
        sample = len(msg.y.data)
        x_data = np.array(msg.x.data)
        y_data = np.array(msg.y.data)
        if(sample==0):
            rospy.loginfo("svm reset")
            self.classifier = SVM()
           
        else:
            x_data = x_data.reshape(sample,512)
            self.classifier.train(x_data, y_data)
            rospy.loginfo("model trained!")

    def broadcast_target(self, center, distance):
        pixel_target = np.array([[distance*center[0]],
                                 [distance*center[1]],
                                 [distance]])

        target = np.matmul(self.inv_mtx, pixel_target)

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()  # please ensure you are using /use_sim_time
        msg.header.frame_id = "local_cam_link"
        msg.child_frame_id = "target"
        msg.transform.translation.x = target[0]
        msg.transform.translation.y = target[1]
        msg.transform.translation.z = target[2]
        msg.transform.rotation.x = 0
        msg.transform.rotation.y = 0
        msg.transform.rotation.z = 0
        msg.transform.rotation.w = 1
        self.broadcaster.sendTransform(msg)

    def image_callback(self, msg):
        self.new_frame_time = time.time()
        fps = 1/(self.new_frame_time-self.prev_frame_time)
        self.prev_frame_time = self.new_frame_time

        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        frame = cv_img.copy()
        
        persons = detector.detect_person(cv2.cvtColor(cv_img.copy(), cv2.COLOR_BGR2RGB))
        num = len(persons)
        target_center = Vector3(-1,-1,-1)

        if(num!=0):
            input_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cropped_list = cut_boxes(input_rgb, persons)
            features = extractor_trt_16.extract_batch(cropped_list)
            features = features[:num,:]

            max_prob = 0
            if(self.classifier.is_trained):
                    results = self.classifier.get_prediction(features)
                    # [[prob_0,prob_1],[prob_0,prob_1],[prob_0,prob_1]]
                    max_id = np.argmax(results[:,1],axis=0)
                    if(results[max_id][1] > self.predict_th):  # 1 is target
                        depth_cropped = cut_box(self.depth_img, persons[max_id])
                        distance = np.median(depth_cropped)/1000  # mm to m
                        box = persons[max_id]
                        center = (int((box[2]-box[0])/2+box[0]), int((box[3]-box[1])/2+box[1]))
                        target_center.x = center[0]
                        target_center.y = center[1]
                        self.broadcast_target(center, distance)
                        
                        draw_box(cv_img, persons[max_id], color=(255,0,0), width=int(10* results[max_id][1]))
                        del persons[max_id]
            
            draw_boxes(cv_img, persons)
            # Save person image and features every 2 second
            if((self.new_frame_time-self.last_update)>self.update_interval):
                for c in range(len(cropped_list)):
                    cropped_list[c] = cv2.cvtColor(cropped_list[c], cv2.COLOR_RGB2BGR)
                    person_data = PersonData()
                    person_data.header.stamp = rospy.Time.now()
                    person_data.image = self.bridge.cv2_to_compressed_imgmsg(cropped_list[c])
                    person_data.features.data = features[c]
                    self.person_pub.publish(person_data)
                    self.last_update = self.new_frame_time

        self.center_pub.publish(target_center)

        cv2.putText(cv_img, "fps: %.2f"%fps, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (0,255,0), 2, cv2.LINE_AA)

        self.visualizer_rw.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))
        self.visualizer_cp.publish(self.bridge.cv2_to_compressed_imgmsg(cv_img))


if __name__ == "__main__":
    node = LocalTracking()
    rospy.spin()

