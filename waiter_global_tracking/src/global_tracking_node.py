#!/usr/bin/env python3
import cv2
import rospy
import time
import numpy as np

from cv_bridge import CvBridge
from tf.transformations import translation_matrix, quaternion_matrix

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from waiter_msgs.msg import PersonData, SvmData

from classify import SVM
from detection import *
from extraction import FtNetExtractor,FtNetExtractor_TRT


BATCH_SIZE = 16
detector = YoloDetector()
extractor_trt = FtNetExtractor_TRT(USE_FP16=True, BATCH_SIZE=BATCH_SIZE)


def get_ext_matrix(extrinsic):
    pos = np.array([extrinsic['x'], extrinsic['y'], extrinsic['z']])
    quat = np.array([extrinsic['qx'], extrinsic['qy'],
                extrinsic['qz'], extrinsic['qw']])

    trans = translation_matrix(pos)
    rot = quaternion_matrix(quat)
    return np.dot(trans,rot)


class GlobalTracking:
    def __init__(self):
        rospy.init_node('global_tracking_node')
        
        # Set up node parameters
        prefix = rospy.get_param('~prefix', "")
        self.update_interval = rospy.get_param('~update_interval', 2)  # 2 second
        self.map_to_cam = get_ext_matrix(rospy.get_param(prefix+'/extrinsic'))

        # Set up node attributes
        self.classifier = SVM()
        self.bridge = CvBridge()
        self.mtx = []
        self.dist = []
        self.has_info = False
        self.last_update = time.time()
        self.predict_th = 0.60
        self.prev_frame_time = 0
        self.new_frame_time = 0
        
        # Set up publish topics
        self.visualizer_rw = rospy.Publisher(prefix+'/global_tracking/image_raw', Image, queue_size=1)
        self.visualizer_cp = rospy.Publisher(prefix+'/global_tracking/image_raw/compressed', CompressedImage, queue_size=1)
        self.person_pub = rospy.Publisher(prefix+'/global_tracking/person_data', PersonData, queue_size=10)
        self.pos_pub = rospy.Publisher(prefix+'/target/global_position', Vector3, queue_size=10)

        # Set up subscribe topics
        rospy.Subscriber(prefix+"/global_cam/image_raw/compressed", CompressedImage, self.image_callback,queue_size=1,buff_size=2**24)
        rospy.Subscriber(prefix+"/global_cam/camera_info", CameraInfo, self.build_camera_info)
        rospy.Subscriber(prefix+"/svm_data", SvmData, self.svm_callback)

    def build_camera_info(self, msg):
        if(not self.has_info):
            self.mtx = np.array(msg.K).reshape([3, 3])
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

    def get_person_position(self, box):
        objpt = np.array([[0,0,0],
                  [0.70,0,0],
                  [0,1.7,0],
                  [0.70,1.7,0]], np.float32)

        imgpt = np.array([[box[0],box[1]],
                             [box[2],box[1]],
                              [box[0],box[3]],
                             [box[2],box[3]]], np.float32)

        ret, rvec, tvec = cv2.solvePnP(objpt, imgpt, self.mtx, self.dist)
        rot, jac = cv2.Rodrigues(rvec)
        iden = np.identity(3)
        cam_to_person = np.concatenate((rot, tvec), axis=1)
        cam_to_person = np.concatenate((cam_to_person, np.array([[0,0,0,1]])), axis=0)
        
        map_to_person = np.matmul(self.map_to_cam, cam_to_person)
        return map_to_person

    def image_callback(self, msg):
        self.new_frame_time = time.time()
        fps = 1/(self.new_frame_time-self.prev_frame_time)
        self.prev_frame_time = self.new_frame_time

        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        frame = cv_img.copy()

        persons = detector.detect_person(cv2.cvtColor(cv_img.copy(), cv2.COLOR_BGR2RGB))
        num = len(persons)
        target_pos = Vector3(-1,-1,-1)

        
        if(num!=0):
            input_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cropped_list = cut_boxes(input_rgb, persons)
            features = extractor_trt.extract_batch(cropped_list)
            features = features[:num,:]

            max_prob = 0
            if(self.classifier.is_trained):
                    results = self.classifier.get_prediction(features)
                    # [[prob_0,prob_1],[prob_0,prob_1],[prob_0,prob_1]]
                    max_id = np.argmax(results[:,1],axis=0)
                    if(results[max_id][1] > self.predict_th):  # 1 is target
                        box = persons[max_id]
                        map_to_person = self.get_person_position(box)
                        pos2d = np.around(map_to_person[:2,3],2)
                        target_pos.x = pos2d[0]
                        target_pos.y = pos2d[1]

                        draw_box(cv_img, persons[max_id], color=(255,0,0), width=int(10* results[max_id][1]))
                        center = (int((box[2]-box[0])/2+box[0]), int((box[3]-box[1])/2+box[1]))
                        cv2.putText(cv_img, f"{pos2d[0]}, {pos2d[1]}", center, cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 0, 255), 2, cv2.LINE_AA)
                        del persons[max_id]

            self.pos_pub.publish(target_pos)
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
            

        cv2.putText(cv_img, "fps: %.2f"%fps, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (0,255,0), 2, cv2.LINE_AA)
        
        self.visualizer_rw.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))
        self.visualizer_cp.publish(self.bridge.cv2_to_compressed_imgmsg(cv_img))
        


if __name__ == "__main__":
    node = GlobalTracking()
    rospy.spin()

