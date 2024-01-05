#!/usr/bin/env python3
import rclpy

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import TransformStamped

from z1_model import ModelZ1
import numpy as np
import random
from spatialmath import SO3, SE3, UnitQuaternion
import message_filters
from cv_bridge import CvBridge
import cv2


class VisualServo(Node):
    def __init__(self):
        super().__init__('visual_servoing_controller')
        self.sub_joint_state = self.create_subscription(
                JointState,
                '/joint_states',
                self.callback_joint_state,
                10)
        self.sub_image = self.create_subscription(
                Image,
                '/left_camera/image',
                self.callback_image,
                2)
        self.sub_camera_info = self.create_subscription(
                Image,
                '/left_camera/camera_info',
                self.callback_camera_info,
                1) 
        self.cv_bridge = CvBridge() 
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controllers/commands',
            1
        )        
        # print(f'initial')
        
        robot_path = '/home/ccs/test_ws/src/z1_description/urdf/z1.urdf'
        self.z1 = ModelZ1(robot_path,display=False)
        
        self.name_map = [7,8,9,1,0,2,3,4,5,6,10,11]
        size = 40.
        # self.s_left_des = np.array([[100,60],[220,60],[220,180],[100,180]]).astype('float64')
        self.s_left_des = np.array([[160-size,120-size],[160+size,120-size],[160+size,120+size],[160-size,120+size]]).astype('float64')
        
        # self.s_left = np.empty_like(self.s_left_des)
        self.s_left = None
        self.v = np.zeros((6))
        # self.q_left_init = np.array([0, 0, -0.1, -0.1, 0, 0])
        # self.q_left_init = np.array([0, 0.5, -1, 0.7, 0, 0])
        
        self.q_left_init = np.array([0, 1.7, -1.7, 0.2, 0, 0])
        
        self.q_left_init = np.array([-1, 1.8, -1.0, -0.4, 1, 0])
        
        self.error_sum = np.zeros(self.z1.NQ)
        self.error_sum_vs = np.zeros(self.z1.NQ)
        # print(self.error_sum)
        self.kp = np.array([50,50,50,50,50,100])
        self.kd = np.array([10,10,10,10,10,10])
        self.ki = np.array([0.2,0.2,0.2,0.2,0.2,1]) * 1000
        
        # self.kd = np.array([0,0,0,0,0,0])
        self.ki = np.array([0,0,0,0,0,0])
        
        self.init_fineshed = 0
        
        self.tau_last = None
        
        # 
        #Load the dictionary that was used to generate the markers.
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)

        # Initialize the detector parameters using default values
        self.parameters =  cv2.aruco.DetectorParameters_create()
        
        self.camera_matrix = np.array([[277.,0.,   160.],[0.,  277., 120.],[0,  0,   1.]])
        self.camera_dist = np.array([0., 0., 0., 0., 0.] )
        
        self.count = 1

        
    def map_data(self,x):
        x_new = np.empty_like(x)
        for i in range(x.shape[0]):
            x_new[i] = x[self.name_map[i]]
            
        return x_new
    
    def detect_aruco(self,img):
        # 4.7.x修改了API
        # dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        # parameters =  cv.aruco.DetectorParameters()
        # detector = cv.aruco.ArucoDetector(dictionary, parameters)
        # frame = cv.imread(...)
        # markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)
        
        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(img, self.dictionary, parameters=self.parameters)
        # print(f'markerCorners: {markerCorners}')
        # print(f'markerIds: {markerIds}')
                
        if markerIds is not None:   
            self.s_left = markerCorners[0][0]  # list np 1 4 2
                    
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.05, self.camera_matrix, self.camera_dist)
            # (rvec-tvec).any() 
            # 其实只有一个aruco
            for i in range(rvec.shape[0]):
                cv2.drawFrameAxes(img, self.camera_matrix, self.camera_dist, rvec[i, :, :], tvec[i, :, :], 0.03)
                cv2.aruco.drawDetectedMarkers(img, markerCorners)
        else:
            self.s_left = None
            cv2.putText(img, "No Ids", (0,64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2,cv2.LINE_AA)  

        cv2.imshow("frame",img)  
        key = cv2.waitKey(1)
        
    def callback_camera_info(self, msg):
        pass
        
        
    def callback_image(self, imgmsd):
        img = self.cv_bridge.imgmsg_to_cv2(imgmsd)
        self.detect_aruco(img)
                
        # 理论上，应该返回一个V或者A，然后被callback_joint_state调用，相当于分内环和外环
        def norm_point(p):# 4*2
            p_tmp = np.empty_like(p)
            p_tmp[:,0] = (p[:,0] - self.camera_matrix[0,2]) / self.camera_matrix[0,0]
            p_tmp[:,1] = (p[:,1] - self.camera_matrix[1,2]) / self.camera_matrix[1,1]
            # p[:,1] -= self.camera_matrix[1,2] # 不能这么写
            return p_tmp
 
                
        if self.s_left is not None:
            # print(f's_left: {self.s_left}')
            # print(f'shape: {self.s_left.shape}')
            # print(f'dtype: {self.s_left.dtype}')
            # print(f'sd: {self.s_left_des}')

            s_left_norm = norm_point(self.s_left)
            s_left_des_norm = norm_point(self.s_left_des)
            # print(f's_left_norm: {s_left_norm}')
            # x = s_left_norm[:,0].T
            # y = s_left_norm[:,1].T
            # z = 1
            # zi = 1/z * np.ones_like(x)
            # zero = np.zeros_like(x)
            
            # print(f'x: {x}')
            # print(f'y: {y}')
            # print(f'zi: {zi}')
            # print(f'zero: {zero}')
            # print(f'lx: {[-zi, zero, x * zi, x * y, -(1 + x ** 2), y]}')
            
            # L = np.array([
            #     [-zi, zero, x * zi, x * y, -(1 + x ** 2), y],
            #     [zero, -zi, y * zi, 1 + y ** 2, -x * y, -x],
            # ])
            # L = L.T
            # print(f'L: {L}')
            z = 1
            zi = 1/z
            L = []
            for p in s_left_norm:
                x = p[0]
                y = p[1]
                L.append(np.array([
                    [-zi, 0, x * zi, x * y, -(1 + x ** 2), y],
                    [0, -zi, y * zi, 1 + y ** 2, -x * y, -x],
                ]))
            L = np.array(L).reshape(-1,6)
            # print(f'L shape: {L.shape}')
            # print(f'L: {L}')
            
            assert L.shape == (8, 6)
            
            error = s_left_norm.flatten() - s_left_des_norm.flatten() # 一行一行展开
            # print(f'L: {L}')
            if self.init_fineshed:
                print(f's: {s_left_norm.flatten()}')
                print(f'sd: {s_left_des_norm.flatten()}')

            self.v = -1 * np.linalg.pinv(L) @ error
            
        else:
            self.v = np.zeros((6))
            
        
 
    def callback_joint_state(self, joints):
        q_all = self.map_data(np.array(joints.position))
        qd_all = self.map_data(np.array(joints.velocity))
        # print(self.init_fineshed)
        # print(joints.header.stamp.sec + joints.header.stamp.nanosec/1.0e9)
        
        q_left = q_all[:6]
        qd_left = qd_all[:6]
        if not self.init_fineshed:
            # 单个机械臂的位置控制。TODO：轨迹规划
            kp = np.diag(self.kp)
            kd = np.diag(self.kd)
            ki = np.diag(self.ki)
                
            qdd = -kp @ (q_left - self.q_left_init) - kd @ qd_left - ki @ self.error_sum
            self.error_sum += (q_left - self.q_left_init) * 1/1000 # 周期
            tau = self.z1.rnea0(q_left,qd_left,qdd) 
            
            print(tau)
            
            self.tau_last = tau
            if np.linalg.norm(q_left-self.q_left_init) < 0.0001:
                self.init_fineshed = 1
            if self.count % 100 == 0:
                print(f'='*30)
                print(f'q: {q_left}')
                # print(f'tau: {tau}')
                # print(f'g: {self.z1.generalized_gravity(q_left)}')
                # print(f'g_des: {self.z1.generalized_gravity(self.q_left_init)}')
        else:
            # visual servoing     
            # print(f'v: {self.v}')
                                
            j = self.z1.jacobian(q_left,'camera_rgb_optical_frame',in_world=False)
            qd_des = np.linalg.pinv(j) @ self.v
            # print(f'qd_des: {qd_des}')
            
            kp = np.diag(np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])) * 5
            ki = np.diag(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))
        
            # qdd = - kp @ (qd_left - qd_des) - ki @ self.error_sum_vs  # 
            qdd = - kp @ (qd_left - qd_des)
            self.error_sum_vs += (qd_left - qd_des) * 1/1000 # 周期
            # print(f'qdd: {qdd}')
            tau = self.z1.rnea0(q_left,qd_left,qdd)

        # tau = np.zeros_like(q_all)
        # print(f'tau: {tau}')
        tau = np.append(tau,np.zeros_like(tau))
        com = Float64MultiArray()
        com.data = tau.tolist()         
        self.publisher.publish(com)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node1 = VisualServo()
    rclpy.spin(node1)
    rclpy.shutdown()
 

if __name__ == '__main__':
    main()