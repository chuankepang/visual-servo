#!/usr/bin/env python3
import rclpy

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Wrench

from z1_model import ModelZ1
import numpy as np
import random
from spatialmath import SO3, SE3, UnitQuaternion
import time

class CarryingController(Node):
    def __init__(self):
        super().__init__('carrying_controller')
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10)
        # self.wrench_subscriber = self.create_subscription(
        #     Wrench,
        #     '/joint_states',
        #     self.wrench_callback,
        #     2)
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controllers/commands',
            1
        )
        
        
        robot_path = '/home/ccs/test_ws/src/z1_description/urdf/z1_dual.urdf'
        self.z1 = ModelZ1(robot_path,display=False)
        
        robot_path = '/home/ccs/test_ws/src/z1_description/urdf/z1.urdf'
        self.z1_singel = ModelZ1(robot_path,display=False)
        self.q_init = np.array([0, 0.5, -1, 0.7, 0, 0, 0, 0.5, -1, 0.7, 0, 0])
        
        pos = SE3(0.5,-0.2,0.2)
        rot = SE3(SO3(np.array([[1, 0, 0],[0,0,-1],[0,1,0]])))
        q_left = self.z1_singel.ikine(pos * rot,'ft_link',is_frame=True)
        print(q_left)
        
        pos = SE3(0.5,0.2,0.2)
        rot = SE3(SO3(np.array([[-1, 0, 0],[0,0,1],[0,1,0]])))
        q_right = self.z1_singel.ikine(pos * rot,'ft_link',is_frame=True)
        print(q_right)
        
        self.q_init = np.block([q_left,q_right])
        
        pos = SE3(0.5,-0.3,0.2)
        rot = SE3(SO3(np.array([[1, 0, 0],[0,0,-1],[0,1,0]])))
        q_left = self.z1_singel.ikine(pos * rot,'ft_link',is_frame=True)

        pos = SE3(0.5,0.3,0.2)
        rot = SE3(SO3(np.array([[-1, 0, 0],[0,0,1],[0,1,0]])))
        q_right = self.z1_singel.ikine(pos * rot,'ft_link',is_frame=True)
    
        self.q_init2 = np.block([q_left,q_right])
        
        pos = SE3(0.5,-0.3,0.3)
        rot = SE3(SO3(np.array([[1, 0, 0],[0,0,-1],[0,1,0]])))
        q_left = self.z1_singel.ikine(pos * rot,'ft_link',is_frame=True)

        pos = SE3(0.5,0.3,0.3)
        rot = SE3(SO3(np.array([[-1, 0, 0],[0,0,1],[0,1,0]])))
        q_right = self.z1_singel.ikine(pos * rot,'ft_link',is_frame=True)
    
        self.q_init3 = np.block([q_left,q_right])
        
        self.name_map = [7,8,9,1,0,2,3,4,5,6,10,11]
        
        self.kp = np.array([50,50,50,50,50,100,50,50,50,50,50,100]) * 1
        self.kd = np.array([10,10,10,10,10,10,10,10,10,10,10,10]) / 1
        # self.ki = np.array([0.2,0.2,0.2,0.2,0.2,1,0.2,0.2,0.2,0.2,0.2,1]) * 1000
        self.ki = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
        self.error_sum = np.zeros(self.z1.NQ)
        
        self.state = 0
        self.count = 0
        
        self.t_start = None
        
    def map_data(self,x):
        x_new = np.empty_like(x)
        for i in range(x.shape[0]):
            x_new[i] = x[self.name_map[i]]
            
        return x_new

 
    def callback(self, msg):
        q = self.map_data(np.array(msg.position))
        qd = self.map_data(np.array(msg.velocity))       
        
        kp = np.diag(self.kp)
        kd = np.diag(self.kd)
        ki = np.diag(self.ki)
        
        
        
        if np.linalg.norm(q-self.q_init) < 0.001:
            self.state = 1    
            self.t_start = time.time()
            
        if self.t_start is not None:
            if time.time() - self.t_start > 10:
                self.state = 2
        
        if self.state == 0:
            qdd = -kp @ (q - self.q_init) - kd @ qd - ki @ self.error_sum
            self.error_sum += (q - self.q_init) * 1/1000
            
        elif self.state == 1:
            qdd = -kp @ (q - self.q_init2) - kd @ qd - ki @ self.error_sum
            self.error_sum += (q - self.q_init2) * 1/1000
            
        elif self.state == 2:
            qdd = -kp @ (q - self.q_init3) - kd @ qd - ki @ self.error_sum
            self.error_sum += (q - self.q_init3) * 1/1000

        tau = self.z1.rnea0(q,qd,qdd) 
        
        # m = self.z1.mass(q)
        # c = self.z1.coriolis(q,qd)
        # g = self.z1.generalized_gravity(q)
        
        # tau2 = m @ qdd + c @ qd + g
        # tau = 0.01 * qdd
        
        # print(f'tau: {tau}')
        # print(f'tau: {tau2}')    
        if self.count % 100 == 0:
            print(f'='*30)
            print(f'q: {q}')
            print(f'q_init: {self.q_init}')
            print(f'state: {self.state}')
 
            # name = 'left_ft_link'
            # print(f'fk: {self.z1.fkine(q,name)}')
            # print(f'tau: {tau}')
            # print(f'g: {self.z1.generalized_gravity(q)}')
            # print(f'g_des: {self.z1.generalized_gravity(self.q_init)}')   

        com = Float64MultiArray()
        com.data = tau.tolist()         
        self.publisher.publish(com)    
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = CarryingController()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()