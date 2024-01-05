#!/usr/bin/env python3
import rclpy

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

from z1_model import ModelZ1
import numpy as np
import random
from spatialmath import SO3, SE3, UnitQuaternion

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        self.subscription = self.create_subscription(
                JointState,
                '/joint_states',
                self.callback,
                10)
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controllers/commands',
            1
        )        
        # print(f'initial')
        
        robot_path = '/home/ccs/test_ws/src/z1_description/urdf/z1.urdf'
        self.z1 = ModelZ1(robot_path,display=False)
        
        self.q_left_des = np.array([0, 0.5, -1, 0.7, 0, 0])
        # self.q_left_des = np.array([0, 0, -0.05, 0, 0, 0])
        
        self.name_map = [7,8,9,1,0,2,3,4,5,6,10,11]
        
        self.error_sum = np.zeros_like(self.q_left_des)
        self.kp = np.array([50,50,50,50,50,100]) / 1
        self.kd = np.array([10,10,10,10,10,10]) / 1
        self.ki = np.array([0.2,0.2,0.2,0.2,0.2,1]) * 100
        
        # self.kd = [0,0,0,0,0,0]
        # self.ki = [0,0,0,0,0,0]
        
        
    def map_data(self,x):
        x_new = np.empty_like(x)
        for i in range(x.shape[0]):
            x_new[i] = x[self.name_map[i]]
            
        return x_new

 
    def callback(self, msg):
        q = self.map_data(np.array(msg.position))
        qd = self.map_data(np.array(msg.velocity))
        print(msg.header.stamp.sec + msg.header.stamp.nanosec/1.0e9)
        # print(q)
        # return 0
        q_left = q[:6]
        qd_left = qd[:6]
                
        kp = np.diag(self.kp)
        kd = np.diag(self.kd)
        ki = np.diag(self.ki)
        
        qdd_left = -kp @ (q_left - self.q_left_des) - kd @ qd_left - ki @ self.error_sum
        self.error_sum += (q_left - self.q_left_des) * 1/1000
        
        
        tau = self.z1.rnea0(q_left,qd_left,qdd_left) 
        
        m = self.z1.mass(q_left)
        c = self.z1.coriolis(q_left,qd_left)
        g = self.z1.generalized_gravity(q_left)
        
        tau2 = m @ qdd_left + c @ qd_left + g
        
        
        print(f'q: {q_left}')
        # print(f'tau: {tau}')
        # print(f'tau: {tau2}')
        
        tau = np.append(tau,np.zeros_like(tau))
        

        com = Float64MultiArray()
        com.data = tau.tolist()         
        self.publisher.publish(com)    
        
        # ros2 topic pub /effort_controllers/commands std_msgs/msg/Float64MultiArray "data:
        #     - 0.000364
        #     - 3.826922
        #     - -5.538757
        #     - -1.454266
        #     - 0.001112
        #     - -0.000459
        #     - 0.0
        #     - 0.0
        #     - 0.0
        #     - 0.0
        #     - 0.0 
        #     - 0.0"


def main(args=None):
    rclpy.init(args=args)

    node1 = TrajectoryController()
    # exec = rclpy.executors
    rclpy.spin(node1)
    # ros topic joint_state and TF
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()