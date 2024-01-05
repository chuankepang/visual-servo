#!/usr/bin/env python3
# import unitree_arm_interface
from roboticstoolbox.robot.ERobot import ERobot
from math import pi
import numpy as np
np.set_printoptions(precision=6, suppress=True)
import pinocchio as pin, math
import numpy as np
from matplotlib import pyplot as plt

from spatialmath import (
    SpatialAcceleration,
    SpatialVelocity,
    SpatialInertia,
    SpatialForce,
)
from spatialmath import SO3, SE3, UnitQuaternion


class ModelZ1Rtb(ERobot):
    def __init__(self,robot_path):
        links, name, urdf_string, urdf_filepath = self.URDF_read(robot_path)
        super().__init__(
            links,
            name=name.upper(),
            manufacturer="Unitree Z1",
            # gripper_links=links[7],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

class ModelZ1:
    """ Z1 model.
        This class implements the model of Z1
    """
    def __init__(self,robot_path,display=True):
        # self.arm =  unitree_arm_interface.ArmInterface(hasGripper = False)
        # self.armModel = self.arm._ctrlComp.armModel
        
        self.model_rtb = ModelZ1Rtb(robot_path)
        self.display = display
        if display:
            print(self.model_rtb)
        
        # load the urdf file of your robot model
        self.model_pin = pin.buildModelFromUrdf(robot_path)
        self.model_pin.gravity.linear[2] = -0
        # Create data required by the algorithms
        self.data = self.model_pin.createData()
        self.NQ = self.model_pin.nq
        self.NV = self.model_pin.nv
        
        if display:
            q = np.ones((self.NQ))
            qd = np.ones((self.NQ))
            print(self.model_pin)
            frame_num = len(self.model_pin.frames)
            print(frame_num)
            print([self.model_pin.frames[i].parent for i in range(frame_num)])
            
            # print(self.model_pin.getFrameId("ft_link"))
            # print(self.model_pin.getFrameId("camera_rgb_frame"))
            
            pin.computeJointJacobians(self.model_pin,self.data, q)
            # print(pin.getFrameJacobian(self.model_pin,self.data, 6, pin.WORLD)) #for a random frame id 100
            # print(pin.getJointJacobian(self.model_pin,self.data, self.model_pin.frames[6].parent, pin.WORLD)) #parent of the frame 100
            # print(pin.getFrameJacobian(self.model_pin,self.data, 6, pin.LOCAL_WORLD_ALIGNED))
            # print(pin.getJointJacobian(self.model_pin,self.data, self.model_pin.frames[6].parent, pin.LOCAL_WORLD_ALIGNED)) 
            # for i in range(frame_num):
            #     print('='*30)
            #     print(i)
            #     print(self.model_pin.frames[i].parent)
            #     print(pin.getFrameJacobian(self.model_pin,self.data, i, pin.WORLD))
            #     print(pin.getFrameJacobian(self.model_pin,self.data, i, pin.LOCAL_WORLD_ALIGNED))
            #     print(pin.getFrameJacobian(self.model_pin,self.data, i, pin.LOCAL))
                
            frame_name = 'camera_rgb_optical_frame' # left_camera_rgb_optical_frame
            frame_id_pin = self.model_pin.getFrameId(frame_name)
            frame_id = 10
            print(pin.getFrameJacobian(self.model_pin,self.data, frame_id_pin, pin.LOCAL_WORLD_ALIGNED))
            print(pin.getJointJacobian(self.model_pin,self.data, self.model_pin.frames[frame_id_pin].parent, pin.LOCAL_WORLD_ALIGNED))
            print(self.model_rtb.jacob0(q,end=self.model_rtb.link_dict[frame_name]))
            print('='*30)
            print(pin.getFrameJacobian(self.model_pin,self.data, frame_id_pin, pin.LOCAL))
            print(pin.getJointJacobian(self.model_pin,self.data, self.model_pin.frames[frame_id_pin].parent, pin.LOCAL))
            print(self.model_rtb.jacobe(q,end=self.model_rtb.link_dict[frame_name]))
            print('='*30)
            print(pin.getFrameJacobian(self.model_pin,self.data, frame_id_pin, pin.WORLD))
            print(pin.getJointJacobian(self.model_pin,self.data, self.model_pin.frames[frame_id_pin].parent, pin.WORLD))
            print('='*30)
                        
            # print(self.model_rtb.link_dict)
            print(self.model_rtb.link_dict[frame_name])
            
            print('='*30)
            
            j0 = pin.getFrameJacobian(self.model_pin,self.data, frame_id_pin, pin.LOCAL_WORLD_ALIGNED)
            je = pin.getFrameJacobian(self.model_pin,self.data, frame_id_pin, pin.LOCAL)
            jw = pin.getFrameJacobian(self.model_pin,self.data, frame_id_pin, pin.WORLD)
            r  = self.data.oMf[frame_id_pin].rotation
            p  = self.data.oMf[frame_id_pin].translation
            print(j0@qd)
            print('='*30)
            print(je@qd)
            print('='*30)
            print(jw@qd)
            print('='*30)
            rr = np.zeros((6,6))
            rr[:3,:3] = r
            rr[3:,3:] = r
            print(rr)
            print('='*30)
            print(rr@je@qd)
            print('='*30)
            ad = rr
            s = np.array([[0,-p[2],p[1]],[p[2],0,-p[0]],[-p[1],p[0],0]])
            ad[:3,3:] = s@r
            print(p)
            print(ad)
            print('='*30)
            print(ad@je@qd)
            print('='*30)
            # j = self.jacobian_dot(q,'camera_rgb_optical_frame',in_world=True)
            # print(j)
            
            
            # qd = np.ones((6))
            # id = self.model_pin.getFrameId('link06')
            
            # pin.computeJointJacobiansTimeVariation(self.model_pin,self.data,q,qd)
            # jd = pin.getFrameJacobianTimeVariation(self.model_pin,self.data, id, pin.LOCAL_WORLD_ALIGNED)
            # print(jd)
            # a = jd @ qd
            # print(a)
            # print('='*30)
            
            # jd = pin.frameJacobianTimeVariation(self.model_pin,self.data, q, qd, id, pin.LOCAL_WORLD_ALIGNED)  # 等价于上面
            # print(jd)
            # a = jd @ qd
            # print(a)
            # print('='*30)
            
            # jd = self.model_rtb.jacob0_dot(q,qd)
            # print(jd)
            # a = jd @ qd
            # print(a)
            # print('='*30)
            
            # pin.forwardKinematics(self.model_pin,self.data,q,qd,0*q)
            # a = pin.getFrameClassicalAcceleration(self.model_pin,self.data, id, pin.LOCAL_WORLD_ALIGNED) # LOCAL_WORLD_ALIGNED
            # print(a)
            
            # a = pin.getFrameAcceleration(self.model_pin,self.data, id, pin.LOCAL_WORLD_ALIGNED)
            # print(a)
            # print('='*30)
            
            q = np.ones((self.NQ))
            v = np.ones((self.NQ))
            a = np.zeros((self.NQ))
            frame_name = 'left_link06'
            frame_id = self.model_pin.getFrameId(frame_name)
            
            pin.forwardKinematics(self.model_pin,self.data, q, v, a)
            pin.computeJointJacobians(self.model_pin,self.data, q)
            pin.computeJointJacobiansTimeVariation(self.model_pin,self.data, q, v)
            pin.updateFramePlacements(self.model_pin,self.data)
            
            T_ee = self.data.oMf[frame_id]
            v_ee = pin.getFrameVelocity(
                self.model_pin,self.data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            ) # 求末端速度，表示在世界系
            v_ee = np.block([v_ee.linear, v_ee.angular])
            J = pin.getFrameJacobian(self.model_pin,self.data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED) # 末端雅克比，世界系
            print("J:", J)
            print("J_rtb:", self.model_rtb.jacob0(q,end=self.model_rtb.link_dict[frame_name]))
            print("v_ee: ", v_ee)
            print("J @ v: ", J @ v)
            assert np.allclose(v_ee, J @ v)
            print('='*30)
            
            a_ee = pin.getFrameClassicalAcceleration(
                self.model_pin,self.data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            ) # 求末端加速度（？？？？？？？哪个加速度项，包含动系的吗，看来是绝对加速度），表示在世界系
            a_ee = np.block([a_ee.linear, a_ee.angular])
            
            dJ_s = pin.getFrameJacobianTimeVariation(
                self.model_pin,self.data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            omega = v_ee[3:6]
            skew = np.matrix(
                [[0, -omega[2], omega[1]], [omega[2], 0, -omega[0]], [-omega[1], omega[0], 0]]
            )
            dJ = dJ_s + np.block([[skew @ J[0:3, :]], [skew @ J[3:6, :]]])

            print("dJ:", dJ)
            # print("dJ_rtb:", self.model_rtb.jacob0_dot(q,v))
            
            # print("dJ_hessian0:", self.model_rtb.hessian0(q,end=self.model_rtb.link_dict[frame_name]))
            
            self.model_rtb.hessian0
            print("a_ee: ", a_ee)        
            print("J @ a + dJ @ v: ", J @ a + dJ @ v)    
            assert np.allclose(a_ee, J @ a + dJ @ v)
            
            print('='*30)

    
    def fkine(self,q,name,is_frame=True):
        # pin.forwardKinematics(self.model_pin,self.data,q)
        # pin.updateFramePlacements(self.model_pin,self.data)
        
        pin.framesForwardKinematics(self.model_pin,self.data, q) # 等于上面两句
        
        if is_frame:
            id = self.model_pin.getFrameId(name)
            fk = self.data.oMf[id]
            if self.display:
                fk_rtb = self.model_rtb.fkine(q,end=self.model_rtb.link_dict[name]) # SE3
                print(f'fk: {fk}')
                print(f'fk_rtb: {fk_rtb}') 
        else:
            id = self.model_pin.getJointId(name)
            # print(id)
            fk = self.data.oMi[id]
            # if self.display:
            #     fk_rtb = self.model_rtb.fkine(q,end=self.model_rtb.links[id+2]) # SE3
            #     print(f'fk: {fk}')
            #     print(f'fk_rtb: {fk_rtb}') 
            
        return np.array(fk.translation), np.array(fk.rotation)
            
    
    def ikine(self,pos,name,is_frame=True):
        pos = SE3(pos)
        # return self.model_rtb.ikine_LMS(pos,end=self.model_rtb.link_dict[name]).q
        return self.model_rtb.ikine_LM(pos,end=self.model_rtb.link_dict[name]).q
    
    
    def jacobian(self,q,name,is_frame=True,in_world=True):
        pin.computeJointJacobians(self.model_pin,self.data,q)
        if in_world:
            if is_frame:
                id = self.model_pin.getFrameId(name)
                j = pin.getFrameJacobian(self.model_pin,self.data, id, pin.LOCAL_WORLD_ALIGNED)
                if self.display:
                    print(f'{j} \n {self.model_rtb.jacob0(q,end=self.model_rtb.link_dict[name])}')
                    
            else:
                id = self.model_pin.getJointId(name)
                j = pin.getJointJacobian(self.model_pin,self.data,id,pin.LOCAL_WORLD_ALIGNED)
                # if self.display:
                #     print(f'{j} \n {self.model_rtb.jacob0(q,end=self.model_rtb.links[id])}')
        else:
            if is_frame:
                id = self.model_pin.getFrameId(name)
                j = pin.getFrameJacobian(self.model_pin,self.data, id, pin.LOCAL)
                if self.display:
                    print(f'{j} \n {self.model_rtb.jacobe(q,end=self.model_rtb.link_dict[name])}')
                    
            else:
                id = self.model_pin.getJointId(name)
                j = pin.getJointJacobian(self.model_pin,self.data,id,pin.LOCAL)
                # if self.display:
                #     print(f'{j} \n {self.model_rtb.jacobe(q,end=self.model_rtb.links[id])}')

        return j
    
    # def jacobian_dot(self,q,name,is_frame=True,in_world=True):
    #     if in_world:
    #         j = self.model_rtb.jacob0_dot(q,end=self.model_rtb.link_dict[name])
            
    #     else:
    #         j = self.model_rtb.jacobe(q,end=self.model_rtb.link_dict[name])
            
    #     return j
    
    # 绝对加速度中的项
    def classical_acceleration(self,q,qd,qdd, name,is_frame=True,in_world=True):
        pin.forwardKinematics(self.model_pin,self.data, q, qd, qdd)
        if in_world:
            id = self.model_pin.getFrameId(name)
            ac = pin.getFrameClassicalAcceleration(self.model_pin,self.data, id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        else:
            id = self.model_pin.getFrameId(name)
            ac = pin.getFrameClassicalAcceleration(self.model_pin,self.data, id, pin.ReferenceFrame.LOCAL)
        return np.block([ac.linear, ac.angular])
    
    def rnea(self,q,qd,qdd,fext):
        return pin.rnea(self.model_pin,self.data,q,qd,qdd,fext)
    
    def rnea0(self,q,qd,qdd):
        return pin.rnea(self.model_pin,self.data,q,qd,qdd)
        
    def mass(self,q):
        # Computes the upper triangular part of the joint space inertia matrix M, stored in data.M
        return pin.crba(self.model_pin,self.data,q) # data.M
    
    def coriolis(self,q,qd):
        # Computes the Coriolis Matrix C
        return pin.computeCoriolisMatrix(self.model_pin,self.data, q, qd)  # data.C=C(q,dq/dt)
    
    def generalized_gravity(self,q):
        return pin.computeGeneralizedGravity(self.model_pin,self.data,q) # data.g =rnea(self.model_pin,self.data, q, 0, 0)

    def static_torque(self,q,fext):
        # data.tau=G(q)-sum(J^T(q)*fext)  =rnea(self.model_pin,self.data, q, 0, 0, fext)
        return pin.computeStaticTorque(self.model_pin,self.data,q,fext)  # data.tau  

    # SDK
    # def fk_hardware(self,q):
    #     return self.armModel.forwardKinematics(q, 6)

    # def ik_hardware(self,T):
    #     # hasIK, q_forward
    #     return self.armModel.inverseKinematics(T, np.zeros(6), True)

    # def ik_qp_hardware(self,T,q_near):
    #     return self.armModel.inverseKinematics(T, q_near, False)
    
    # def inverseDynamics_hardware(self):
    #     # The torque required by the z1 arm at the homo position to resist gravity
    #     return self.armModel.inverseDynamics(np.zeros(6), np.zeros(6), np.zeros(6), np.zeros(6))
    
    # def jacobian_hardware(self,q):
    #     return self.armModel.CalcJacobian(q)
    
    # def jacobian_qdot_hardware(self,V,q_near):
    #     return self.armModel.solveQP(V, q_near, self.arm._ctrlComp.dt)       


if __name__ == '__main__':
    robot_path = '/home/ccs/ROS2/gz_ws/src/z1_description/urdf/z1.urdf'
    # robot_path = '/home/ccs/ROS2/gz_ws/src/z1_description/urdf/z1_dual.urdf'
    
    model = ModelZ1(robot_path)   
  
    q = np.zeros((model.NQ))
    qd = np.ones((model.NQ))
    qdd = np.ones((model.NQ))
    fk = model.fkine(q,'left_camera_rgb_optical_frame')
    fk = model.fkine(q,'left_joint6',is_frame=False)
    print('='*30)
    
    j = model.jacobian(q,'left_camera_rgb_optical_frame')
    print('='*30)
    j = model.jacobian(q,'left_joint6',is_frame=False)
    print('='*30)
    j = model.jacobian(q,'left_camera_rgb_optical_frame',in_world=False)
    print('='*30)
    j = model.jacobian(q,'left_joint6',is_frame=False,in_world=False)
    print('='*30)
    
    print(f'Torque is: {model.rnea0(q,qd,qdd)}')
    fext = [pin.Force.Zero() for j in range(model.NQ+1)]
    print(f'Torque is: {model.rnea(q,qd,qdd,fext)}')
    
    fext[5].linear[1] += 100.0  # N linear三个轴不一样
    print(f'Torque is: {model.rnea(q,qd,qdd,fext)}')
    
    # f_ext[model.getJointId("right_wheel")].linear[2] += 100.0  # N
    # print(f'Torque is: {model.rnea(q,qd,qdd)}')
    
    # Computes the upper triangular part of the joint space inertia matrix M, stored in data.M
    M = model.mass(q)
    print('M_Matrix is: ', M)

    # Computes the Coriolis Matrix C
    C = model.coriolis(q, qd)
    print('C_Matrix is: ', C)
    
    # Verify the anti-symmetric property of dM/dt - 2* C, take the fifth sequence as example
    dt = 1e-8
    q_plus = pin.integrate(model.model_pin,q,qd*dt)
    M_plus = model.mass(q_plus)
    dM = (M_plus - M)/dt
    A = dM - 2*C
    print('A is: ', A)
    res = A + A.T
    print('res is: ', res)
    
    
