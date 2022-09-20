import numpy as np
import yaml
import os
import pybullet as p
import pybullet_data
import random
import time
import sys
from lcm_message.lcm_interface import LCMInterface
from rosInterface import  ROSInterface
import rospy
import threading
# import lcm_message.lcm_msgs 
class robotSim():
    def __init__(self):
        self.run_path=os.path.dirname(__file__)
        self.yaml_path=os.path.join(self.run_path,'config','config.yaml')
        self.loadConfig()
        if self.urdf_name=="px2_et1":
            self.urdf_path=os.path.join(self.run_path,'../URDF/px2_et1/px2_et1.urdf')
        elif self.urdf_name=="laikago":
            self.urdf_path=os.path.join(self.run_path,'../URDF/laikago/urdf/laikago.urdf')
        else:
            print("wrong urdf name")
            os.abort()
        self.joint_kp=[0]*12
        self.joint_kd=[0]*12
        self.tau_ff=[0]*12
        self.q_des=[0]*12
        self.dq_des=[0]*12
        self.ddq_des=[0]*12
        self.body_position=[0]*3
        self.body_velocity=[0]*3
        self.body_quaternion=[0]*4 #x y z w
        self.body_euler=[0]*4 #x y z w
        self.joint_position=[0]*12
        self.joint_velocity=[0]*12
        self.joint_torque=[0]*12
        self.joint_command=[]
        self.msgs_channel=ROSInterface()
        self.jointsID=[]
    def subscribeROSmsg(self):
        self.msgs_channel.subscriber()

    def loadConfig(self):
        if os.path.exists(self.yaml_path):
            with open(file=self.yaml_path, mode="rb") as f:
                config = yaml.load(f, Loader=yaml.FullLoader)
                self.sim_mode = config.get("sim_mode")
                self.urdf_name = config.get("urdf_name")
                self.useFixedBase=config.get("useFixedBase")
                self.message_type=config.get("message_channle")
                self.sim_step = config.get("sim_step")
                self.basePosition=config.get("basePosition")
                self.baseOrientation=config.get("baseOrientation")
                self.gravity=config.get("gravity")
                print("loading Parameter from yaml in robotSim finished")
        else:
            print("wrong!,can not find yaml's path in robotSim!")
            os.abort()
            
    def loadURDF(self):       
            self.robot_id=p.loadURDF(self.urdf_path,basePosition=self.basePosition,baseOrientation=self.baseOrientation, useFixedBase=self.useFixedBase)
            if self.robot_id<0:
                print("loading URDF failed!")
                os.abort()
            else:
                print("loading laikago URDF finished!")

    def updateRobotStates(self):
        self.body_velocity = p.getBaseVelocity(self.robot_id)
        self.body_position, self.body_quaternion = p.getBasePositionAndOrientation(self.robot_id)
        self.body_euler = p.getEulerFromQuaternion(self.body_quaternion)
        joint_states = p.getJointStates(self.robot_id, self.jointsID)
        for i in np.arange(len(self.jointsID)):
            self.joint_position[i]=joint_states[i][0]
            self.joint_velocity[i]=joint_states[i][1]
        # rotation matrix from world to body
        self.msgs_channel.RobotStates.body_position=self.body_position
        self.msgs_channel.RobotStates.body_quaternion=self.body_quaternion
        self.msgs_channel.RobotStates.body_euler=self.body_euler
        self.msgs_channel.RobotStates.body_linear_velocity_in_world=self.body_velocity[0]
        self.msgs_channel.RobotStates.body_angular_velocity_in_world=self.body_velocity[1]
        rotation_matrix = p.getMatrixFromQuaternion(self.body_quaternion)
        root_rotation_matrix = np.array(rotation_matrix).reshape(3, 3)
        velocity=root_rotation_matrix.transpose()@np.array(self.body_velocity[0]).reshape(3,1)
        self.msgs_channel.RobotStates.body_linear_velocity_in_body=velocity.reshape(1,3).tolist()[0]
        angular_velocity=root_rotation_matrix.transpose()@np.array(self.body_velocity[1]).reshape(3,1)
        self.msgs_channel.RobotStates.body_angular_velocity_in_body=angular_velocity.reshape(1,3).tolist()[0]

        self.msgs_channel.RobotStates.joint_position=self.joint_position
        self.msgs_channel.RobotStates.joint_velocity=self.joint_velocity


    def publishStates(self):
        self.msgs_channel.publish()

    def updateCommand(self):
        self.joint_command=self.msgs_channel.JointCommand
        print(self.joint_command)        
    
    def runLegControl(self):
        joint_kp=[1]*self.joint_num
        joint_kd=[0]*self.joint_num
        tau_ff=[0]*self.joint_num
        q_des=[0]*self.joint_num
        dq_des=[0]*self.joint_num
        q_data=self.joint_position
        dq_data=self.joint_velocity
        
        tau=[1]*self.joint_num
        for i in np.arange(self.joint_num):
            tau[i]=joint_kp[i]*(q_des[i]-q_data[i])+joint_kd[i]*(dq_des[i]-dq_data[i])+tau_ff[i]
        p.setJointMotorControlArray(bodyUniqueId=self.robot_id,jointIndices=self.jointsID,controlMode=p.TORQUE_CONTROL,forces=tau)   
         
    def initSimulation(self): #the use of function in pybullet refer to https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit# 
        physicsClient=p.connect(p.GUI)                       #connect to GUI physics server
        if not p.isConnected:
            print("wrong!,can not connect to pybullet physics server")
            os.abort()  
        self.loadURDF()          
        p.setGravity(self.gravity[0],self.gravity[1],self.gravity[2])
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #use pybullet data package
        p.setTimeStep(self.sim_step)
        print("set simulation step:",self.sim_step)
        # self.joint_num=p.getNumJoints(self.robot_id)
        self.jointsID=[]
        for j in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, j)
            if joint_info[2] == p.JOINT_REVOLUTE:
                self.jointsID.append(j)
            elif joint_info[2] == p.JOINT_FIXED:
                a=1
            else:
                print('Error: no available joint type.')
        self.joint_num=len(self.jointsID)
        for joint_id in self.jointsID:#disable velocity control mode ,and reset joint control mode to torque control
            p.setJointMotorControl2(self.robot_id, joint_id,controlMode=p.VELOCITY_CONTROL, force=0)                 
        print("init simulation finished!")
        
    def runSimulation(self):
        thread=threading.Thread(target=self.msgs_channel.thread_spin)
        thread.start()
        self.subscribeROSmsg()#订阅信息
        while p.isConnected:
            begin_time = time.time()
            self.updateRobotStates()
            self.publishStates()
            self.runLegControl()
            p.stepSimulation()
            print(self.joint_command)
            current_time=time.time()
            end_time=begin_time+self.sim_step
            run_time=current_time-begin_time
            if run_time>self.sim_step:
                print("running time is bigger than simulation step,please change sim_step ")
            while current_time<end_time:
                current_time=time.time()
    


if __name__=='__main__':
    print("start simulation ")
    Rsim=robotSim()
    Rsim.initSimulation() #load urdf and set pyubullet
    Rsim.runSimulation() 
    
