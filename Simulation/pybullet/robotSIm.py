
from lib2to3.pytree import BasePattern
from numpy import arange
import yaml
import os
import pybullet as p
import pybullet_data
import random
    #loafParam()  #loadURDF()  #initTerrain()  #initStair()  #subscribeROSmsg()  #runControl()
    #getDataFromSim()  #publishROSmsg()
class robotSim():
    def __init__(self):
        self.run_path=os.path.dirname(__file__)
        self.yaml_path=os.path.join(self.run_path,'config','config.yaml')
        self.laikago_urdf_path=os.path.join(self.run_path,'..','URDF','laikago','urdf','laikago.urdf')
        self.mini_cheetah_urdf_path=os.path.join(self.run_path,'..','URDF','mini_cheetah','mini_cheetah.urdf')
        #print("yaml_path:",self.yaml_path)
        
    def loadParameter(self):
        if os.path.exists(self.yaml_path):
            with open(file=self.yaml_path, mode="rb") as f:
                config = yaml.load(f, Loader=yaml.FullLoader)
                self.sim_mode = config.get("sim_mode")
                self.robot_name = config.get("robot_name")
                self.sim_step=config.get("sim_step")
                print("loading Parameter from yaml in robotSim finished")
        else:
            print("wrong!,can not find yaml's path in robotSim!")
            os.abort()
            
    def loadURDF(self):         
        if self.robot_name=="laikago":
            self.robotID=p.loadURDF(self.laikago_urdf_path,basePosition=[0,0,1.0],useFixedBase=True)
            print("loading laikago URDF finished!")
        elif self.robot_name=="mini_Cheetah":
            self.robotID=p.loadURDF(self.mini_cheetah_urdf_path,basePosition=[0,0,1.0],useFixedBase=True)
            print("loading mini_Cheetah URDF finished!")
        else:
            print("cannot load URDF,plese check your model !")
            os.abort()
    def addTerrain(self):
        colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=0.3)
        geom1=p.createCollisionShape(p.GEOM_BOX, halfExtents=[1,1,1])
        terrainID=p.createMultiBody(geom1,colSphereId)
    def initTerrain(self):
        heightPerturbationRange = 0.06
        numHeightfieldRows = 256
        numHeightfieldColumns = 256
        heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns
        for j in range(int(numHeightfieldColumns/2)):
            for i in range(int(numHeightfieldRows/2)):
                height = random.uniform(0, heightPerturbationRange)
                heightfieldData[2*i+2*j*numHeightfieldRows] = height
                heightfieldData[2*i+1+2*j*numHeightfieldRows] = height
                heightfieldData[2*i+(2*j+1)*numHeightfieldRows] = height
                heightfieldData[2*i+1+(2*j+1)*numHeightfieldRows] = height
        terrainShape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, meshScale=[.05, .05, 1], heightfieldTextureScaling=(
            numHeightfieldRows-1)/2, heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns)
        ground_id = p.createMultiBody(0, terrainShape)
        p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
        return ground_id
    def getDataFromSim(self):
        a=1
    def subscribeROSmsg(self):
        a=1
    
    def runLegControl(self):
        joint_kp=[1]*self.joint_num
        joint_kd=[1]*self.joint_num
        tau_ff=[0]*self.joint_num
        q_des=[0]*self.joint_num
        dq_des=[0]*self.joint_num
        q_data=[0]*self.joint_num
        dq_data=[0]*self.joint_num
        
        tau=[1]*self.joint_num
        #print("self.jointsID",self.jointsID)
        for i in arange(self.joint_num):
            tau[i]=joint_kp[i]*(q_des[i]-q_data[i])+joint_kd[i]*(dq_des[i]-dq_data[i])+tau_ff[i]
        p.setJointMotorControlArray(bodyUniqueId=self.robotID,jointIndices=self.jointsID,controlMode=p.TORQUE_CONTROL,forces=tau)   
         
    def initSimulation(self): #the use of function in pybullet refer to https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit# 
        self.loadParameter()
        physicsClient=p.connect(p.GUI)                       #connect to GUI physics server
        p.resetSimulation()                                  #remove all objects from the world and reset the world to initial conditions
        if not p.isConnected:
            print("wrong!,can not connect to pybullet physics server")
            os.abort()  
        self.loadURDF()          
        p.setGravity(0,0,-9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #use pybullet data package
        p.setTimeStep(self.sim_step)
        print("set simulation step:",self.sim_step)
        p.resetDebugVisualizerCamera(1, 45, -30, [0, 0, 0.5])
        # self.initTerrain()
        self.addTerrain()
        self.joint_num=p.getNumJoints(self.robotID)
        self.jointsID=arange(self.joint_num)
        #disable velocity control mode ,and reset joint control mode to torque control
        for i in arange(self.joint_num):
            p.setJointMotorControl2(self.robotID, self.jointsID[i],controlMode=p.VELOCITY_CONTROL, force=0)
            
            
        print("init simulation finished!")
        
    def runSimulation(self):
        while p.isConnected:
            self.subscribeROSmsg()
            self.getDataFromSim()
            self.runLegControl()
            p.stepSimulation()
        
    


if __name__=='__main__':
    print("start simulation ")
    Cheetah=robotSim()
    Cheetah.initSimulation()
    Cheetah.runSimulation()
    
