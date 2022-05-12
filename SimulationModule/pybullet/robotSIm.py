
from lib2to3.pytree import BasePattern
import yaml
import os
import pybullet as p
import pybullet_data
    #loafParam()  #loadURDF()  #initTerrain()  #initStair()  #subscribeROSmsg()  #runControl()
    #getDataFromRobot()  #publishROSmsg()
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
        p.resetDebugVisualizerCamera(1, 45, -30, [0, 0, 0.5])
        print("self.sim_step:",self.sim_step)
        
    def runSimulation(self):
        while p.isConnected:
            
            p.stepSimulation()
        
    


if __name__=='__main__':
    print("start simulation ")
    Cheetah=robotSim()
    Cheetah.initSimulation()
    Cheetah.runSimulation()
