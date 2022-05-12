from importlib.resources import path
import yaml
import os
    #loafParam()  #loadURDF()  #initTerrain()  #initStair()  #subscribeROSmsg()  #runControl()
    #getDataFromRobot()  #publishROSmsg()
class robotSim():
    def __init__(self):
        path1=os.path.dirname(__file__)
        self.yaml_path=os.path.join(path1,'config','config.yaml')
        #print("yaml_path:",self.yaml_path)
        
    def loadParam(self):
        if os.path.exists(self.yaml_path):
            with open(file=self.yaml_path, mode="rb") as f:
                config = yaml.load(f, Loader=yaml.FullLoader)
                self.sim_mode = config.get("sim_mode")
                self.robot_name = config.get("robot_name")
                print("loading Parameter from yaml in robotSim finished")
        else:
            os.abort("can not find yaml's path in robotSim")
    def loadURDF(self):
        a= 1      
    def initSimulation(self):
        self.loadParam()
    


if __name__=='__main__':
    print("start simulation ")
    Cheetah=robotSim()
    Cheetah.initSimulation()
    #initSimulation()
    #runSimulation()