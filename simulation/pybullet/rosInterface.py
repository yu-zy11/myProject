import rospy
import time
import threading
# from ros_interface import FootsContact
from ros_msgs.msg import JointCommand
from ros_msgs.msg import RobotStates

class ROSInterface:
    def __init__(self):
        self.joint_q=[0]*12
        self.joint_dq=[0]*12
        self.body_p=[0]*3
        self.body_v=[0]*3
        self.body_rpy=[0]*3
        self.body_omega=[0]*3
        self.foot_p=[0]*12
        self.foot_v=[0]*12
        self.JointCommand=JointCommand()
        self.RobotStates=RobotStates()
        self.init_message()
        self.real_data_receive={}
        self.sim_data_receive={}

    def init_message(self):
        rospy.init_node('pybullet_sim',anonymous=False)
        self.pub=rospy.Publisher("/RobotStates",RobotStates,queue_size=10)
        rate=rospy.Rate(500)

    def commandCallback(self,data):
            self.JointCommand=data

    def publish(self):
        self.pub.publish(self.RobotStates)
        # print(["self.approximation",self.RobotStates])

    def subscriber(self):
        rospy.Subscriber("/JointCommand", JointCommand, self.commandCallback)
    def thread_spin(self):
        rospy.spin()
    

    
if __name__=='__main__':
    print("start Fitting ")
    rosinter=ROSInterface()
    rosinter.subscriber()
    #call back thread 
    thread=threading.Thread(target=thread_spin)
    thread.start()

    #main thread
    rosinter=ROSInterface()
    rosinter.subscriber()
    while True:
        rosinter.publish()
        # rosinter.subscriber()
        print(rosinter.real_data_receive)
        time.sleep(1.0)