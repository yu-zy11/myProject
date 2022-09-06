import imp
import lcm
from lcm_msgs import states, command
import time

class LCMInterface:
    def __init__(self) :
        self.lc=lcm.LCM()
        self.cmd_msgs=command()
        self.state_msgs=states()
        self.msg={}

    def subscribe_command(self):
        self.lc.subscribe("/joint_command",self.command_callback)

    def publish_states(self):
        self.state_msgs.body_quaternion=(0 ,0 ,0 ,1) # [x y z w]
        self.lc.publish("/robot_states",self.state_msgs.encode())

    def command_callback(self,channel,data):
        self.msg = command.decode(data)


    def update_states(self):
        a=1
    def update_command(self):
        a=1

#test
if __name__=='__main__':
    print("start lcm test ")
    sim=LCMInterface()
    sim.subscribe_command() #订阅
    #
    #test
    sim.cmd_msgs.joint_kd=[1]*12
    sim.cmd_msgs.joint_kp=[1]*12
    sim.cmd_msgs.torque_feedforward=[0]*12
    sim.cmd_msgs.joint_position=[0]*12
    sim.cmd_msgs.joint_velocity=[0]*12
    sim.lc.publish("/joint_command",sim.cmd_msgs.encode())
    while True:
        sart_time=time.time()
        sim.publish_states()
        # sim.lc.handle()     #更新
        sim.lc.handle_timeout(1000)
        
        end_tiem=time.time()
        #test
        print(sim.msg.joint_kp)
