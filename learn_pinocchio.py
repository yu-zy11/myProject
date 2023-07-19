from os.path import join
import pinocchio as se3
from pinocchio.romeo_wrapper import RomeoWrapper

PKG = '/opt/openrobots/share'
URDF = join(PKG, 'romeo_description/urdf/romeo.urdf')

robot = RomeoWrapper(URDF, [PKG])  # Load urdf model
robot.initDisplay(loadModel=True)
