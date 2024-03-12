import aubo_controller as aubocl
from aubo_controller import *

if __name__ == '__main__':
    IP = '192.168.1.10'
    PORT = 8899
    file = './test_pos.yml'
    rob = AuboRobot()

    if rob.connect_rob(IP, PORT) == 0:
        joint_rad = [-0.8647310137748718, 0.17900170385837555, 1.4653223752975464, -0.29312679171562195, 1.5710787773132324, 1.8066211938858032]
        rob.set_joint(joint=joint_rad)
        rob.disconnect_rob()
    rob.robot_release()