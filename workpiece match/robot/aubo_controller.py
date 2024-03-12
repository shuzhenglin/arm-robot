import sys
sys.path.append(r'D:\Code\Line_scan\Scanner\robot')
sys.path.append(r'D:\Line_scan\Scanner\robot\Dlls')

import robotcontrol as aubo
from robotcontrol import *
import numpy as np
import logging
import yaml
import math
from scipy.spatial.transform import Rotation

logger = logging.getLogger('main.robotcontrol')

class AuboError(Exception):
    pass

class AuboRobot():
    def __init__(self) -> None:
        logger_init()
        Auboi5Robot.initialize()
        self.robot = Auboi5Robot()
        handle = self.robot.create_context()
        logger.info("robot.rshd={0}".format(handle))

    def stop(self):
        if self.robot:
            self.robot = None

    def robot_start(self):
        self.robot.robot_startup()
        self.robot.set_collision_class(7)
        self.robot.set_end_max_line_acc(end_maxacc=0.2)
        self.robot.set_end_max_line_velc(end_maxvelc=0.05)

    def init_robot(self):
        self.robot.init_profile()

    def set_joint_param(self):
        self.robot.set_joint_maxacc((0.1, 0.1, 0.1, 0.1, 0.1, 0.1))
        self.robot.set_joint_maxvelc((0.3, 0.3, 0.3, 0.3, 0.3, 0.3))

    def get_robot_pose(self):
        pose = self.robot.get_current_waypoint()
        return pose
    def record_robot_pose(self):
        pose = self.robot.get_current_waypoint()
        pose['pos'][0] = pose['pos'][0] * 1000
        pose['pos'][1] = pose['pos'][1] * 1000
        pose['pos'][2] = pose['pos'][2] * 1000
        return pose

    def connect_rob(self, ip="192.168.1.10", port=8899):
        try:
            result = self.robot.connect(ip, port)
            if result != RobotErrorType.RobotError_SUCC:
                logger.info("connect server{0}:{1} failed.".format(ip, port))
            else:
                self.init_robot()
                self.robot_start()
                # self.set_end_speed(vel=2, acc=2)
                return 0
        except AuboError as e:  
            raise AuboError("[Aubo-Error]: Robot connect error {}".format(e))
        
    def disconnect_rob(self):
        self.robot.disconnect()

    def forward(self, joint_radian):
        pose = self.robot.forward_kin(joint_radian)
        return pose

    def set_end_speed(self, vel, acc):
        self.robot.set_end_max_line_velc(end_maxvelc=vel)
        self.robot.set_end_max_line_acc(end_maxacc=acc)
        

    def move_sl(self, points=[], acc=0.2, speed=0.05, radius=0.001):
        self.robot.set_end_max_line_acc(end_maxacc=acc)
        self.robot.set_end_max_line_velc(end_maxvelc=speed)
        self.robot.set_blend_radius(blend_radius=radius)
        for point in points:
            res = self.robot.add_waypoint(point)
            if res != 0:
                logger.info("move line add waypoint error")
            else:
                logger.info("add success")
        res = self.robot.move_track(RobotMoveTrackType.CARTESIAN_MOVEP)
        self.robot.remove_all_waypoint()
        self.robot.set_end_max_line_velc(end_maxvelc=speed)

    def get_robot_state(self):
        return self.robot.get_robot_state()

    def robot_release(self):
        if self.robot.connected:
            self.robot.robot_shutdown()
            self.robot.disconnect()
        self.robot.uninitialize()
    
    def joint2cart(self, joint):
        res = self.robot.forward_kin(joint)
        print("res:",res)
        cart = res["pos"]
        cart.extend(res["ori"])
        return cart
    
    def set_joint(self, joint):
        self.robot.move_joint(joint)
    
    def set_cart(self, cart):
        if len(cart) != 6:
            raise AuboError("[Aubo-Error]: Set Cartesian coordinate system, data length error")
        else:
            res = self.robot.move_line(cart)
            if res != RobotErrorType.RobotError_SUCC:
                raise AuboError("[Aubo-Error]: Robot move_line error")

    def inverse(self, pos, ori):
        joint_move = self.robot.inverse_kin(pos=pos,ori=ori)
        return joint_move['joint']

    def x_move(self, joint_radian, distance=0.01):
        temp = self.robot.forward_kin(joint_radian)
        temp['pos'][0] += distance
        joint_move = self.robot.inverse_kin(pos=temp['pos'], ori=temp['ori'])
        return joint_move

    def y_move(self, joint_radian, distance=0.01):

        temp = self.robot.forward_kin(joint_radian)
        temp['pos'][1] += distance
        joint_move = self.robot.inverse_kin(pos=temp['pos'], ori=temp['ori'])
        return joint_move
    
    def xyz_move(self, joint_radian, xdis=0.01, ydis=0.005, zdis=0.01):
        temp = self.robot.forward_kin(joint_radian)
        temp['pos'][0] += xdis
        temp['pos'][1] += ydis
        temp['pos'][2] += zdis
        joint_move = self.robot.inverse_kin(pos=temp['pos'], ori=temp['ori'])
        return joint_move

    def quaternion2rpy(self, quaternion):
        rpy = self.robot.quaternion_to_rpy(quaternion)
        return rpy
    
    def move_in_flange(self, pose, anglex=0., angley=0., anglez=0., x_move=0., y_move=0., z_move=0.):
        R1 = np.array(quaternion2rotation(pose['ori']))
        T1 = np.reshape(np.array((pose['pos'])), (3,1))
        tf_base_tool1 = matrix_concat(R=R1, T=T1)
        #R2 = np.identity(3, dtype=R1.dtype)
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(anglex), -math.sin(anglex)],
                       [0, math.sin(anglex), math.cos(anglex)]])
        Ry = np.array([[math.cos(angley), 0, math.sin(angley)],
                      [0, 1, 0],
                      [-math.sin(angley), 0, math.cos(angley)]])
        Rz = np.array([[math.cos(anglez), -math.sin(anglez), 0],
                      [math.sin(anglez), math.cos(angley), 0],
                      [0, 0, 1]])
        R2 = np.dot(np.dot(Rx, Ry), Rz)
        T2 = np.reshape(np.array((x_move, y_move, z_move)), (3, 1))
        tf_tool1_tool2 = matrix_concat(R=R2, T=T2)
        tf_base_tool2 = np.dot(tf_base_tool1, tf_tool1_tool2)
        R3 = tf_base_tool2[:3, :3]
        T3 = tf_base_tool2[:3, 3:4]
        new_pose = {}
        new_pose['ori'] = rotation2quaternion(R3)
        new_pose['pos'] = T3.reshape(1, 3).tolist()
        new_pose['pos'] = new_pose['pos'][0]
        joint_move = self.robot.inverse_kin(pos=new_pose['pos'], ori=new_pose['ori'])
        self.set_cart(cart=joint_move['joint'])


def matrix_concat(R, T):
    temp = np.concatenate((R, T),axis=1)
    RT = np.vstack((temp, np.array([0, 0, 0, 1])))
    return RT

def quaternion2rotation(ori):
    Rq = np.array((ori[1], ori[2], ori[3], ori[0]))
    Rm = Rotation.from_quat(Rq)
    rotation_matrix = Rm.as_matrix()
    return rotation_matrix

def rotation2quaternion(R):
    Rm = Rotation.from_matrix(R)
    ans = Rm.as_quat()
    quaternion = list((ans[3], ans[0], ans[1], ans[2]))
    return quaternion

def write_yaml(f, data):
    with open(f, 'w', encoding='utf-8') as f:
        yaml.dump(data=data, stream=f, allow_unicode=True, sort_keys=False, default_flow_style=True, width=168)

def add_yaml(f, data):
    with open(f, 'a', encoding='utf-8') as f:
        yaml.dump(data=data, stream=f, allow_unicode=True, sort_keys=False, default_flow_style=True, width=168)

def load_yaml(f):
    with open(f, 'r', encoding='utf-8') as f:
        result = yaml.load(f, Loader=yaml.FullLoader)
    return result

def read_last_line(f):
    with open(f, 'r', encoding='utf-8') as file:
        lines = file.readlines()
        last_line = lines[-1].strip()
    return last_line