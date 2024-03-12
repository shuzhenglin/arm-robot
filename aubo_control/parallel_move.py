from aubo_controller import *
import argparse
import subprocess

def dic_concat(dic1, dic2):
    return {**dic1, **dic2}

def flange_move(rob):
    parser = argparse.ArgumentParser()
    parser.add_argument('-x', type=float, default=0., help='give a move distance at x axis')
    parser.add_argument('-y', type=float, default=0., help='give a move distance at y axis')
    parser.add_argument('-z', type=float, default=0., help='give a move distance at z axis')
    parser.add_argument('-anglex', type=float, default=0., help='give a rotation angle at x axis')
    parser.add_argument('-angley', type=float, default=0., help='give a rotation angle at y axis')
    parser.add_argument('-anglez', type=float, default=0., help='give a rotation angle at x axis')
    parser.add_argument('--camera', type=bool, default=False, help='use camera grab a line')
    args = parser.parse_args()
    x = args.x
    y = args.y
    z = args.z
    anglex = args.anglex
    angley = args.angley
    anglez = args.anglez
    cur_pos = rob.get_robot_pose()
    rob.move_in_flange(pose=cur_pos, anglex=anglex, angley=angley, anglez=anglez, x_move=x, y_move=y, z_move=z)


if __name__ == '__main__':
    IP = '192.168.1.10'
    PORT = 8899
    rob = AuboRobot()
    if rob.connect_rob(IP, PORT) == 0:
        flange_move(rob=rob)
        rob.disconnect_rob()
    rob.robot_release()