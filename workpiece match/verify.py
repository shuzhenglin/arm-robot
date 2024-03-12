import sys
sys.path.append(r'D:\Code\Line_scan\robot\Dlls')
from robot.aubo_controller import *
import numpy as np
import zmq
import subprocess
import os
from plyfile import PlyData, PlyElement

# zmq
zmq_IP = '192.168.1.234'
zmq_PORT = 5555
# robot
Robot_IP = '192.168.1.10'
Robot_PORT = 8899

def dic_concat(dic1, dic2):
    return {**dic1, **dic2}

def zmq_connect():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect('tcp://{0}:{1}'.format(zmq_IP, zmq_PORT))
    return socket

def grab_ply(rob, ply_folder, pos_folder, facet_name, socket): # facet_name = 'facet1'
    pose_file = os.path.join(pos_folder, facet_name + '.yml')
    if os.path.exists(pose_file):
        pass
    else:
        pose_file = open(pose_file, "w")
        pose_file.close()
    count = 0
    pose_dic = {}
    while count < 10: #! 扫描次数
        facet = facet_name + '_{}.ply'.format(count)#{:0>4d}
        ply_file = os.path.join(ply_folder, facet)
        msg = send_flag(socket, ply_file) # 发送拍摄指令
        if msg == b'ok':
            print(msg)
            pose_name = facet_name +'_{}'.format(count)
            rec_pos = rob.record_robot_pose()
            pose_dic[pose_name] = rec_pos # 记录当前机械臂位姿
            if os.path.getsize(pose_file): 
                yml_data = load_yaml(f=pose_file)
                write_yaml(f=pose_file,data=dic_concat(yml_data, pose_dic))
            else:
                write_yaml(f=pose_file, data=pose_dic)
        # 移动
        cur_pos = rob.get_robot_pose()
        y = 0.005 # ! 确定一下移动的方向
        rob.move_in_flange(pose=cur_pos, y_move=y) # 前扫
        count += 1
    return pose_file

def move2target(rob, T122_mat):
    cur_pos = load_yaml(r'D:\Code\Line_scan\Data\result\armpose\temp_pose.yml')
    rob.set_cart(cur_pos['joint'])

    pose = rob.get_robot_pose()
    img_folder = r'D:\Code\Line_scan\Data\Images'
    exe_folder = r'D:\Code\Line_scan\exe'
    image_name = 'image_intrim.png'
    img_file = os.path.join(img_folder, image_name)
    camera_2d = os.path.join(exe_folder, 'test_camera ')
    p = subprocess.Popen(camera_2d + img_file, shell=True)
    p.wait()

    armpose_file = r'D:\Code\Line_scan\Data\result\armpose\target_pose.yml'
    write_yaml(f=armpose_file, data=pose)

    R = quaternion2rotation(pose['ori'])
    T = np.reshape(np.array(pose['pos']), (3, 1))
    Te2b_mat_1 = np.vstack((np.hstack((R, T)), np.array([0, 0, 0, 1])))
    Te2b_mat_2 = np.dot(T122_mat, Te2b_mat_1)
    next_pose = {}
    next_pose['pos'] = np.reshape(Te2b_mat_2[:3, 3], (1, 3)).tolist()[0]
    next_pose['ori'] = rotation2quaternion(Te2b_mat_2[:3, :3])
    joint_move = rob.inverse(pos=next_pose['pos'], ori=next_pose['ori'])
    rob.set_end_speed(vel=0.1, acc=0.1)
    rob.set_cart(joint_move)

def send_flag(socket, flag):
    socket.send_string(flag)
    msg = socket.recv()
    return msg

def read_ply(ply_name): 
    plydata = PlyData.read(ply_name)
    points = plydata['vertex'].data.copy()
    cloud = np.zeros([len(points), 3])
    for i in range(len(points)):
        point = points[i]
        cloud[i] = np.array([point[0], point[1], point[2]])
    return cloud

def write_ply(save_path,points,text=True):
    """
    save_path : path to save: '/yy/XX.ply'
    pt: point_cloud: size (N,3)
    """
    points = [(points[i,0], points[i,1], points[i,2]) for i in range(points.shape[0])]
    vertex = np.array(points, dtype=[('x', 'f4'), ('y', 'f4'),('z', 'f4')])
    el = PlyElement.describe(vertex, 'vertex', comments=['vertices'])
    PlyData([el], text=text).write(save_path)

def read_calc(filename):
    dic = load_yaml(filename)
    R = np.array(dic['R'])
    T = np.array(dic['T'])
    return [R, T]

def transform(endpos, RT_s, cloud_pts):
    res = None
    R0 = np.array(quaternion2rotation(endpos['ori']))
    T0 = np.reshape(np.array(endpos['pos']), (3, 1))
    RT_0 = matrix_concat(R=R0, T=T0)
    for cloud in cloud_pts:
        cloud = np.array(cloud)
        if not np.allclose(cloud, np.array([0., 0., 0.])) and (cloud[2] > 130.):
            cloud = np.hstack((cloud, np.array([1.])))
            cloud.transpose()
            ans = np.dot(RT_0, np.dot(RT_s, cloud))
            ans = np.reshape(ans[:3], (1,3))
            if res is None:
                res = ans
            else:
                res = np.concatenate((res, ans))
    return res


def concate_ply(ply_folder, facet_name, ply_res_folder, calc_file, pose_file): # 每次传入一个面的ply
    ply_dir = []
    cloud_list = None
    ply_names = os.listdir(ply_folder)
    facet_ply_path = os.path.join(ply_res_folder, facet_name + '.ply')
    calc_RT = read_calc(calc_file)
    R, T = calc_RT
    RT_s = np.vstack((np.hstack((R, T)), np.array([0, 0, 0, 1])))
    endpos_list = load_yaml(pose_file)
    for filename in ply_names:
        ply_dir.append(os.path.join(ply_folder, filename))
    # 遍历target目录下所有ply文件,并将所有文件转换到base
    for index, filename in enumerate(ply_dir):
        points = read_ply(filename)
        pos_name = os.path.basename(filename)
        endpos = pos_name.split('.')[0]
        #endpos = facet_name + '_{:0>4d}.ply'.format(index)
        
        cloud = transform(endpos_list[endpos], RT_s, points)
        if cloud is not None:
            if cloud_list is None:
                cloud_list = cloud
            else:
                cloud_list = np.concatenate((cloud_list, cloud))
    write_ply(save_path=facet_ply_path, points=cloud_list)
    return facet_ply_path

def verify_func(rob, ply_folder, facet_name, pos_folder, calc_file):
    #zmq
    # socket = zmq_connect()

    # grab_ply(rob, ply_folder, pos_folder, facet_name, socket)
    pose_file = r'D:\Code\Line_scan\Data\armpose\facet_target.yml'
    facet_ply_path = concate_ply(ply_folder, facet_name, ply_res_folder, calc_file, pose_file) # 拼接点云面
    # facet_ply_path =r'D:\Code\Line_scan\Data\result\plydata\target\facet_target.ply'
    # *调用点云匹配
    T122_file = r'D:\Code\Line_scan\Data\result\armpose\T122.yml'
    p = subprocess.Popen(args=r'D:\Code\Dot_Match\scan_algo\test_registration.exe ' + r'D:\Code\Line_scan\Data\result\plydata\template\facet_temp.ply' + ' ' + facet_ply_path + ' ' + T122_file, shell=True)
    p.wait()
    
    T122_list = load_yaml(T122_file) # 1到2位置变换RT

    matrix= np.mat(T122_list['matrix'])
    T122_R = matrix[:3, :3]
    T122_T = matrix[:3, 3] / 1000
    T122_mat = np.vstack((np.hstack((T122_R, T122_T)), np.array([0, 0, 0, 1])))
    
    move2target(rob, T122_mat)


if __name__ == '__main__':
    rob = AuboRobot()
    current_dir = os.path.dirname(os.path.abspath(__file__))
    Data_folder = os.path.join(current_dir, 'Data')
    calc_folder = os.path.join(current_dir, r'Data\calc_data')
    exe_folder = os.path.join(current_dir, 'exe')

    # ply采集路径
    ply_folder = os.path.join(Data_folder, r'plydata\target')

    # ply储存路径
    ply_res_folder = os.path.join(Data_folder, r'result\plydata\target')
    pos_folder = os.path.join(Data_folder, 'armpose')
    img_folder = os.path.join(Data_folder, 'Images')
    calc_file = os.path.join(calc_folder, 'T4.yml')

    os.makedirs(ply_folder, exist_ok=True)
    os.makedirs(pos_folder, exist_ok=True)
    os.makedirs(ply_res_folder, exist_ok=True)

    
    
    if rob.connect_rob(Robot_IP, Robot_PORT) == 0:
        facet_name = 'facet_target'
        # 验证
        verify_func(rob, ply_folder, facet_name, pos_folder, calc_file)

        # 2D
        image_name = 'image_target.png'
        img_file = os.path.join(img_folder, image_name)
        camera_2d = os.path.join(exe_folder, 'test_camera ')
        p = subprocess.Popen(camera_2d + img_file, shell=True)

        rob.disconnect_rob()
    rob.robot_release()