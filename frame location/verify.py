from robot.aubo_controller import *
import numpy as np
import zmq
import subprocess
import os
from plyfile import PlyData, PlyElement
from threading import Thread

logfile = r'logfiles\verify.log'
loggerv = logging.getLogger(__name__)
rh = logging.handlers.TimedRotatingFileHandler(logfile, 'D')
logging.basicConfig(filename=logfile, level = logging.INFO, format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger.addHandler(rh)

# zmq
zmq_IP = '192.168.1.234'
zmq_PORT = 5555
# robot
Robot_IP = '192.168.1.10'
Robot_PORT = 8899

def zmq_connect():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect('tcp://{0}:{1}'.format(zmq_IP, zmq_PORT))
    return socket

def send_flag(socket, flag):
    socket.send_string(flag)
    msg = socket.recv()
    return msg

def dic_concat(dic1, dic2):
    return {**dic1, **dic2}

def create_yamlfile(yamlfile):
    if os.path.exists(yamlfile):
        pass
    else:
        yamlfile = open(yamlfile, "w")
        yamlfile.close()

def write_ply(save_path,points,text=True):
    """
    save_path : path to save: '/yy/XX.ply'
    pt: point_cloud: size (N,3)
    """
    points = [(points[i,0], points[i,1], points[i,2]) for i in range(points.shape[0])]
    vertex = np.array(points, dtype=[('x', 'f4'), ('y', 'f4'),('z', 'f4')])
    el = PlyElement.describe(vertex, 'vertex', comments=['vertices'])
    PlyData([el], text=text).write(save_path)

def read_ply(ply_name): 
    plydata = PlyData.read(ply_name)
    points = plydata['vertex'].data.copy()
    cloud = np.zeros([len(points), 3])
    i = 0
    p = np.array([-32.768, -32.768, -32.768])
    q = np.array([0., 0. ,0.])
    for point in points:
        point = np.array([point[0], point[1], point[2]])
        if np.allclose(point, q):
            pass
        elif np.allclose(point, p):
            pass
        else:
            cloud[i] = point
            i += 1
    return cloud

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
        cloud = np.hstack((np.array(cloud), np.array([1.])))
        cloud.transpose()
        ans = np.dot(RT_0, np.dot(RT_s, cloud))
        ans = np.reshape(ans[:3], (1,3))
        if res is None:
            res = ans
        else:
            res = np.concatenate((res, ans))
    return res

def concate_ply(ply_folder, facet_name, ply_res_folder, calc_file, pose_file, cpt_file): # 每次传入一个面的ply
    start = time.perf_counter()
    ply_dir = []
    cloud_list = None
    ply_names = os.listdir(ply_folder)
    # ply存储路径
    # facet_dir = os.path.join(ply_res_folder, 'facet')
    # os.makedirs(facet_dir, exist_ok=True)
    facet_ply_path = os.path.join(ply_res_folder, facet_name + '.ply')

    calc_RT = read_calc(calc_file)
    R, T = calc_RT
    RT_s = np.vstack((np.hstack((R, T)), np.array([0, 0, 0, 1])))
    endpos_list = load_yaml(pose_file)
    for filename in ply_names:
        ply_dir.append(os.path.join(ply_folder, filename))
    # 遍历target目录下所有ply文件,并将所有文件转换到base
    for filename in ply_dir:
        points = read_ply(filename)
        pos_name = os.path.basename(filename)
        endpos = pos_name.split('.')[0]
        
        cloud = transform(endpos_list[endpos], RT_s, points)
        if cloud_list is None:
            cloud_list = cloud
        else:
            cloud_list = np.concatenate((cloud_list, cloud))
    write_ply(save_path=facet_ply_path, points=cloud_list)
    # ? 调用进程计算孔的中心点
    find_hole = subprocess.Popen(args=r'D:\Code\Dot_Match\scan_algo\find_hole.exe ' + facet_ply_path + ' ' + cpt_file, stdout=subprocess.PIPE,shell=True)
    print(find_hole.stdout.read())
    end = time.perf_counter()
    loggerv.info('concate spend time:{}'.format(end - start))

def grab_ply(rob, num, hole_id, ply_folder, pose_file, pose_dic, socket):
    i = 0
    while i < num:
        ply_filename = os.path.join(ply_folder, '{0}_{1}.ply'.format(hole_id, i))
        msg = send_flag(socket, ply_filename)
        if msg == b'ok':
            print(msg)
            # 储存位姿
            pose_name = hole_id + '_{}'.format(i)
            rec_pose = rob.record_robot_pose()
            pose_dic[pose_name] = rec_pose
            if not os.path.getsize(pose_file):
                write_yaml(f=pose_file, data=pose_dic)
            else:
                yml_data = load_yaml(f=pose_file)
                write_yaml(f=pose_file, data=dic_concat(yml_data, pose_dic))

            # 前扫
            cur_pos = rob.get_robot_pose()
            y = 0.001 # !确定一下移动的方向
            rob.move_in_flange(pose=cur_pos, y_move=y)
            i += 1
    return 0

def integrate_cpt(cpt_list, cpt_folder):
    cloud = None
    # cpt_res_dir = os.path.join(ply_res_folder, 'integration')
    # os.makedirs(cpt_res_dir, exist_ok=True)
    cpt_res_file = os.path.join(cpt_folder, 'integration_point.ply')
    for filename in cpt_list:
        plydata = PlyData.read(filename)
        point = plydata['vertex'].data.copy()
        point = point.tolist()
        point = np.array(point)
        if cloud is None:
            cloud = point
        else:
            cloud = np.concatenate((cloud, point), axis=0)
    write_ply(save_path=cpt_res_file, points=cloud)
    return cpt_res_file

def processing_temp(rob, ply_folder, ply_res_folder, pose_folder, calc_file, socket): # * ply_folder = './Data/plydata' ply_res_folder = './Data/result/temp'
    count = 1
    cpt_list = []
    cpt_folder = os.path.join(ply_res_folder, 'cpt')
    os.makedirs(cpt_folder, exist_ok=True)
    armpose_file =  r'.\Data\result\armpose\temp_pose.yml'

    thread_list = []
    while count <= 4:
        pose_dic = {}
        hole_id = 'hole{}'.format(count)
        
        pose_file = os.path.join(pose_folder, hole_id + '.yml')
        create_yamlfile(pose_file)

        ply_file = os.path.join(ply_folder, hole_id)
        os.makedirs(ply_file, exist_ok=True)
        
        num = 38 # ? 移动次数
        # ! 四个圆孔位置不同，每次前扫需要预先调整位置
        start = time.perf_counter()
        ret = grab_ply(rob, num, hole_id, ply_file, pose_file, pose_dic, socket)
        end = time.perf_counter()
        loggerv.info('grab spend time:{}'.format(end - start))

        cpt_file = os.path.join(cpt_folder, 'center_point{}.ply'.format(count))
        cpt_list.append(cpt_file)
        # TODO concate 子线程
        t = Thread(target=concate_ply, kwargs={'ply_folder':ply_file, 'facet_name':hole_id, 'ply_res_folder':ply_res_folder,
                                               'calc_file':calc_file, 'pose_file':pose_file, 'cpt_file':cpt_file})
        t.start()
        thread_list.append(t)

        if ret == 0:
            count += 1
        # * 孔间移动
        if count == 2:
            cur_pos = rob.get_robot_pose()
            y = 0.397 # !确定一下移动的方向
            rob.move_in_flange(pose=cur_pos, y_move=y)
        elif count == 3:
            cur_pos = rob.get_robot_pose()
            y = -0.473 # !确定一下移动的方向
            x = -0.25 # !确定一下移动的方向
            rob.move_in_flange(pose=cur_pos, y_move=y, x_move=x)
        elif count == 4:
            cur_pos = rob.get_robot_pose()
            y = 0.397 # !确定一下移动的方向
            rob.move_in_flange(pose=cur_pos, y_move=y)
        else:
            cur_pos = rob.get_robot_pose()
            y = -0.236 # !确定一下移动的方向
            x = 0.125
            rob.move_in_flange(pose=cur_pos, y_move=y, x_move=x)
            photograph_pose = rob.get_robot_pose()
            write_yaml(f=armpose_file, data=photograph_pose)
            # * 拍照
            image_name = 'chess.png'
            img_file = os.path.join(ply_res_folder, image_name)
            exe_folder = r'D:\Code\Dot_Match\exe'
            camera_2d = os.path.join(exe_folder, 'test_camera.exe ')
            p = subprocess.Popen(camera_2d + img_file, shell=True)
            p.communicate()
            # * 回位
            cur_pos = rob.get_robot_pose()
            y = -0.237 # !确定一下移动的方向
            x = 0.125
            rob.move_in_flange(pose=cur_pos, y_move=y, x_move=x)
            break
    for t in thread_list:
        t.join()
    cpt_res_file = integrate_cpt(cpt_list, cpt_folder)

def processing_target(rob, ply_folder, ply_res_folder, pose_folder, calc_file, socket): # * ply_folder = './Data/plydata' ply_res_folder = './Data/result/target'
    count = 1
    cpt_list = []
    cpt_folder = os.path.join(ply_res_folder, 'cpt')
    os.makedirs(cpt_folder, exist_ok=True)
    thread_list = []
    while count <= 4:
        pose_dic = {}
        hole_id = 'hole{}'.format(count)
        
        pose_file = os.path.join(pose_folder, hole_id + '.yml')
        create_yamlfile(pose_file)

        ply_file = os.path.join(ply_folder, hole_id)
        os.makedirs(ply_file, exist_ok=True)
        
        num = 38 # ? 移动次数
        # ! 四个圆孔位置不同，每次前扫需要预先调整位置
        start = time.perf_counter()
        ret = grab_ply(rob, num, hole_id, ply_file, pose_file, pose_dic, socket)
        end = time.perf_counter()
        loggerv.info('grab spend time:{}'.format(end - start))

        cpt_file = os.path.join(cpt_folder, 'center_point{}.ply'.format(count))
        cpt_list.append(cpt_file)
        # TODO concate 子线程
        t = Thread(target=concate_ply, kwargs={'ply_folder':ply_file, 'facet_name':hole_id, 'ply_res_folder':ply_res_folder,
                                               'calc_file':calc_file, 'pose_file':pose_file, 'cpt_file':cpt_file})
        t.start()
        thread_list.append(t)

        if ret == 0:
            count += 1
        # * 孔间移动
        if count == 2:
            cur_pos = rob.get_robot_pose()
            y = 0.397 # !确定一下移动的方向
            rob.move_in_flange(pose=cur_pos, y_move=y)
        elif count == 3:
            cur_pos = rob.get_robot_pose()
            y = -0.473 # !确定一下移动的方向
            x = -0.25 # !确定一下移动的方向
            rob.move_in_flange(pose=cur_pos, y_move=y, x_move=x)
        elif count == 4:
            cur_pos = rob.get_robot_pose()
            y = 0.397 # !确定一下移动的方向
            rob.move_in_flange(pose=cur_pos, y_move=y)
        else:
            cur_pos = rob.get_robot_pose()
            y = -0.236 # !确定一下移动的方向
            x = 0.125
            rob.move_in_flange(pose=cur_pos, y_move=y, x_move=x)
            #* 拍照
            image_name = 'interim.png'
            img_file = os.path.join(ply_res_folder, image_name)
            exe_folder = r'D:\Code\Dot_Match\exe'
            camera_2d = os.path.join(exe_folder, 'test_camera.exe ')
            p = subprocess.Popen(camera_2d + img_file, shell=True)
            p.communicate()
            break
    for t in thread_list:
        t.join()
    cpt_res_file = integrate_cpt(cpt_list, cpt_folder)

def move2target(rob, T122_mat):
    # T122_mat = np.linalg.inv(T122_mat)
    armpose_file = r'.\Data\result\armpose\target_pose.yml'
    pose = rob.get_robot_pose()
    write_yaml(f=armpose_file, data=pose)

    R = quaternion2rotation(pose['ori'])
    T = np.reshape(np.array(pose['pos']), (3, 1))
    Te2b_mat_1 = np.vstack((np.hstack((R, T)), np.array([0, 0, 0, 1])))
    Te2b_mat_2 = np.dot(T122_mat, Te2b_mat_1)
    # print(Te2b_mat_2)
    next_pose = {}
    next_pose['pos'] = np.reshape(Te2b_mat_2[:3, 3], (1, 3)).tolist()[0]
    next_pose['ori'] = rotation2quaternion(Te2b_mat_2[:3, :3])
    # print(next_pose)
    # print(pose['pos'])
    joint_move = rob.inverse(pos=next_pose['pos'], ori=next_pose['ori'])
    # print(joint_move)
    rob.set_cart(joint_move['joint'])

def verify_func(rob, ply_res_folder, temp_cpt, target_cpt, T122_file):
    # ? 调用进程
    test_icp = subprocess.Popen(args=r'D:\Code\Dot_Match\scan_algo\test_icp.exe ' + temp_cpt + ' ' + target_cpt + ' ' + T122_file, shell=True)
    test_icp.communicate()

    T122_list = load_yaml(T122_file)
    matrix= np.mat(T122_list['matrix'])
    T122_R = matrix[:3, :3]
    T122_T = matrix[:3, 3] / 1000
    T122_mat = np.vstack((np.hstack((T122_R, T122_T)), np.array([0, 0, 0, 1])))

    move2target(rob, T122_mat)

    image_name = 'chess.png'
    img_file = os.path.join(ply_res_folder, image_name)
    exe_folder = r'D:\Code\Dot_Match\exe'
    camera_2d = os.path.join(exe_folder, 'test_camera.exe ')
    p = subprocess.Popen(camera_2d + img_file, shell=True)
    p.communicate()

    cur_pos = rob.get_robot_pose()
    y = -0.237 # !确定一下移动的方向
    x = 0.125
    rob.move_in_flange(pose=cur_pos, y_move=y, x_move=x)

# if __name__  == '__main__':
#     rob = AuboRobot()
#     current_dir = os.path.dirname(os.path.abspath(__file__))
#     temp_cpt = os.path.join(current_dir, r'Data\result\temp\cpt\integration_point.ply')
#     target_cpt = os.path.join(current_dir, r'Data\result\target\cpt\integration_point.ply')
#     T122_file = os.path.join(current_dir, r'Data\result\armpose\T122.yml')
#     if rob.connect_rob(Robot_IP, Robot_PORT) == 0:
#         verify_func(rob, temp_cpt, target_cpt, T122_file)
#         rob.disconnect_rob()
#     rob.robot_release()