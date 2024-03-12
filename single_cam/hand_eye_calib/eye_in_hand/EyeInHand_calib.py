import cv2 as cv
import numpy as np
from utils import file_utils
import os
import copy
import yaml
from transform_3d import ICP
from threading import Thread
import logging

logger = logging.getLogger(__name__)
logdir = './logfile'
os.makedirs(logdir, exist_ok=True)
logging.basicConfig(filename=os.path.join(logdir, 'error.txt'),
                     format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                     level=logging.INFO)

def task(index, image, pattern_size, chessboard_corners, image_indexes):
    gray_img = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray_img, pattern_size)
    if ret:
        chessboard_corners.append(corners)
        image_indexes.append(index)
    else:
        print("No chessboard found in image: ", index)

def findChessboardCorners(images, pattern_size):
    chessboard_corners = []
    image_indexes = []
    task_list = []
    for index, image in enumerate(images):
        t = Thread(target=task, kwargs={'index':index, 'image':image, 'pattern_size':pattern_size, 'chessboard_corners':chessboard_corners, 'image_indexes':image_indexes})
        task_list.append(t)
    for t in task_list:
        t.start()
    for t in task_list:
        t.join()
    return chessboard_corners, image_indexes

def computeCameraPoses(chessboard_corners, pattern_size, square_size, intrinsic_matrix, distort_coef):
    object_points_3d = np.zeros((pattern_size[0]*pattern_size[1], 3), dtype=np.float32)
    object_points_3d[:,:2]=np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1,2)*square_size

    Rw2c_mat = []
    Tw2c_mat = []
    
    for corners_2d in chessboard_corners:
        _, rvec, tvec = cv.solvePnP(object_points_3d, corners_2d, intrinsic_matrix, distort_coef)
        R_mat, _ = cv.Rodrigues(rvec)
        Rw2c_mat.append(R_mat)
        Tw2c_mat.append(tvec)
    
    return object_points_3d, Rw2c_mat, Tw2c_mat

def eyeInHandCalib(config_file, image_folder, pose_folder, result_folder):
    config_info = file_utils.loadConfig(config_file)
    chessboard_info, camera_info = config_info['chessboard'], config_info['camera']
    cam_imgs = file_utils.loadImages(image_folder)
    arm_poses = file_utils.loadPoses(pose_folder)

    if len(cam_imgs)!=len(arm_poses):
        print('image numer: {}, pose number: {}, not equal!'.format(len(cam_imgs), len(arm_poses)))
        return
        
    detect_folder = os.path.join(result_folder, 'corners')
    # reproject_folder = os.path.join(result_folder, 'reproject')
    extrinsic_folder = os.path.join(result_folder, 'extrinsic')
    os.makedirs(detect_folder, exist_ok=True)
    # os.makedirs(reproject_folder, exist_ok=True)
    os.makedirs(extrinsic_folder, exist_ok=True)
    
    pattern_size = (chessboard_info['num_x'], chessboard_info['num_y'])
    chessboard_corners, image_indexes = findChessboardCorners(cam_imgs, pattern_size)
    
    if len(image_indexes)<2:
        print('chessboard detection all failed!')
        return
    
    for nn in range(len(image_indexes)):
        tmp_img = copy.deepcopy(cam_imgs[image_indexes[nn]])
        cv.drawChessboardCorners(tmp_img, pattern_size, chessboard_corners[nn], True)
        cv.imwrite(os.path.join(detect_folder, '{}.jpg'.format(image_indexes[nn])), tmp_img)
    
    arm_poses_select = [arm_poses[i] for i in image_indexes]

    intrinsic_mat = np.zeros((3,3),dtype=np.float32)
    intrinsic_mat[0,0] = camera_info['intrinsic']['fx']
    intrinsic_mat[1,1] = camera_info['intrinsic']['fy']
    intrinsic_mat[0,2] = camera_info['intrinsic']['cx']
    intrinsic_mat[1,2] = camera_info['intrinsic']['cy']
    intrinsic_mat[2,2] = 1
    
    distort_coef = np.zeros((5,1), dtype=np.float32)
    distort_coef[0, 0] = camera_info['distort']['k1']
    distort_coef[1, 0] = camera_info['distort']['k2']
    distort_coef[4, 0] = camera_info['distort']['k3']
    
    square_size = (chessboard_info['size_x'], chessboard_info['size_y'])
    
    object_points_3d, Rw2c_mat_list, Tw2c_mat_list = computeCameraPoses(chessboard_corners, 
                                                      pattern_size, 
                                                      square_size, 
                                                      intrinsic_mat, 
                                                      distort_coef)
    
    Pw2c_mat_list = [np.concatenate((R,T), axis=1) for R,T in zip(Rw2c_mat_list, Tw2c_mat_list)]
    Pw2c_mat_list = [np.concatenate((M, np.array([[0,0,0,1]])), axis=0) for M in Pw2c_mat_list]

    # Rc2w_mat_list = [T[:3,:3] for T in Pc2w_mat_list]
    # Tc2w_mat_list = [T[:3, 3] for T in Pc2w_mat_list]
    
    Re2b_mat_list = [T[:3,:3] for T in arm_poses_select]
    Te2b_mat_list = [T[:3,3] for T in arm_poses_select]
    
    Pe2b_mat_list = arm_poses_select

    for i in range(0,5):
        R_c2e_mat, T_c2e_mat = cv.calibrateHandEye(Re2b_mat_list, 
                                                   Te2b_mat_list, 
                                                   Rw2c_mat_list, 
                                                   Tw2c_mat_list, 
                                                   method=i)
        
        params_dict = {}
        params_dict['R'] = R_c2e_mat.tolist()
        params_dict['T'] = T_c2e_mat.tolist()
        filename = os.path.join(extrinsic_folder, "calc_{}.yml".format(i))
        with open(filename, 'w', encoding='utf-8') as f:
            yaml.dump(stream=f, data=params_dict)

    corner_error(chessboard_corners, object_points_3d, Pw2c_mat_list, Pe2b_mat_list, intrinsic_mat, distort_coef, result_folder)

def transformation_error(Pw2c_mat_list, Pe2b_mat_list, result_folder):

    Pw2c_mat_inv_list = [np.linalg.inv(T) for T in Pw2c_mat_list]
    Pe2b_mat_inv_list = [np.linalg.inv(T) for T in Pe2b_mat_list]

    Pc2e_mat_list = extrinsic_param(result_folder)

    assert(len(Pw2c_mat_list) == len(Pe2b_mat_list))
    for i in range(0, 5):
        Ern_avg = 0
        Pc2e_mat_list = extrinsic_param(result_folder)
        n= len(Pw2c_mat_list)
        for index in range(1, n):
            Eqax = np.dot(np.dot(Pe2b_mat_inv_list[index], Pe2b_mat_list[index - 1]), Pc2e_mat_list[i])
            Eqbx = np.dot(Pc2e_mat_list[i], np.dot(Pw2c_mat_list[index], Pw2c_mat_inv_list[index - 1]))
            Ern = np.linalg.norm((Eqax - Eqbx), ord=2)
            Ern_avg = Ern_avg + Ern
            print(Ern)
        print(Ern_avg / (n - 1))
        
def corner_error(chessboard_corners, object_points_3d, Pw2c_mat_list, Pe2b_mat_list, intrinsic_mat, distort_coef, result_folder):
    Pc2e_mat_list = extrinsic_param(result_folder)
    n = len(Pe2b_mat_list)
    m = len(object_points_3d)
    print(len(chessboard_corners))
    print(m)

    Pb2e_mat_list = [np.linalg.inv(T) for T in Pe2b_mat_list]
    Pe2c_mat_list = [np.linalg.inv(T) for T in Pc2e_mat_list]
    for i in range(0, 5): # 每个方法对应一个Pc2e
        total_err = 0
        Pw2b_mat_list = Pw2b_function(Pw2c_mat_list, Pc2e_mat_list[i], Pe2b_mat_list)
        for index in range(n):
            averge_err = 0
            real_corners = chessboard_corners[index]
            for corner_index in range(m):
                temp = np.dot(Pe2c_mat_list[i], np.dot(Pb2e_mat_list[index], Pw2b_mat_list[index]))
                compute_corner, _ = cv.projectPoints(object_points_3d[corner_index], temp[:3, :3], temp[:3, 3], intrinsic_mat, distort_coef)
                corner_err = np.linalg.norm((real_corners[corner_index] - compute_corner))
                averge_err += corner_err
            averge_err /= m
            print('image{0}error:{1}'.format(index, averge_err))
            logger.info('image{0}error:{1}'.format(index, averge_err))
            total_err += averge_err
        print('calc{0}_error:{1}'.format(i, total_err / n))
        logger.info('calc{0}_error:{1}'.format(i, total_err / n))

def Pw2b_function(Pw2c_mat_list, Pc2e, Pe2b_mat_list):
    assert(len(Pe2b_mat_list) == len(Pw2c_mat_list))
    n = len(Pe2b_mat_list)
    Pw2b_mat_list = []
    for index in range(n):
        Pw2b = np.dot(Pe2b_mat_list[index],(np.dot(Pc2e, Pw2c_mat_list[index])))
        Pw2b_mat_list.append(Pw2b)
    return Pw2b_mat_list

def extrinsic_param(result_folder):
    extrinsic_folder = os.path.join(result_folder, 'extrinsic')
    Pc2e_mat_list = []
    for i in range(0, 5):
        filename = os.path.join(extrinsic_folder, "calc_{}.yml".format(i))
        with open(filename, 'r', encoding='utf-8') as f:
            dic = yaml.load(f, Loader=yaml.FullLoader)
        R = np.array(dic['R'])
        T = np.array(dic['T'])
        Pc2e_mat = np.concatenate((np.array(R),np.array(T)), axis=1)
        Pc2e_mat = np.concatenate((Pc2e_mat, np.array([[0,0,0,1]])), axis=0)
        Pc2e_mat_list.append(Pc2e_mat)
    return Pc2e_mat_list

def eyeInHandCalib_byline(config_file, points_folder, pose_folder, result_folder):
    config_info = file_utils.loadConfig(config_file)
    arm_poses = file_utils.loadPoses(pose_folder)
    pts_list = file_utils.loadPoints(points_folder)
    
    assert(len(pts_list)==2)
    
    Rw2c_1, Tw2c_1 =  ICP.icp_match(config_info['view1'], pts_list[0])
    Rw2c_2, Tw2c_2 =  ICP.icp_match(config_info['view2'], pts_list[1])
    
    print(Rw2c_1)
    print(Tw2c_1)
    
    print(Rw2c_2)
    print(Tw2c_2)
    
    Rw2c_mat_list = [Rw2c_1, Rw2c_2]
    Tw2c_mat_list = [Tw2c_1, Tw2c_2]
    
    Re2b_mat_list = [T[:3,:3] for T in arm_poses]
    Te2b_mat_list = [T[:3,3] for T in arm_poses]
    
    extrinsic_folder = os.path.join(result_folder, "extrinsic")
    os.makedirs(extrinsic_folder, exist_ok=True)
    
    for i in range(0,5):
        R_c2e_mat, T_c2e_mat = cv.calibrateHandEye(Re2b_mat_list, 
                                                   Te2b_mat_list, 
                                                   Rw2c_mat_list, 
                                                   Tw2c_mat_list, 
                                                   method=i)
        
        params_dict = {}
        params_dict['R'] = R_c2e_mat
        params_dict['T'] = T_c2e_mat
        filename = os.path.join(extrinsic_folder, "calc_{}.yml".format(i))
        with open(filename, 'w', encoding='utf-8') as f:
            yaml.dump(params_dict)