import glob
import yaml
import cv2 as cv
import numpy as np
from scipy.spatial.transform import Rotation

def quaternion2rotation(ori):
    Rq = np.array(ori)
    Rm = Rotation.from_quat(Rq)
    rotation_matrix = Rm.as_matrix()
    return rotation_matrix

def loadImages(folder):
    image_files = sorted(glob.glob('{}/*.[jp][pn]g'.format(folder)), reverse=False)
    cam_images = [cv.imread(file, cv.IMREAD_COLOR) for file in image_files]
    return cam_images

def loadPoses(folder):
    yml_files = sorted(glob.glob('{}/*.yml'.format(folder)), reverse=False)
    assert(len(yml_files)==1)
    with open(yml_files[0], 'r', encoding='utf-8') as f:
        content = f.read()
        arm_poses = yaml.load(content, Loader=yaml.FullLoader)
        for i, arm_pose in enumerate(arm_poses):
            T = np.reshape(np.array(arm_pose[:3]), (3, 1))
            R = quaternion2rotation(arm_pose[3:])
            arm_poses[i] = np.vstack((np.hstack((R, T)), np.array([0, 0, 0, 1])))
        return arm_poses

def loadConfig(file):
    with open(file, 'r', encoding='utf-8') as f:
        content = f.read()
        config_info= yaml.load(content, Loader=yaml.FullLoader)
        return config_info
        # return config_info['chessboard'], config_info['camera']

def loadPoints(folder):
    pts_files = sorted(glob.glob('{}/*.yml'.format(folder)), reverse=False)
    # print(pts_files)
    pts_list = []
    for file in pts_files:
        with open(file, 'r', encoding='utf-8') as f:
            content = f.read()
            config_info= yaml.load(content, Loader=yaml.FullLoader)
            pts_list.append(config_info['points'])
    return pts_list

if __name__ == '__main__':
    # folder = r'../data_lg/poses/'
    # poses = loadPoses(folder)
    # for x in poses:
    #     print(x)
        
    # folder = r'../data_lg/laser/'
    # images = loadPoints(folder)
    # print(len(images))
    
    # file = r'../config/conf_lg.yml'
    # config_info = loadConfig(file)
    # cube_conf, view1_conf, view2_conf = config_info['cube'], config_info['view1'], config_info['view2']
    # print(cube_conf)
    # print(view1_conf)
    # print(view2_conf)

    print(loadPoses('./data_cg/poses/'))