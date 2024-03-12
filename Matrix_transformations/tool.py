import numpy as np
import yaml
import os
from plyfile import PlyData, PlyElement
import argparse
from scipy.spatial.transform import Rotation
import time

parser = argparse.ArgumentParser(description='Pending files')
parser.add_argument('-file', type=str, help='give a Pending file name')
args = parser.parse_args()

base_path = os.path.dirname(os.path.join(__file__))

def load_yaml(filename):
    with open(filename, 'r', encoding='utf-8') as f:
        result = yaml.load(f, Loader=yaml.FullLoader)
    return result

def write_yaml(f, data):
    with open(f, 'w', encoding='utf-8') as f:
        yaml.dump(data=data, stream=f, allow_unicode=True, sort_keys=False, default_flow_style=True, width=168)

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
    q = np.array([0., 0. ,0.])
    for point in points:
        point = np.array([point[0], point[1], point[2]])
        if np.allclose(point, q):
            pass
        elif np.isnan(point).any(axis=0):
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

def quaternion2rotation(ori):
    Rq = np.array((ori[1], ori[2], ori[3], ori[0]))
    Rm = Rotation.from_quat(Rq)
    rotation_matrix = Rm.as_matrix()
    return rotation_matrix

def matrix_concat(R, T):
    temp = np.concatenate((R, T),axis=1)
    RT = np.vstack((temp, np.array([0, 0, 0, 1])))
    return RT

def transform(endpos, RT_s, cloud_pts):
    res = None
    R0 = np.array(quaternion2rotation(endpos['ori']))
    T0 = np.reshape(np.array(endpos['pos']), (3, 1))
    RT_0 = matrix_concat(R=R0, T=T0)
    for cloud in cloud_pts:
        cloud = np.array(cloud)
        # if not np.allclose(cloud, np.array([0., 0., 0.])) and cloud[2] > 100.:
        cloud = np.hstack((np.array(cloud), np.array([1.])))
        cloud.transpose()
        # import pdb;
        # pdb.set_trace()
        ans = np.dot(RT_0, np.dot(RT_s, cloud))
        ans = np.reshape(ans[:3], (1,3))
        if res is None:
            res = ans
        else:
            res = np.concatenate((res, ans))
    return res

def concate_ply(ply_folder, facet_name, ply_res_folder, calc_file, pose_file): # 每次传入一个面的ply
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
    for filename in ply_names.sort():
        ply_dir.append(os.path.join(ply_folder, filename))
    # 遍历target目录下所有ply文件,并将所有文件转换到base
    for j, filename in enumerate(ply_dir):
        points = read_ply(filename)
        endpos = 'ball_{:0>4d}'.format(j + 1)
        print(endpos)
        cloud = transform(endpos_list[endpos], RT_s, points)
        if cloud_list is None:
            cloud_list = cloud
        else:
            cloud_list = np.concatenate((cloud_list, cloud))
    write_ply(save_path=facet_ply_path, points=cloud_list)
    end = time.perf_counter()
    print("spend time:", end - start)