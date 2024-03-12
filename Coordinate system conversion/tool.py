import numpy as np
import yaml
import os
from plyfile import PlyData, PlyElement
import pandas as pd
from scipy.spatial.transform import Rotation

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

def transform(filename, endpos, RT_s, cloud_pts):
    res = None
    R0 = np.array(quaternion2rotation(endpos['ori']))
    T0 = np.reshape(np.array(endpos['pos']), (3, 1))
    RT_0 = matrix_concat(R=R0, T=T0)

    for cloud in cloud_pts:
        cloud = np.array(cloud)
        if not np.allclose(cloud, np.array([0., 0., 0.])) and cloud[2] > 100.:
            cloud = np.hstack((np.array(cloud), np.array([1.])))
            cloud.transpose()
            ans = np.dot(RT_0, np.dot(RT_s, cloud))
            ans = np.reshape(ans[:3], (1,3))
            if res is None:
                res = ans
            else:
                res = np.concatenate((res, ans))
    write_ply(save_path=filename, points=res)