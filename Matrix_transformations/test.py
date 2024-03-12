from scipy.spatial.transform import Rotation
import numpy as np
import yaml

def load_yaml(filename):
    with open(filename, 'r', encoding='utf-8') as f:
        result = yaml.load(f, Loader=yaml.FullLoader)
    return result

def write_yaml(f, data):
    with open(f, 'w', encoding='utf-8') as f:
        yaml.dump(data=data, stream=f, allow_unicode=True, sort_keys=False, default_flow_style=True, width=168)

def quaternion2rotation(ori):
    Rq = np.array((ori[1], ori[2], ori[3], ori[0]))
    Rm = Rotation.from_quat(Rq)
    rotation_matrix = Rm.as_matrix()
    return rotation_matrix

def rotation_matrix_to_euler_angles(rotation_matrix):
    r = Rotation.from_matrix(rotation_matrix)
    euler_angles = r.as_euler('ZYX', degrees=True)
    return euler_angles

if __name__ == "__main__":
    filename = "circle.yml" #TODO
    target_file = '../KUKA_Controller/YAML/target_pose.yml'
    ori = [0.012978862496493597, 0.008574672119294795, 0.9997889756033772, 0.01341746579250857]
    rot = quaternion2rotation(ori)
    pos = np.array([])
    pos.reshape(-1, 1)

    RT_0 = np.vstack((np.hstack((rot, pos)), np.array([0, 0, 0, 1])))
    #TODO RT_1
    RT_1 = load_yaml(filename)
    res = np.dot(RT_1, RT_0)
    R = res[:3, :3]
    T = res[:3, 3]
    euler = rotation_matrix_to_euler_angles(R)
    dic = load_yaml(target_file)
    i = 0
    for key in dic.keys():
        if i <= 2:
            dic[key] = str(T[i])
        else:
            dic[key] = str(euler[i - 3])
        i += 1
    write_yaml(target_file, dic)
    # print(T, "\n", euler)
