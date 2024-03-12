import yaml
import numpy as np
import math
def quaternion2rpy(quaternion):
    #import pdb;pdb.set_trace()
    w = quaternion[0]
    x = quaternion[1]
    y = quaternion[2]
    z = quaternion[3]

    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(2 * (w * y - x * z))
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [r, p, y]

def isRotationMatrix(R):
    Rt = np.transpose(R) #矩阵转置
    shouldBeIdentity = np.dot(Rt, R) #旋转矩阵和其本身的转置相乘得到单位阵
    I = np.identity(3, dtype=R.dtype) # 单位阵
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rot2euler(R):
    # assert(isRotationMatrix(R))#断言

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])#cosγ^2cosβ^2 + sinγ^2cosβ^2 = cosβ^2(cos90°=0)
    # 求rpy会有三种情况
    singular = sy < 1e-6
    # β != 0
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2]) * 180 / np.pi
        y = math.atan2(-R[2, 0], sy) * 180 / np.pi
        z = math.atan2(R[1, 0], R[0, 0]) * 180 / np.pi
    else:
        x = math.atan(-R[1, 2], R[1, 1]) * 180 / np.pi
        y = math.atan(-R[2, 0], sy) * 180 / np.pi
        z = 0

    return [x, y, z]

def write_yaml(f, data):
    with open(f, 'w', encoding='utf-8') as f:
        yaml.dump(data=data, stream=f, allow_unicode=True, sort_keys=False, default_flow_style=True, width=148)

def add_yaml(f, data):
    with open(f, 'a', encoding='utf-8') as f:
        yaml.dump(data=data, stream=f, allow_unicode=True, sort_keys=False, default_flow_style=True, width=168)

def load_yaml(f):
    with open(f, 'r', encoding='utf-8') as f:
        result = yaml.load(f, Loader=yaml.FullLoader)
    return result
if __name__ == '__main__':
    f = r'D:\aubo-i5\aubo_control\transform\rotation.yml'
    rotation =  load_yaml(f)['R']
    rpy = rot2euler(np.array(rotation))
    print(rpy)