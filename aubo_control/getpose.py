import aubo_controller as aubocl
from aubo_controller import *
import redis


kwargs = {
    'host': 'localhost',
    'port': 6379,
    'decode_responses': True,
    'retry_on_timeout': 3,
    'max_connections': 1024  # 默认2^31
}


def dic_concat(dic1, dic2):
    return {**dic1, **dic2}

if __name__ == '__main__':
    IP = '192.168.1.10'
    PORT = 8899
    data = {}
    file = './Data/arm_pose/frame.yml' # 修改机械臂位姿存储路径
    if os.path.exists(file):
        pass
    else:
        file = open(file, "w")
        file.close()
    pool = redis.ConnectionPool(**kwargs)
    r = redis.Redis(connection_pool=pool)
    r.set('name', 'right_1') #每次首先给redis赋初值
    name = r.get('name')
    rob = AuboRobot()
    if rob.connect_rob(IP, PORT) == 0:
        pose = rob.record_robot_pose()
        data[name] = pose # 修改位姿名称
        rob.disconnect_rob()
    rob.robot_release()
    if os.path.getsize(file): # 判断文件内是否有可以用作拼接的字典
        yml_data = load_yaml(f=file)
        write_yaml(f=file,data=dic_concat(yml_data, data)) # 写入拼接后的字典
    else:
        write_yaml(f=file, data=data)

    new_name = name.split('_')[0] + '_' + str(int(name.split('_')[1]) + 1)
    r.set('name', new_name)