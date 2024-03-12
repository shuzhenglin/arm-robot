from kuka_controller import *
from scipy.spatial.transform import Rotation as R
import time


def euler2quaternion(euler):
    #rotation sequence of kuka is 'zyx'
    r = R.from_euler('ZYX', euler, degrees=True)
    quaternion = r.as_quat()
    #output sequence is (x,y,z,w)
    quaternion = [float(quaternion[3]), float(quaternion[0]), float(quaternion[1]), float(quaternion[2])]
    return quaternion

if __name__ == "__main__":
    parser.add_argument('-number', type=int, default=1, help='give a number of position')
    args = parser.parse_args()
    flagxml = os.path.join(base_path, 'XML/Getpose.xml')
    rob = Robot_connect()
    xml = Xml_dispose()
    yml = Yaml_dispose()

    xmldata = xml.ReadXml(flagxml)
    start = time.perf_counter()
    rob.SendMsg(xmldata)
    #* tell client set success
    msg = rob.GetMsg()
    end = time.perf_counter()
    logger.info("getpose spend time:{}".format(end - start))
    rob.tcp_close()

    data = xml.findattrib_fromstring(msg, ".//Data//LastPos")

    # print(type(pose['X']))
    for key, val in data.items():
        data[key] = float(val)
    # print(type(pose['X']))
    euler = [data['A'], data['B'], data['C']]
    quaternion = euler2quaternion(euler)
    # print(quaternion)
    pose = {}
    pose['pos'] = [data['X'], data['Y'], data['Z']]
    pose['ori'] = quaternion
    pose_name = 'end_{:0>4d}'.format(args.number)
    pose_dic = {}
    pose_dic[pose_name] = pose
    # print(pose_dic)
    
    pose_file = os.path.join(base_path, 'YAML/current_pose.yml')
    if os.path.exists(pose_file):
        pass
    else:
        file = open(pose_file, "w")
        file.close()
    if os.path.getsize(pose_file):
        yml_dic = yml.LoadYaml(file=pose_file)
        yml.WriteYaml(file=pose_file,data={**yml_dic, **pose_dic})
    else:
        yml.WriteYaml(file=pose_file,data=pose_dic)

    del rob