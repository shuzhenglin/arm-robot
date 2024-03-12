from kuka_controller import *
from scipy.spatial.transform import Rotation as R
import subprocess
import time

def euler2quaternion(euler):
    #rotation sequence of kuka is 'zyx'
    r = R.from_euler('ZYX', euler, degrees=True)
    quaternion = r.as_quat()
    #output sequence is (x,y,z,w)
    quaternion = [float(quaternion[3]), float(quaternion[0]), float(quaternion[1]), float(quaternion[2])]
    return quaternion

if __name__ == "__main__":
    parser.add_argument('-posf', type=str, help='give a pose file name')
    parser.add_argument('-plyf', type=str, help='give a ply file name')
    args = parser.parse_args()
    start_xml = os.join(base_path, "XML/Setpose.xml")
    stop_xml = os.join(base_path, "XML/stop_grab.xml")
    rob = Robot_connect()
    xml = Xml_dispose()
    yml = Yaml_dispose()
    #* make dir
    dirs = '/Data/Matrix_transformations/plydata/' + args.plyf
    if not os.path.exists(dirs):
        os.makedirs(dirs)

    #* grab ply
    cam = Camera_connect()
    pose_file = os.path.join(base_path, 'YAML/{}.yml'.format(args.posf))
    if os.path.exists(pose_file):
        pass
    else:
        file = open(pose_file, "w")
        file.close()
    count = 1
    zmqmsg = None
    flag = None
    pose_dic = {}
    start = time.perf_counter()
    #? Modify the number of scans
    while count <= 6:
        flag = None
        xmldata = xml.ReadXml(start_xml)
        rob.SendMsg(xmldata)
        robmsg = rob.GetMsg()
        #* recieve robot message, tell client robot already arrived
        flag = xml.findtext_fromstring(robmsg, ".//Data//Message")
        #* recieve robot message, tell client the position of robot
        data = xml.findattrib_fromstring(robmsg, ".//Data//LastPos")
        
        #TODO transform
        for key, val in data.items():
            data[key] = float(val)
        euler = [data['A'], data['B'], data['C']]
        quaternion = euler2quaternion(euler)
        pose = {}
        pose['pos'] = [data['X'], data['Y'], data['Z']]
        pose['ori'] = quaternion
        pose_name = 'end_{:0>4d}'.format(count)
        pose_dic = {}
        pose_dic[pose_name] = pose

        if os.path.getsize(pose_file):
            yml_dic = yml.LoadYaml(file=pose_file)
            yml.WriteYaml(file=pose_file,data={**yml_dic, **pose_dic})
        else:
            yml.WriteYaml(file=pose_file,data=pose_dic)
        #* communicate with camera
        while flag == "arrive":
            zmqmsg = cam.send_flag("/Data/Matrix_transformations/plydata/{0}/{1}.ply".format(args.plyf, 'object_{:0>4d}'.format(count)))
            if zmqmsg == b'ok':
                time.sleep(0.5)
                break
        count += 1
    zmqmsg = cam.send_flag("stop")
    end = time.perf_counter()
    logger.info("getpose spend time:{}".format(end - start))
    xmldata = xml.ReadXml(stop_xml)
    rob.SendMsg(xmldata)
    rob.tcp_close()

    del rob
    del cam
    # Convert to a facet
    transf = subprocess.run("python ../Matrix_transformations/pointcloud2base.py -file {}".format(args.plyf), stdout=subprocess.PIPE)
    print(transf.stdout)