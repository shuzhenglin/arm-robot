from verify import *
import os

logfile = './logfiles/make_template time.log'
loggerv = logging.getLogger(__name__)
rh = logging.handlers.TimedRotatingFileHandler(logfile, 'D')
logging.basicConfig(filename=logfile, level = logging.INFO, format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
loggerv.addHandler(rh)


if __name__  == '__main__':
    rob = AuboRobot()
    current_dir = os.path.dirname(os.path.abspath(__file__))
    Data_folder = os.path.join(current_dir, 'Data')
    calc_folder = os.path.join(current_dir, r'Data\calc_data')
    exe_folder = os.path.join(current_dir, 'exe')

    ply_folder = os.path.join(Data_folder, r'plydata\test')
    ply_res_folder = os.path.join(Data_folder, r'result\plydata\test')

    pos_folder = os.path.join(Data_folder, 'armpose')
    img_folder = os.path.join(Data_folder, 'Images')
    calc_file = os.path.join(calc_folder, 'test_calc.yml') #*标定参数两边一定要一起换
    
    os.makedirs(ply_folder, exist_ok=True)
    os.makedirs(ply_res_folder, exist_ok=True)
    os.makedirs(pos_folder, exist_ok=True)
    os.makedirs(img_folder, exist_ok=True)

    if rob.connect_rob(Robot_IP, Robot_PORT) == 0:
        # template
        socket = zmq_connect()
        facet_name = 'test' #'facet_temp'

        # 计时
        grab_start = time.perf_counter()
        grab_ply(rob, ply_folder, pos_folder, facet_name, socket)
        grab_end = time.perf_counter()
        spend_time = grab_end - grab_start
        loggerv.info('grab spend time:{}'.format(spend_time)) # 面扫时间
        print('grab spend time:', spend_time)

        # 计时
        pose_file = os.path.join(r'D:\Code\Line_scan\Data\armpose', '{}.yml'.format(facet_name))
        concate_start = time.perf_counter()
        concate_ply(ply_folder, facet_name, ply_res_folder, calc_file, pose_file)
        concate_end = time.perf_counter()
        spend_time = concate_end - concate_start
        loggerv.info('concate spend time:{}'.format(spend_time)) # 面扫时间
        print('concate spend time:', spend_time)
        
        #!移动
        # cur_pos = rob.get_robot_pose()
        # y = -0.05 # !确定一下移动的方向
        # rob.move_in_flange(pose=cur_pos, y_move=y)

        # pose = rob.get_robot_pose()
        # armpose_file = r'D:\Code\Line_scan\Data\result\armpose\temp_pose.yml'
        # write_yaml(f=armpose_file, data=pose)
        # 2D
        # image_name = 'image_temp.png'
        # img_file = os.path.join(img_folder, image_name)
        # camera_2d = os.path.join(exe_folder, 'test_camera ')
        # p = subprocess.Popen(camera_2d + img_file, shell=True)
        # p.wait()

        #! 移动
        # joint = [-0.9966002106666565, -0.029870357364416122, 1.1643990278244019, -0.3574163317680359, 1.5653859376907349, 1.6576992273330688]
        # rob.set_cart(joint)

        rob.disconnect_rob()
    rob.robot_release()