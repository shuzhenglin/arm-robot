from verify import *

if __name__ == '__main__':
    rob = AuboRobot()
    current_dir = os.path.dirname(os.path.abspath(__file__))
    ply_folder = os.path.join(current_dir, r'Data\plydata\target')
    ply_res_folder = os.path.join(current_dir, r'Data\result\target')
    pose_folder = os.path.join(current_dir, r'Data\armpose\target')
    calc_file = os.path.join(current_dir, r'Data\calc_data\calc_0.yml')

    os.makedirs(ply_folder, exist_ok=True)
    os.makedirs(ply_res_folder, exist_ok=True)
    os.makedirs(pose_folder, exist_ok=True)

    temp_cpt = os.path.join(current_dir, r'Data\result\temp\cpt\integration_point.ply')
    target_cpt = os.path.join(current_dir, r'Data\result\target\cpt\integration_point.ply')
    T122_file = os.path.join(current_dir, r'Data\result\armpose\T122.yml')

    if rob.connect_rob(Robot_IP, Robot_PORT) == 0:
        socket = zmq_connect()
        start_glob = time.perf_counter()
        processing_target(rob, ply_folder, ply_res_folder, pose_folder, calc_file, socket)
        end_glob = time.perf_counter()
        loggerv.info('total spend time:{}'.format(end_glob - start_glob))
        #* 验证
        verify_func(rob, ply_res_folder, temp_cpt, target_cpt, T122_file)

        rob.disconnect_rob()
    rob.robot_release()