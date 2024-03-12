from verify import *
from robot.aubo_controller import *

if __name__ == '__main__':
    rob = AuboRobot()
    current_dir = os.path.dirname(os.path.abspath(__file__))
    ply_folder = os.path.join(current_dir, r'Data\plydata\temp')
    ply_res_folder = os.path.join(current_dir, r'Data\result\temp')
    pose_folder = os.path.join(current_dir, r'Data\armpose\temp')
    calc_file = os.path.join(current_dir, r'Data\calc_data\calc_0.yml')

    os.makedirs(ply_folder, exist_ok=True)
    os.makedirs(ply_res_folder, exist_ok=True)
    os.makedirs(pose_folder, exist_ok=True)

    if rob.connect_rob(Robot_IP, Robot_PORT) == 0:
        socket = zmq_connect()
        processing_temp(rob, ply_folder, ply_res_folder, pose_folder, calc_file, socket)
        rob.disconnect_rob()
    rob.robot_release()