from tool import *
import os
import subprocess

def file_process(filename, n):
    newname = filename.split('.')[0] + '{}.yml'.format(n)
    return newname

if __name__ == '__main__':
    ply_dir = os.path.join(base_path, 'ply_data/' + args.file) #* 修改ply文件路径
    res_dir = os.path.join(base_path, 'result') #* 修改储存结果文件结果
    calc_dir = os.path.join(base_path, 'calc_data')
    robotpos_file = os.path.join(base_path, 'robot/{}.yml'.format(args.file)) #* 修改机械臂位姿文件

    if os.path.exists(ply_dir):
        pass
    else:
        os.makedirs(ply_dir)

    if os.path.exists(res_dir):
        pass
    else:
        os.makedirs(res_dir)

    unit = subprocess.run("python Unified_unit.py -file {}".format(args.file), stdout=subprocess.PIPE)
    print(unit.stdouts)
    
    endpos_list = load_yaml(robotpos_file)
    calc_mode = 1 # 修改RTs
    ply_names = os.listdir(ply_dir)
    calc_names = os.listdir(calc_dir)

    calc_file = os.path.join(calc_dir, 'calc_' + '{}.yml'.format(calc_mode))
    calc_RT = read_calc(calc_file)
    R, T = calc_RT
    RT = matrix_concat(R, T)
    concate_ply(ply_dir, 'concat3', res_dir, calc_file, robotpos_file)