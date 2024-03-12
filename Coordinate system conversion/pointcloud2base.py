from tool import *
import os

def file_process(filename, n):
    newname = filename.split('.')[0] + '{}.yml'.format(n)
    return newname

if __name__ == '__main__':
    ply_dir = r'D:\Matrix_transformations\ply_data\7-14' # 修改ply文件路径(和机械臂位姿名称相同)
    res_dir = r'D:\Matrix_transformations\result' # 修改储存结果文件结果
    calc_dir = r'D:\Matrix_transformations\calc_data'
    robotpos_file = r'D:\Matrix_transformations\robot\7-14.yml' #修改机械臂位姿文件

    if os.path.exists(ply_dir):
        pass
    else:
        os.makedirs(ply_dir)

    if os.path.exists(res_dir):
        pass
    else:
        os.makedirs(res_dir)

    
    endpos_list = load_yaml(robotpos_file)
    calc_mode = 5 # 修改RTs
    ply_names = os.listdir(ply_dir)
    calc_names = os.listdir(calc_dir)

    calc_file = os.path.join(calc_dir, 'calc_' + '{}.yml'.format(calc_mode))
    calc_RT = read_calc(calc_file)
    R, T = calc_RT
    RT = matrix_concat(R, T)
    for j in range(len(ply_names)):
        ply_name = os.path.join(ply_dir, ply_names[j])
        result_file = os.path.join(res_dir , ply_names[j])
        # 保持处理后的ply和原始ply文件名称相同，包括R0的命名也对应相同
        cloud_pts = read_ply(ply_name)
        endpos = ply_names[j].split('.')[0]
        transform(filename=result_file, endpos=endpos_list[endpos], RT_s=RT, cloud_pts=cloud_pts)