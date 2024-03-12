import numpy as np
import cv2 as cv
from utils import file_utils
import copy
import os
from PIL import Image
from threading import Thread
# from matplotlib import pyplot

def task(img, index, pattern_size, refine, term_criteria, chessboard_corners, image_indexes):
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray_img, pattern_size)
    if ret:
        if refine:
            corners = cv.cornerSubPix(gray_img, corners, (11, 11), (-1, -1), term_criteria) 
        chessboard_corners.append(corners)
        image_indexes.append(index)
    else:
        print("No chessboard found in image: ", index)

def findChessboardCorners(images, pattern_size, refine=True):
    chessboard_corners = []
    image_indexes = []
    term_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_COUNT, 30, 0.001)
    task_list = []
    for index, img in enumerate(images):
        t=Thread(target=task, kwargs={'img':img, 'index':index, 'pattern_size':pattern_size, 
                                      'refine':refine, 'term_criteria':term_criteria, 
                                      'chessboard_corners':chessboard_corners,
                                      'image_indexes':image_indexes})
        task_list.append(t)
    for t in task_list:
        t.start()
    for t in task_list:
        t.join()
    return chessboard_corners, image_indexes
    
def calcReprojectErrors(chess_corners_2d, obj_points_3d, intrinsic, dist_coef, rvecs, tvecs):
    reproject_errors = []
    err_x_list = []
    err_y_list = []
    for i in range(len(chess_corners_2d)):
        img_points, _ = cv.projectPoints(obj_points_3d[i], rvecs[i], tvecs[i], intrinsic, dist_coef)
        err = cv.norm(chess_corners_2d[i], img_points, cv.NORM_L2)/len(img_points)
        err_x = [(chess_corner[0, 0]-img_pt[0, 0]) for chess_corner, img_pt in zip(chess_corners_2d[i], img_points)]
        err_y = [(chess_corner[0, 1]-img_pt[0, 1]) for chess_corner, img_pt in zip(chess_corners_2d[i], img_points)]
        err_x_list.append(err_x)
        err_y_list.append(err_y)
        reproject_errors.append(err)
    return err_x_list, err_y_list, reproject_errors

def analysisError(err_x_list, err_y_list, reproject_err_list, image_indexes):
    for i in range(len(reproject_err_list)):
        print('image index: %d, reproject error: %f'%(image_indexes[i], reproject_err_list[i]))
    max_err = max(reproject_err_list)
    idx = reproject_err_list.index(max_err)
    print('max reprojection error: image index: %d, reprojection error: %f'%(idx, max_err))
    
    avg_err = sum(reproject_err_list)/len(reproject_err_list)
    print('mean reprojection error: %f'%(avg_err))

def singleCamCalib(config_file, image_folder, result_folder):
    config_info = file_utils.loadConfig(config_file)
    img_list = file_utils.loadImages(image_folder)
    
    assert(len(img_list)!=0)    
    
    chess_info = config_info['chessboard']
    pattern_size = (chess_info['num_x'], chess_info['num_y'])
    square_size = (chess_info['size_x'], chess_info['size_y'])
    
    chessboard_corners, image_indexes = findChessboardCorners(img_list, pattern_size)
    print('find Chessboard Corners done')
    
    detect_folder = os.path.join(result_folder, 'corners')
    os.makedirs(detect_folder, exist_ok=True)
    for nn in range(len(image_indexes)):
        tmp_img = copy.deepcopy(img_list[image_indexes[nn]])
        cv.drawChessboardCorners(tmp_img, pattern_size, chessboard_corners[nn], True)
        cv.imwrite(os.path.join(detect_folder, '{}.jpg'.format(image_indexes[nn])), tmp_img)
        
    if len(image_indexes)<2:
        print('chessboard detection all failed!')
        return
        
    object_points_3d = np.zeros((pattern_size[0]*pattern_size[1], 3), dtype=np.float32)
    object_points_3d[:,:2]=np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1,2)*square_size
    object_points_list = [object_points_3d for i in range(len(image_indexes))]
    
    image_size = img_list[0].shape[:2]
    rms, intrinsic_mat, distort_mat, rvecs, tvecs = cv.calibrateCamera(object_points_list, 
                                                                       chessboard_corners, 
                                                                       image_size, 
                                                                       None, 
                                                                       None)
    print('rms: ', rms)
    print('intrinsic matrix: ', intrinsic_mat)
    print('distort coefficient: ', distort_mat)

    file_utils.saveYaml(os.path.join(result_folder, 'camera.yml'), intrinsic_mat, distort_mat)
    
    err_x_list, err_y_list, reproject_err_list = calcReprojectErrors(chessboard_corners, 
                                                                     object_points_list, 
                                                                     intrinsic_mat, 
                                                                     distort_mat, 
                                                                     rvecs, 
                                                                     tvecs)
    analysisError(err_x_list, err_y_list, reproject_err_list, image_indexes)

def bmptojpg(image_folder):
    json_dir = image_folder
    label_names = os.listdir(json_dir)
    label_dir = []
    for filename in label_names:
        label_dir.append(os.path.join(json_dir, filename))

    for i,filename in enumerate(label_dir):
        im = Image.open(filename)
        newname = label_names[i].split('.')[0] + '.jpg'
        im.save(os.path.join(json_dir, newname))