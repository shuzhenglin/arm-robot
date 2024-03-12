from cam_calib import SingleCam_calib

if __name__ == '__main__':
    config_file = './config/conf.yml'
    image_folder = './data/images_pt'
    result_folder = './data/results'
    #SingleCam_calib.bmptojpg(image_folder)
    SingleCam_calib.singleCamCalib(config_file, image_folder, result_folder)