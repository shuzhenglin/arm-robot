from eye_in_hand import EyeInHand_calib

if __name__ =='__main__':
    # config_file = './config/conf.yml'
    # image_folder = './data/images/'
    # pose_folder = './data/poses/'
    # result_folder = './data/results'
    # EyeInHand_calib.eyeInHandCalib(config_file, image_folder, pose_folder, result_folder)
    
    config_file = './config/conf_cg.yml'
    image_folder = './data_cg/images'
    pose_folder = './data_cg/poses/'
    result_folder = './data_cg/results'
    EyeInHand_calib.eyeInHandCalib(config_file, image_folder, pose_folder, result_folder)