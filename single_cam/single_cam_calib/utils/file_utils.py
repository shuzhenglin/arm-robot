import glob
import yaml
import cv2 as cv

def loadConfig(file):
    with open(file, 'r', encoding='utf-8') as f:
        content = f.read()
        config_info= yaml.load(content, Loader=yaml.FullLoader)
        return config_info
    

def loadImages(folder):
    image_files = sorted(glob.glob('{}/*.bmp'.format(folder)), reverse=False)
    cam_images = [cv.imread(file, cv.IMREAD_COLOR) for file in image_files]
    return cam_images

def saveYaml(file, intrinsic, distort):
    param_dict = {}
    param_dict['intrinsic'] = intrinsic.tolist()
    param_dict['distort'] = distort.tolist()
    with open(file, 'w', encoding='utf-8') as f:
        yaml.dump(stream=f, data=param_dict)