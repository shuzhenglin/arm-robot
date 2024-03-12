from tool import *

def unified_mm(file_dir):
    label_dir = []
    labels_name = os.listdir(file_dir)
    # q = np.array([0, 0 ,0])
    for filename in labels_name:
        label_dir.append(os.path.join(file_dir, filename))
    for filename in label_dir:
        plydata = PlyData.read(filename)
        points = plydata['vertex'].data.copy()
        size = len(points)
        cloud = np.empty([size, 3])
        j = 0
        for i in range(len(points)):
            point = points[i]
            p = np.array([point[0], point[1], point[2]])
            # if np.allclose(p, q):
            #     continue
            # else:
            cloud[j] = p / 1000
            j += 1
        write_ply(save_path=filename, points=cloud)
if __name__ == '__main__':
    ply_dir = os.path.join(r'D:\Matrix_transformations\ply_data', args.file)
    unified_mm(ply_dir)