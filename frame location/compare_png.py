import os.path

import cv2
import numpy as np
import time
base_path = r"D:\Code\Dot_Match\Data\result"
src1_path = os.path.join(base_path , 'temp', 'chess.png')
src2_path = os.path.join(base_path, 'target', 'chess.png')
src1 = cv2.imread(src1_path)
src2 = cv2.imread(src2_path)
src1 = src1[100:src1.shape[0]-100,0:src1.shape[1]]
src2 = src2[100:src2.shape[0]-100,0:src2.shape[1]]
add_img = cv2.addWeighted(src1, 0.5, src2, 0.5, 1)  # 图像融合,方便显示

src1 = np.float32(src1)
src2 = np.float32(src2)
src1 = cv2.cvtColor(src1, cv2.COLOR_BGR2GRAY)
src2 = cv2.cvtColor(src2, cv2.COLOR_BGR2GRAY)

# 函数说明：利用傅里叶变化，在频域进行相位匹配从而计算两张图片的平移量
start_time = time.time()
dst = cv2.phaseCorrelate(src1,src2)
end_time =time.time()
print("offsetv1 = {:.4f}s".format(end_time-start_time))
cv2.putText(add_img,str(dst[0]),(100,100),cv2.FONT_HERSHEY_COMPLEX,1, (0,255,0), 3)
cv2.imwrite(base_path + "\\" + "{}.jpg".format(src1_path.split("\\")[-1][:-4] +"+"+ src2_path.split("\\")[-1][:-4]), add_img)
print(dst[0])