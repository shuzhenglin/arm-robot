import numpy as np
from numpy import linalg

def icp_match(pts3d_ls1, pts3d_ls2):
    pt_n1 = len(pts3d_ls1)
    pt_n2 = len(pts3d_ls2)
    if(pt_n1 != pt_n2 or pt_n1<2):
        print('less than 2 view point or two coordinate not equal')
        return None, None
    pc3d_1 = np.zeros([1,3], dtype=np.float32)
    pc3d_2 = np.zeros([1,3], dtype=np.float32)

    pts3d_np1 = np.array(pts3d_ls1, dtype=np.float32)
    pts3d_np2 = np.array(pts3d_ls2, dtype=np.float32)
    
    for id in range(pt_n1):
        pc3d_1 = pc3d_1 + pts3d_np1[id]
        pc3d_2 = pc3d_2 + pts3d_np2[id]
    pc3d_1 = pc3d_1/pt_n1
    pc3d_2 = pc3d_2/pt_n2
    
    pts3d_t1 = np.zeros([pt_n1, 3], dtype=np.float32)
    pts3d_t2 = np.zeros([pt_n2, 3], dtype=np.float32)
    for i in range(pt_n1):
        pts3d_t1[i] = pts3d_np1[i] -pc3d_1
        pts3d_t2[i] = pts3d_np2[i] -pc3d_2
    
    W = np.zeros([3,3], dtype=np.float32)
    for i in range(pt_n1):
        W= W+np.dot(pts3d_t1.transpose(), pts3d_t2)
    
    U, sigma, VT = linalg.svd(W)
    R_ = np.dot(U,VT)
    
    B = linalg.eigvals(R_)
    print(B)
    if np.all(B>0):
        R=R_
    else:
        R=-R_
    T = pc3d_1.transpose() - np.dot(R, pc3d_2.transpose())
    return R, T
