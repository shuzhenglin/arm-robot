import numpy as np
Rw2c_mat_list = [[[1, 1, 0],[0, 1, 0],[0, 0, 1]], [[0, 0, -1],[0, 1, 0], [1, 0, 0]]]
Tw2c_mat_list = [[[1], [1], [1]], [[0], [0], [0]]]

Pw2c_mat_list = [np.concatenate((R,T), axis=1) for R,T in zip(Rw2c_mat_list, Tw2c_mat_list)]
Pw2c_mat_list = [np.concatenate((M, np.array([[0,0,0,1]])), axis=0) for M in Pw2c_mat_list]

print(Pw2c_mat_list)