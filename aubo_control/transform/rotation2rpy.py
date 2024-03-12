import numpy as np
from scipy.spatial.transform import Rotation as R

R = []

r = R.from_matirix(R)
print(r.as_matrix())
euler_angles = r.as_euler('xyz', degrees=True)
print(euler_angles)