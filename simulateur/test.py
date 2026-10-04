import numpy as np

l = np.array([[1,2], [1,2]])


l_sum = np.array([np.sum(l[:, 0]),  np.sum(l[:, 1])])

print(l_sum)