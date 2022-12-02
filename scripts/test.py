import numpy as np
import matplotlib.pyplot as plt



points = np.loadtxt("xyz_raw_3d.csv", delimiter=",")
print(np.max(points[:,0]-np.min(points[:,0])), np.max(points[:,1]-np.min(points[:,1])))

# # arr1 = np.array([[0,0,0,0,0], 
# #                  [0,0,0,0,0],
# #                  [0,0,0,0,0],
# #                  [0,0,0,0,0],
# #                  [0,0,0,0,0],])
# arr2 = np.array([[1,1],
#                  [4,4],
#                  [2,3]])
# min_x = np.min(arr2[:,0])
# min_y = np.min(arr2[:,1])
# max_x = np.max(arr2[:,0])
# max_y = np.max(arr2[:,1])
# print(min_x, max_x, min_y, max_y)

# arr1 = np.zeros((max_x+1, max_y+1))
# print(arr1.shape)
# for i in range(arr2.shape[0]):
#     arr1[arr2[i,0]-1, arr2[i,1]-1] = 1
# print(arr1)
