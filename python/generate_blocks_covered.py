import numpy as np
import math as m
from math import cos
from math import sin
from math import pi
import matplotlib.pyplot as plt
import json
from tqdm import tqdm

disc = 0.2 #Grid size

l = 0.79 #Length of car
w = 0.59 #Width of car

n1 = 101 #Discretization size along car boundary  
n2 = 101 #Discretization size along car interior

SIZE = 100000

def getIndex(x_grid,y_grid):
    return (y_grid*SIZE+x_grid)

def interpolate(x1,y1,x2,y2,n):
    line = [[x1,y1]]
    for i in range(1,n):
        xtemp = x1 + (i/(n-1)*(x2-x1))
        ytemp = y1+ (i/(n-1)*(y2-y1))
        line.append([xtemp,ytemp])
    return np.array(line)

def getBlockIndex(x,y):
    blockX =  m.floor((x+(disc/2))/disc)
    blockY =  m.floor((y+(disc/2))/disc)
    return [blockX, blockY]

def getCornerGlobalPose(x,y,theta):
    R_I_B = [[cos(theta),-sin(theta)],[sin(theta),cos(theta)]]
    CB = np.array([[l/2,w/2],[-l/2,w/2],[-l/2,-w/2],[l/2,-w/2]])
    CI = np.zeros([4,2])
    for i in range(0,4):
        CI[i] = [x,y]+R_I_B@CB[i]
    return CI
    

def getInterpolationLines(x,y,theta):
    CI = getCornerGlobalPose(x,y,theta)
    line1 = interpolate(CI[0][0],CI[0][1],CI[3][0],CI[3][1],n1)
    line2 = interpolate(CI[1][0],CI[1][1],CI[2][0],CI[2][1],n1)
    all_lines = line2

    for i in range(0,n1):
        line = interpolate(line1[i][0],line1[i][1],line2[i][0],line2[i][1],n2)
        all_lines = np.append(all_lines,line,0)
        
    return all_lines

def getCoveredIndices(x,y,theta):
    all_lines = getInterpolationLines(x,y,theta)
    all_block_indices = [getBlockIndex(all_lines[0][0],all_lines[0][1])]
    all_indices = [getIndex(all_block_indices[0][0],all_block_indices[0][1])]
    
    for i in range(1,np.shape(all_lines)[0]):
        new_block_indices = getBlockIndex(all_lines[i][0],all_lines[i][1])
        new_index = getIndex(new_block_indices[0],new_block_indices[1])
        if new_index not in all_indices:
            all_block_indices = np.append(all_block_indices, [new_block_indices],0)
            all_indices = np.append(all_indices, new_index)
            
    return all_block_indices

if __name__ == "__main__":
    x = 0
    y = 0 
    theta = pi/6

    all_lines = getInterpolationLines(x,y,theta)
    all_block_indices = getCoveredIndices(x,y,theta)
    primtive_covered_indices_str = [str(ele) for ele in all_block_indices ]
    # print(primtive_covered_indices_str)
    # plt.scatter(all_block_indices.T[0],all_block_indices.T[1])
    # plt.xlim([-15, 15])
    # plt.ylim([-15, 15])
    # plt.grid()
    # plt.show()

    f = open('mprims.json')
    data = json.load(f)

    # print(data["0.0"]["0"])

    labels_angles = [str(l) for l in np.arange(0,360,11.25)]
    labels_primitives = [str(l) for l in np.arange(0,7,1)]

    # labels_angles = ["0.0"]
    # labels_primitives = ["0"]

    for la in tqdm(labels_angles, desc = "Robot Angle"):
        # print(la)
        for lp in tqdm(labels_primitives, desc="Primitve"):
            # print(lp)
            x_all = data[la][lp]['mprim'][0]
            y_all = data[la][lp]['mprim'][1]
            theta_all = data[la][lp]['mprim'][2]

            primtive_covered_indices = getCoveredIndices(x_all[0],y_all[0],theta_all[0])
            primtive_covered_indices_str = [str(ele) for ele in primtive_covered_indices ]

            for i in range(1,np.shape(x_all)[0]):
                # print(x_all[i], y_a÷ll[i], theta_all[i])
                covered_indices = getCoveredIndices(x_all[i],y_all[i],theta_all[i])
                for ele in covered_indices:
                    if str(ele) not in primtive_covered_indices_str:
                        # print(np.shape(primtive_covered_indices))
                        # print(np.shape([ele]))
                        primtive_covered_indices = np.append(primtive_covered_indices,[ele],0)
                        primtive_covered_indices_str = np.append(primtive_covered_indices_str,str(ele))

            data[la][lp]['collisions'] = [primtive_covered_indices.T[0].tolist(),primtive_covered_indices.T[1].tolist()]


    with open('mprims_new.json', 'w') as fp:
        json.dump(data, fp)
    
    # print(primtive_covered_indices_str)


    # plt.scatter(primtive_covered_indices.T[0],primtive_covered_indices.T[1])
    # plt.xlim([-10, 20])
    # plt.ylim([-10, 10])
    # plt.grid()
    # plt.show()
    