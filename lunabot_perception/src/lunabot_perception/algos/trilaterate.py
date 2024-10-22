# Trilateration nonlinear diff. eq. test
# Based on Reliable computation of the points of intersection of n spheres in IR n by I.D. Coope
import math
import numpy as np
from tqdm import tqdm
# import scipy.optimize as sci


# Convert Bag to csv
import bagpy
from bagpy import bagreader
import pandas as pd

# b = bagreader('test.bag')

# # # replace the topic name as per your need
# LASER_MSG = b.message_by_topic('/sensors')
# LASER_MSG
# df_laser = pd.read_csv(LASER_MSG)
# df_laser = df_laser[['uwb_dists_0','uwb_dists_1','uwb_dists_2']]
# df_laser.to_csv('test_sensors.csv', index=False)  

# Function to minimize (from cited paper (Eqn 18)
def S(v, d1, d2, d3, a1, a2, a3):
    s = (
        (np.linalg.norm(v - a1) - d1) ** 2
        + (np.linalg.norm(v - a2) - d2) ** 2
        + (np.linalg.norm(v - a3) - d3) ** 2
    )
    return s


def is_full_rank(A):
    return np.linalg.matrix_rank(A) == A.shape[0]


class Trilaterate:
    n: int = 3

    def __init__(self, pts_cfg):
        assert pts_cfg.shape[0] == self.n
        assert pts_cfg.shape[1] == self.n
        self.pts_cfg = pts_cfg

    def trilaterate(self, Dists):
        """Solution by Orthogonal decomp"""
        #from scipy import linalg
        #from scipy.linalg import solve
        # a_n = pts_cfg[-1]
        # A = pts_cfg[:-1]
        # A -= a_n
        # A = A.T
        # Q,R = linalg.qr(A)
        # R = R[:-1]

        # c = np.zeros(self.n-1)
        # for i in range(self.n-1):
        #     c[i] = 0.5 * (D[-1]**2 - D[i]**2 + np.linalg.norm(R[i])**2)
        # y = solve(R.T, c)
        # z = np.sqrt(D[-1]**2 - np.linalg.norm(y)**2)
        # v = np.zeros(self.n)
        # v[:self.n-1] = y
        # v[-1] = z

        # x = Q @ v + a_n
        # return x
        

        # pos = sci.minimize(
        #     S,
        #     [-5, -5, 0.5],
        #     args=(D[0], D[1], D[2], self.pts_cfg[0], self.pts_cfg[1], self.pts_cfg[2]),
        #     method="Powell",
        # )
        # return pos.x
        
        
    
        pt1 = pts_cfg[0]
        pt2 = pts_cfg[1]
        pt3 = pts_cfg[2]
        
        # # # Formula from: https://www.101computing.net/cell-phone-trilateration-algorithm/    
        # ## 2D trilateration
        # A = 2*pt2[0] - 2*pt1[0]
        # B = 2*pt2[1] - 2*pt1[1]
        # C = Dists[0]**2 - Dists[1]**2 - pt1[0]**2 + pt2[0]**2 - pt1[1]**2 + pt2[1]**2
        # D = 2*pt3[0] - 2*pt2[0]
        # E = 2*pt3[1] - 2*pt2[1]
        # F = Dists[1]**2 - Dists[2]**2 - pt2[0]**2 + pt3[0]**2 - pt2[1]**2 + pt3[1]**2
        # x = (C*E - F*B) / (E*A - B*D)
        # y = (C*D - A*F) / (B*D - A*E)
        # return [[x,y,0],[x,y,0]]
    
        # Formula from: https://math.stackexchange.com/q/2969614
        ## 3D Trilateration
        
        #known points
        x1 = pt1[0]
        y1 = pt1[1]
        z1 = pt1[2]
        x2 = pt2[0]
        y2 = pt2[1]
        z2 = pt2[2]
        x3 = pt3[0]
        y3 = pt3[1]
        z3 = pt3[2]
        
        #Distances
        d1 = Dists[0]
        d2 = Dists[1]
        d3 = Dists[2]
         
        epsilon = .0001 # Precision
        
        # Find the unit vector <ex1, ey1, ez1> between pts 1 and 2
        ex1 = x2 - x1 #1.5
        ey1 = y2 - y1 #0
        ez1 = z2 - z1 #0
        h = math.sqrt( ex1**2 + ey1**2 + ez1**2 ) #1.5
        if h <= epsilon:
            print("Error: First and second point are too close")
            return None
        ex1 = ex1 / h #1
        ey1 = ey1 / h #0
        ez1 = ez1 / h #0 

        # Find the unit vector <ex2, ey2, ez2> = pos3 - pos1 - e1*(e1 (dot) (pos3-pos1))
        
        # dot product of e1 and (pos3-pos1)
        i = ex1*(x3 - x1) + ey1*(y3 - y1) + ez1*(z3 - z1) #0+0+0=0

        ex2 = x3 - x1 - i*ex1 #0
        ey2 = y3 - y1 - i*ey1 #0.9
        ez2 = z3 - z1 - i*ez1 #0
        t = math.sqrt(ex2*ex2 + ey2*ey2 + ez2*ez2) #magnitude 0.9
        if t <= epsilon:
            print("Error: the three fixed points are too close to being on the same line.")
        
        #make unit vector
        ex2 = ex2 / t #0
        ey2 = ey2 / t #1
        ez2 = ez2 / t #0

        # dot product of e2 and (pos3-pos1)
        j = ex2*(x3 - x1) + ey2*(y3 - y1) + ez2*(z3 - z1) #0.9
        
        if j <= epsilon and j >= -epsilon:
            print("Error: the three fixed points are too close to being on the same line.")

        # cross product of e1 and e2
        ex3 = ey1*ez2 - ez1*ey2 #0
        ey3 = ex1*ez2 - ez1*ex2 #0
        ez3 = ex1*ey2 - ey1*ex2 #1
        
        
        u = (d1**2 - d2**2 + h**2) / (2*h) 
        v = (d1**2 - d3**2 + i*(i - 2*u) + j**2) / (2*j)
        ww = d1**2 - u**2 - v**2
        if ww < -epsilon: # No Sols
            print(f"Error: no Solutions ww: {ww}")
            return None
        elif ww < epsilon: # 1 Sol
            x = (x1 + u*ex1 + v*ex2)
            y = (y1 + u*ey1 + v*ey2)
            z = (z1 + u*ez1 + v*ez2) #0
            return [[x,y,z],[x,y,z]] #Return twice for code later
        else: # 2 Sols
            w = math.sqrt(ww)
            
            x = (x1 + u*ex1 + v*ex2 + w*ex3)
            y = (y1 + u*ey1 + v*ey2 + w*ey3)
            z = (z1 + u*ez1 + v*ez2 + w*ez3)
            a = [x,y,z]
            x = (x1 + u*ex1 + v*ex2 - w*ex3)
            y = (y1 + u*ey1 + v*ey2 - w*ey3)
            z = (z1 + u*ez1 + v*ez2 - w*ez3)
            b = [x,y,z]
            return [a,b]
        

    

# Find distances
def dist_from_pos(x, a1, a2, a3):
    d1 = np.linalg.norm(a1 - x)
    d2 = np.linalg.norm(a2 - x)
    d3 = np.linalg.norm(a3 - x)
    return (d1, d2, d3)

import pandas as pd
if __name__ == "__main__":

    error_accumulator = 0

    # Beacon positions
    # a1,a2,a3 = np.array([0, 0, 0]), np.array([1.5, 0, 0]), np.array([1.5, 0.9, 0])
    # a1,a2,a3 = np.array([0, 0, 0]), np.array([1.5, 0, 0]), np.array([0, 0.9, 0])
    # a1,a2,a3 = np.array([0, 0, 0]), np.array([0, 0.9, 0]), np.array([1.5, 0.9, 0])
    # a1,a2,a3 = np.array([0, 0, 0]), np.array([0, 0.9, 0]), np.array([1.5, 0, 0])
    # a1,a2,a3 = np.array([0, 0, 0]), np.array([1.5, 0.9, 0]), np.array([1.5, 0, 0]) #Maybe
    # a1,a2,a3 = np.array([0, 0, 0]), np.array([1.5, 0.9, 0]), np.array([0, 0.9, 0])
    
    # a1,a2,a3 = np.array([1.5, 0, 0]), np.array([0, 0, 0]), np.array([1.5, 0.9, 0])
    # a1,a2,a3 = np.array([1.5, 0, 0]), np.array([0, 0, 0]), np.array([0, 0.9, 0])
    # a1,a2,a3 = np.array([1.5, 0, 0]), np.array([0, 0.9, 0]), np.array([1.5, 0.9, 0])
    # a1,a2,a3 = np.array([1.5, 0, 0]), np.array([0, 0.9, 0]), np.array([0, 0, 0]) #Maybe
    # a1,a2,a3 = np.array([1.5, 0, 0]), np.array([1.5, 0.9, 0]), np.array([0, 0, 0])
    # a1,a2,a3 = np.array([1.5, 0, 0]), np.array([1.5, 0.9, 0]), np.array([0, 0.9, 0])

    # a1,a2,a3 = np.array([0, 0.9, 0]), np.array([0, 0, 0]), np.array([1.5, 0, 0])
    # a1,a2,a3 = np.array([0, 0.9, 0]), np.array([0, 0, 0]), np.array([1.5, 0.9, 0])
    # a1,a2,a3 = np.array([0, 0.9, 0]), np.array([1.5, 0, 0]), np.array([0, 0, 0])
    # a1,a2,a3 = np.array([0, 0.9, 0]), np.array([1.5, 0, 0]), np.array([1.5, 0.9, 0]) #Maybe
    # a1,a2,a3 = np.array([0, 0.9, 0]), np.array([1.5, 0.9, 0]), np.array([0, 0, 0]) 
    # a1,a2,a3 = np.array([0, 0.9, 0]), np.array([1.5, 0.9, 0]), np.array([1.5, 0, 0])
    
    # a1,a2,a3 = np.array([1.5, 0.9, 0]), np.array([0, 0, 0]), np.array([1.5, 0, 0])
    # a1,a2,a3 = np.array([1.5, 0.9, 0]), np.array([0, 0, 0]), np.array([0, 0.9, 0]) #Maybe
    # a1,a2,a3 = np.array([1.5, 0.9, 0]), np.array([1.5, 0, 0]), np.array([0, 0, 0])
    # a1,a2,a3 = np.array([1.5, 0.9, 0]), np.array([1.5, 0, 0]), np.array([0, 0.9, 0])
    # a1,a2,a3 = np.array([1.5, 0.9, 0]), np.array([0, 0.9, 0]), np.array([0, 0, 0])
    # a1,a2,a3 = np.array([1.5, 0.9, 0]), np.array([0, 0.9, 0]), np.array([1.5, 0, 0])
    
    #rotations of the correct position
    # a1,a2,a3 = np.array([0, 0, 0]), np.array([1.5, 0.9, 0]), np.array([1.5, 0, 0]) #Maybe (-3.121,-4.3967,+-2.809)

    # a1,a2,a3 = np.array([1.5, 0, 0]), np.array([0, 0.9, 0]), np.array([0, 0, 0]) #Maybe (4.621,-4.3967, +-2.809)

    # a1,a2,a3 = np.array([0, 0.9, 0]), np.array([1.5, 0, 0]), np.array([1.5, 0.9, 0]) #Maybe (-3.121,5.2967, +- 2.809)

    # a1,a2,a3 = np.array([1.5, 0.9, 0]), np.array([0, 0, 0]), np.array([0, 0.9, 0]) #Maybe (4.621,5.2967,+- 2.809)
    
    a1,a2,a3 = np.array([0, 0, 0]), np.array([1.8288, 0, 0]), np.array([0, 1.8288, 0]) 
    
    # a1 = np.array([0, -0.876, 0])  # TX1
    # a2 = np.array([0, 0, 0])  # TX2
    # a3 = np.array([1.105, 0, 0])  # TX3

    pts_cfg = np.array([a1, a2, a3])

    tril = Trilaterate(pts_cfg)
    df = pd.read_csv("test_sensors.csv")
    df = df[['uwb_dists_0','uwb_dists_1','uwb_dists_2']]
    poses = []
    print(df.head(3))
    minz = -1
    maxz = 5
    for index, row in df.iterrows():
        # print(row['uwb_dists_0'])
        # break
        D = np.array([row['uwb_dists_2'],row['uwb_dists_0'],row['uwb_dists_1']])
        # print(D)
        posLst = tril.trilaterate(D)
        
        # print(posLst)
        
        if(posLst==None):
            pos = [None,None,None]
        elif(posLst[0][2]>maxz or posLst[0][2]<minz):
            pos = posLst[1]
        elif(posLst[1][2]>maxz or posLst[1][2]<minz):
            pos = posLst[0]
        else:
            pos = posLst[0]
        
        poses.append(pos)
        print(posLst)
        # break
    
    df['posex'] = [i[0] for i in poses]
    df['posey'] = [i[1] for i in poses]
    df['posez'] = [i[2] for i in poses]
    df.to_csv('poses.csv', index=False)  





    
    # D = np.array([d1, d2, d3])
    # posLst = tril.trilaterate(D)
    
    # print(posLst)
    # if(posLst==None):
    #     # print("Error")
    #     continue
    # if(posLst[0][2]>maxz or posLst[0][2]<minz):
    #     pos = posLst[1]
    # elif(posLst[1][2]>maxz or posLst[1][2]<minz):
    #     pos = posLst[0]
    # else:
    #     pos = posLst[0]


    # trials = 100

    # running_error = np.zeros((trials))
    
    #Tests how much error the algorithm has on average 
    # for i in range(0, trials):
    #     minz = -1
    #     maxz = 2
    #     x = np.array([np.random.uniform(-5, -3), np.random.uniform(-5, -3), np.random.uniform(minz,maxz)])
    #     d1, d2, d3 = dist_from_pos(x, a1, a2, a3)

    #     # Add noise to signals
    #     d1 += np.random.uniform(-0.001, 0.001)
    #     d2 += np.random.uniform(-0.001, 0.001)
    #     d3 += np.random.uniform(-0.001, 0.001)

    #     D = np.array([d1, d2, d3])
    #     posLst = tril.trilaterate(D)
        
    #     print(posLst)
    #     if(posLst==None):
    #         # print("Error")
    #         continue
    #     if(posLst[0][2]>maxz or posLst[0][2]<minz):
    #         pos = posLst[1]
    #     elif(posLst[1][2]>maxz or posLst[1][2]<minz):
    #         pos = posLst[0]
    #     else:
    #         pos = posLst[0]
        
    #     # Perform optimizations
    #     error = np.linalg.norm(pos[:2] - x[:2]) ** 2

    #     print("Iteration", i, "X:", x, "Result", pos)
    #     print("err", error)
    #     running_error[i] = error
    
    # print("Average error:", np.mean(running_error))
    # print("mean squared error", np.mean(running_error ** 2))
    
