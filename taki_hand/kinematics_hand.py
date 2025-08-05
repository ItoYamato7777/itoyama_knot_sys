import math
import numpy as np
from math import sin, cos, radians, pi, sqrt 
from geo import FRAME, MATRIX, VECTOR

b_len = 0.089 #[m] length between each finger
fing_len = [0.023, 0.070, 0.0615] #[m]
t_wrist2fing12 = FRAME(xyzrpy = [-(74.1+7)/1000.,0, 0, pi, 0, pi])
t_wrist2fing01 = t_wrist2fing12 * FRAME(xyzrpy = [0.,  b_len/4., b_len*sqrt(3)/4., radians(120), 0, 0]) 
t_wrist2fing20 = t_wrist2fing12 * FRAME(xyzrpy = [0., -b_len/4., b_len*sqrt(3)/4.,-radians(120), 0, 0])
t_wrist2hand_c = t_wrist2fing12 * FRAME(xyzrpy = [0., 0., b_len*sqrt(3)/6., 0, 0, 0])
t_wrist2urg = FRAME(xyzrpy = [-0.054, 0, 0, 0, 0, pi])

t_wrist2fing0 = t_wrist2fing12 * FRAME(xyzrpy = [0,  0, b_len*sqrt(3)/2, 0, 0, 0])
t_wrist2fing1 = t_wrist2fing12 * FRAME(xyzrpy = [0,  b_len/2, 0, -radians(120), 0, 0])
t_wrist2fing2 = t_wrist2fing12 * FRAME(xyzrpy = [0, -b_len/2, 0,  radians(120), 0, 0])

def t_joint1(angle):
  si = sin(angle)
  co = cos(angle)
  return np.array([[1., 0.,  0., 0.],
                   [0., co, -si, 0.],
                   [0., si,  co, 0.],
                   [0., 0.,  0., 1.]])

def t_joint2(angle):
  si = sin(angle)
  co = cos(angle)
  return np.array([[ co, 0., si,  fing_len[0]],
                   [ 0., 1., 0.,  0.        ],
                   [-si, 0., co,  0.        ],
                   [ 0., 0., 0.,  1.        ]])

def t_joint3(angle):
  si = sin(angle)
  co = cos(angle)
  return np.array([[ co, 0., si, fing_len[1]],
                   [ 0., 1., 0., 0.         ],
                   [-si, 0., co, 0          ],
                   [ 0., 0., 0., 1.         ]])

def calc_frame(joint_angles):
  frms = []
  frms.append(t_joint1(joint_angles[0]))
  frms.append(t_joint2(joint_angles[1]))
  frms.append(t_joint3(joint_angles[2]))
  
  frms.append(t_joint1(joint_angles[3]))
  frms.append(t_joint2(joint_angles[4]))
  frms.append(t_joint3(joint_angles[5]))
  
  frms.append(t_joint1(joint_angles[6]))
  frms.append(t_joint2(joint_angles[7]))
  frms.append(t_joint3(joint_angles[8]))
  return frms

def solve_fk(joint_angles):
  t_joint3_ftip = np.array([[1., 0., 0., fing_len[2]],
                             [0., 1., 0., 0.         ],
                             [0., 0., 0., 0.         ],
                             [0., 0., 0., 1.         ]])

  frms = calc_frame(joint_angles)
  fing0 = t_wrist2fing0 * FRAME(frm = frms[0].dot(frms[1]).dot(frms[2]).dot(t_joint3_ftip))
  fing1 = t_wrist2fing1 * FRAME(frm = frms[3].dot(frms[4]).dot(frms[5]).dot(t_joint3_ftip))
  fing2 = t_wrist2fing2 * FRAME(frm = frms[6].dot(frms[7]).dot(frms[8]).dot(t_joint3_ftip))
  return [fing0, fing1, fing2]
  
  
def calc_jacob(frms, vec_tip = [fing_len[2], 0., 0.,]):
  #initialize
  jacob_pos = np.dot(np.array([[vec_hand[0]],[vec_hand[1]],[vec_hand[2]],[1.0]]),np.ones((1,3)))
  jacob_rot = np.array([[1., 0., 0.],
                        [0., 1., 1.],
                        [0., 0., 0.],
                        [0., 0., 0.]])

  drx = np.array([[ 0., 0., 0., 0.],
                  [ 0., 0.,-1., 0.],
                  [ 0., 1., 0., 0.],
                  [ 0., 0., 0., 0.]])
  
  dry = np.array([[ 0., 0., 1., 0.],
                  [ 0., 0., 0., 0.],
                  [-1., 0., 0., 0.],
                  [ 0., 0., 0., 0.]])

  drz = np.array([[ 0.,-1., 0., 0.],
                  [ 1., 0., 0., 0.],
                  [ 0., 0., 0., 0.],
                  [ 0., 0., 0., 0.]])

  for i, mat in enumerate([[ dry, frms[2]],
                           [ dry, frms[1]],
                           [ drx, frms[0]]]):

    jacob_pos[0:4, 2-i:3-i] = np.dot(mat[0], jacob_pos[0:4, 2-i:3-i])
    jacob_pos = np.dot(mat[1], jacob_pos)

  for i, mat in enumerate([frms[2], frms[1], frms[0]]):
    jacob_rot[0:4, 2-i:3] = np.dot(mat, jacob_rot[0:4, 2-i:3])

  return jacob_pos, jacob_rot

