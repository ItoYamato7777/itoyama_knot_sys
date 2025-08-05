import numpy as np
import math
from numpy.linalg import svd
from math import sin, cos, radians
from cv2 import Rodrigues

def tYaw(angle):
  si = sin(angle)
  co = cos(angle)
  return np.array([[co, -si, 0., 0.],
                   [si,  co, 0., 0.],
                   [0.,  0., 1., 0.],
                   [0.,  0., 0., 1.]])

sin15deg = sin(radians(15))
cos15deg = cos(radians(15))
tB2CY = tYaw
tCY2NY = tYaw
def tNY2NP(angle):
  si = sin(angle)
  co = cos(angle)
  return np.array([[ co, 0., si, 0.    ],
                   [ 0., 1., 0., 0.    ],
                   [-si, 0., co, 0.5695],
                   [ 0., 0., 0., 1.    ]])

tCY2RSB = np.array([[1,       0.,        0.,  0.    ],
                    [0, cos15deg, -sin15deg, -0.145 ],
                    [0, sin15deg,  cos15deg,  0.3703],
                    [0,       0.,        0.,  1.    ]])
tCY2LSB = np.array([[1,       0.,        0.,  0.    ],
                    [0, cos15deg,  sin15deg,  0.145 ],
                    [0,-sin15deg,  cos15deg,  0.3703],
                    [0,       0.,        0.,  1.    ]])
tSB2SY = tYaw
def tRSY2RSP(angle):
  si = sin(angle)
  co = cos(angle)
  return np.array([[ co, 0., si,  0.   ],
                   [ 0., 1., 0., -0.095],
                   [-si, 0., co,  0.   ],
                   [ 0., 0., 0.,  1.   ]])
def tLSY2LSP(angle):
  si = sin(angle)
  co = cos(angle)
  return np.array([[ co, 0., si,  0.   ],
                   [ 0., 1., 0.,  0.095],
                   [-si, 0., co,  0.   ],
                   [ 0., 0., 0.,  1.   ]])
def tSP2EP(angle):
  si = sin(angle)
  co = cos(angle)
  return np.array([[ co, 0., si,  0.   ],
                   [ 0., 1., 0.,  0.   ],
                   [-si, 0., co, -0.250],
                   [ 0., 0., 0.,  1.   ]])
def tEP2WY(angle): 
  si = sin(angle)
  co = cos(angle)
  return np.array([[co, -si,  0., -0.03 ],
                   [si,  co,  0.,  0.   ],
                   [0.,  0.,  1., -0.235],
                   [0.,  0.,  0.,  1.   ]])
def tWY2WP(angle): 
  si = sin(angle)
  co = cos(angle)
  return np.array([[ co, 0., si,  0.  ],
                   [ 0., 1., 0.,  0.  ],
                   [-si, 0., co,  0.  ],
                   [ 0., 0., 0.,  1.  ]])
def tWP2WR(angle):
  si = sin(angle)
  co = cos(angle)
  return np.array([[1., 0.,  0., -0.04],
                   [0., co, -si,  0.  ],
                   [0., si,  co, -0.09],
                   [0., 0.,  0.,  1.  ]])

def calc_frame(joint_angle):
  frms = []
  #body#
  frms.append( tB2CY(    joint_angle[0] ))
  #neck
  frms.append( tCY2NY(   joint_angle[1] ))
  frms.append( tNY2NP(   joint_angle[2] ))
  #armR
  frms.append( np.dot(tCY2RSB, tSB2SY(joint_angle[3] )))
  frms.append( tRSY2RSP( joint_angle[4] ))
  frms.append( tSP2EP(   joint_angle[5] ))
  frms.append( tEP2WY(   joint_angle[6] ))
  frms.append( tWY2WP(   joint_angle[7] ))
  frms.append( tWP2WR(   joint_angle[8] ))
  #armL
  frms.append( np.dot(tCY2LSB, tSB2SY(joint_angle[9] )))
  frms.append( tLSY2LSP( joint_angle[10] ))
  frms.append( tSP2EP(   joint_angle[11] ))
  frms.append( tEP2WY(   joint_angle[12] ))
  frms.append( tWY2WP(   joint_angle[13] ))
  frms.append( tWP2WR(   joint_angle[14] ))
  return frms

def solve_fk(joint_angle):
  frms = calc_frame(joint_angle)
  dic={}
  dic["Neck"]  = frms[0].dot(frms[1]).dot(frms[2])
  dic["R_arm"] = frms[0].dot(frms[3]).dot(frms[ 4]).dot(frms[ 5]).dot(frms[ 6]).dot(frms[ 7]).dot(frms[ 8])
  dic["L_arm"] = frms[0].dot(frms[9]).dot(frms[10]).dot(frms[11]).dot(frms[12]).dot(frms[13]).dot(frms[14])
  return dic

def calc_jacob(arm_frms, vec_hand = [0.,0.,0.,]):
  #initialize
  jacob_pos = np.dot(np.array([[vec_hand[0]],[vec_hand[1]],[vec_hand[2]],[1.0]]),np.ones((1,7)))
  jacob_rot = np.array([[0., 0., 0., 0., 0., 0., 1.],
                        [0., 0., 1., 1., 0., 1., 0.],
                        [1., 1., 0., 0., 1., 0., 0.],
                        [0., 0., 0., 0., 0., 0., 0.]])

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

  for i, mat in enumerate([[ drx, arm_frms[6] ],
                           [ dry, arm_frms[5] ],
                           [ drz, arm_frms[4] ],
                           [ dry, arm_frms[3] ],
                           [ dry, arm_frms[2] ],
                           [ drz, arm_frms[1] ],
                           [ drz, arm_frms[0] ]]):

    jacob_pos[0:4, 6-i:7-i] = np.dot(mat[0], jacob_pos[0:4, 6-i:7-i])
    jacob_pos = np.dot(mat[1], jacob_pos)

  for i, mat in enumerate([arm_frms[5], arm_frms[4], arm_frms[3], arm_frms[2], arm_frms[1], arm_frms[0]]):
    jacob_rot[0:4, 6-i:7] = np.dot(mat, jacob_rot[0:4, 6-i:7])

  return jacob_pos, jacob_rot




def calc_delta_theta(frms,
                     delta_R = None,
                     delta_L = None,
                     vec_r_hand = [0,0,0],
                     vec_l_hand = [0,0,0],
                     delta_CY = None):
  rlim = 0.001
  jacobian = np.zeros((13,13))
  #initializing
  delta = np.zeros((13,1))

  if not delta_CY is None:
    jacobian[12, 0] = 1.
    delta[12,0] = delta_CY

  if not delta_R is None:
    r_jacob_pos, r_jacob_rot = calc_jacob([frms[i] for i in [0,3,4,5,6,7,8]], vec_r_hand)
    jacobian[0: 3 , 0: 7] = r_jacob_pos[0:3, 0:7]
    jacobian[3: 6 , 0: 7] = r_jacob_rot[0:3, 0:7]
    delta[0:6, 0] = delta_R
   
  if not delta_L is None:
    l_jacob_pos, l_jacob_rot = calc_jacob([frms[i] for i in [0,9,10,11,12,13,14]], vec_r_hand)
    jacobian[6: 9 , 0: 1] = l_jacob_pos[0:3, 0:1] #CY
    jacobian[9:12 , 0: 1] = l_jacob_rot[0:3, 0:1] #CY
    jacobian[6: 9 , 7:13] = l_jacob_pos[0:3, 1:7]
    jacobian[9:12 , 7:13] = l_jacob_rot[0:3, 1:7]
    delta[6:12, 0] = delta_L

  ## calc inv jacobian
  u_mat, w_vec, v_trn = svd(jacobian, full_matrices = 0, compute_uv = 1)
  w_max = max(w_vec)
  w_min = w_max * rlim
  w_inv = np.zeros_like(w_vec)
  for i, w in enumerate(w_vec):
    if w > w_min:
      w_inv[i] = 1. / w
  inv_jacobi = np.dot( np.dot( v_trn.T, np.diag(w_inv)), u_mat.T)
  delta_theta =  np.dot(inv_jacobi,  delta).reshape(13)
  return delta_theta


def solve_ik(joint_angle,
             target_R = None,
             target_L = None,
             t_wrist2hand_R = np.eye(4),
             t_wrist2hand_L = np.eye(4),
             target_CY = None,
             return_continuous_path = True,
             cycle_max = 1000,
             th_lim = 0.05):
  """
  joint_angle: initial joint angle (solving ik from this posture)
  target_R, target_L: target posture of hand (not wrist)
  t_wrist2hand_R, t_wrist2hand_L: transformation from wrist to hand
  target_CY: target angle of Chest Yaw
  return_continurous_path: 
  True -> return intermidiate joint angle in order to move hand linearly.
  False -> return final joint angle
  cycle_max: max number of solving ik
  th_lim -> max(abs(step[n] - step[n+1]))
  """
  
  joint_angle = np.array(joint_angle, dtype = np.float)[0:15]
  start_angle = np.copy(joint_angle)
  
  lim = 0.00001
#  th_lim = 0.05
  step = 0.3
  if return_continuous_path:
    angle_list = [joint_angle.tolist()]

  for cycle in xrange(cycle_max):
    #calc frames
    frms = calc_frame(joint_angle)

    #calc distance from current pose to target pose
    if target_R is None:
      delta_R = None
      dist_R = 0.0
    else:
      current_R = frms[0].dot(frms[3]).dot(frms[ 4]).dot(frms[ 5]).dot(frms[ 6]).dot(frms[ 7]).dot(frms[ 8]).dot(t_wrist2hand_R)
      diff_pos_R = target_R[0:3,3] - current_R[0:3,3]
      diff_rot_R = Rodrigues(np.dot(target_R[0:3,0:3], current_R[0:3,0:3].T))[0].reshape(3)
      dist_R = math.sqrt(np.sum(np.array([diff_pos_R[0], diff_pos_R[1], diff_pos_R[2], diff_rot_R[0], diff_rot_R[1], diff_rot_R[2]]) **2 ))
    if target_L is None:
      delta_L = None
      dist_L = 0.0
    else:
      current_L = frms[0].dot(frms[9]).dot(frms[10]).dot(frms[11]).dot(frms[12]).dot(frms[13]).dot(frms[14]).dot(t_wrist2hand_L)
      diff_pos_L = target_L[0:3,3] - current_L[0:3,3]
      diff_rot_L = Rodrigues(np.dot(target_L[0:3,0:3], current_L[0:3,0:3].T))[0].reshape(3)
      dist_L = math.sqrt(np.sum(np.array([diff_pos_L[0], diff_pos_L[1], diff_pos_L[2], diff_rot_L[0], diff_rot_L[2], diff_rot_L[2]]) **2 ))
    if target_CY is None:
      delta_CY = None
      diff_CY = 0
    else:
      diff_CY = target_CY - joint_angle[0]

    #calc distance from current posture to target one.
    dist = max(dist_R, dist_L, abs(diff_CY))

    #loop end condition: dist is small enough.
    if dist < lim:
      if return_continuous_path:
        return angle_list
      else:
        return joint_angle
      
    #if dist is larger than step, resize delta.
    #  if (dist > step): k = step/dist
    #  else: k = 1
    k = min(1, step/dist)
    if not target_R is None:
      diff_pos_R = k * diff_pos_R
      diff_rot_R = k * diff_rot_R
#      diff_rot_vec_R = diff_rot_R[0] * diff_rot_R[1]
      delta_R = np.array([diff_pos_R[0], diff_pos_R[1], diff_pos_R[2], diff_rot_R[0], diff_rot_R[1], diff_rot_R[2]])
      
    if not target_L is None:
      diff_pos_L = k * diff_pos_L
      diff_rot_L = k * diff_rot_L
#      diff_rot_vec_L = diff_rot_L[0] * diff_rot_L[1]
      delta_L = np.array([diff_pos_L[0], diff_pos_L[1], diff_pos_L[2], diff_rot_L[0], diff_rot_L[1], diff_rot_L[2]])
      
    if not target_CY is None:
      delta_CY = k * diff_CY

    #calc delta theta for each joints
    delta_theta = calc_delta_theta(frms, delta_R, delta_L, t_wrist2hand_R[0:3,3], t_wrist2hand_L[0:3,3], delta_CY)
    delta_theta_max = np.abs(delta_theta).max()
    if delta_theta_max > th_lim:
      delta_theta = (th_lim / delta_theta_max) * delta_theta
    #update joint angles    
    joint_angle[0] += delta_theta[0]
    joint_angle[3:15] = joint_angle[3:15] + delta_theta[1:13]
    #limitation of joint_angle
    tf = np.logical_or((joint_angle > angle_max),(joint_angle < angle_min))
    if True in tf:
      if return_continuous_path:
        if target_CY is None:
          #solve_ik by point by point
          tmp = solve_ik(joint_angle, target_R, target_L, t_wrist2hand_R, t_wrist2hand_L, None, False, cycle_max - cycle, 0.5)
          if tmp is None:
            return None
          else:
            #print "retry"
            return solve_ik(start_angle, target_R, target_L, t_wrist2hand_R, t_wrist2hand_L, tmp[0], True, cycle_max, th_lim)
        else:
          #print "cannot reach the position"
          return None
      else:
        joint_angle[tf] = angle_center[tf]

    if return_continuous_path:
      angle_list.append(joint_angle.tolist())

  #print "cannot reach the position"
  return None

def devide_by_part(joint_angle):
  lst = list(joint_angle)
  return [lst[0:3], lst[3:9], lst[9:15],[],[]]


angle_max = np.radians([ 163,  70,   70,
                         88,   60,    0,  105,  100,  163,
                         88,   60,    0,  165,  100,  163])

angle_min = np.radians([-163, -70, -20,
                        -88, -140, -158, -165, -100, -163,
                        -88, -140, -158, -105, -100, -163])

angle_center = (angle_max + angle_min)/2
JointNames = ["CY", "NY", "NP",
              "RSY", "RSP", "REP", "RWY", "RWP", "RWR",
              "LSY", "LSP", "LEP", "LWY", "LWP", "LWR"]
