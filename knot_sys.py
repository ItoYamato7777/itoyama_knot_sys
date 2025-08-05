import os
import sys
import time
import math
import numpy as np
import cv2
from math import pi
from math import radians

def read_txt(fileName):
  f = open(fileName)
  return f.read()

path_IncludeFiles = read_txt("./includefiles.txt").replace("\n", "")
sys.path.append(path_IncludeFiles)

## Head ENSENSO Camera ##
sys.path.append( path_IncludeFiles + "/ensenso_handle")

import ensenso_handle

## for HIRO ##
sys.path.append( path_IncludeFiles + "/hiro_handle")
from hiro_handle import *

import kinematics_hiro as k_hiro

## for Robot Hand ##
import hand_sys as hand
import kinematics_hand as k_hand
from hand_sys import r_hand, l_hand, t_hand

## Vision ##
import vision

## PointCloud ##
import pointcloud

## Geometry ##
from geo import VECTOR, MATRIX, FRAME

## The others ##
import file_io

sys.path.append( path_IncludeFiles + "/HiroUrg/hiro_urg_handle")
import hiro_urg_handle

# Run Embryonic_rtc
comp = ensenso_handle.run_embryonic_rtc()

argv = sys.argv + ["-ORBgiopMaxMsgSize", "104857600"]
env = RtmEnv(argv, ["localhost:2809", "hiro022"])
hiro = HiroEnv(argv, "hiro022", env.orb)
hiro.init()


time.sleep(2)
for ns in env.name_space:
  env.name_space[ns].list_obj()
  
sys.path.append( path_IncludeFiles + "/hiroControlRTC/myHRP_handle")
import myHRP_handle as myHRP


print("Loading Ensenso")
ensenso = ensenso_handle.C_ensenso(ensenso_handle.env, comp)

print("Loading URG")
urg = hiro_urg_handle.C_hiro_urg(env, comp)


## Load Parameters ##
color_mtx = np.load( path_IncludeFiles + "/params/color_mtx.npy")
color_dist = np.load( path_IncludeFiles + "/params/color_dist.npy")
ir_mtx = np.load( path_IncludeFiles + "/params/ir_mtx.npy")
ir_dist =np.load( path_IncludeFiles + "/params/ir_dist.npy")

tNeck2Depth = FRAME(frm = np.load( path_IncludeFiles + "/params/neck2ir.npy"))
tNeck2Color = FRAME(frm = np.load( path_IncludeFiles + "/params/neck2color.npy"))

tWrist2Hand = FRAME(xyzrpy = [-0.077, 0, 0, pi, 0, pi])
tHand2Wrist = FRAME(xyzrpy = [-0.077, 0, 0, pi, 0, pi])


def to_ready():
  ret = raw_input("Calibrate joints? [Y]/n : ")
  if ret.lower() == "n":
    hiro.setupLogger()
    init_force_sensor(True)
    
  elif ret.lower() == "y" or ret == "":
    hand.activate()
    hiro.calibrateJoint()
    hiro.servoOn()
    init_force_sensor(False)
#    mv_time = go_initial_position("LR", True, 0)
    hiro.setJointAnglesDeg([[0, 0, 45],[],[],[],[]], 2, False)
    time.sleep(2.1)

def reset_hiro():
  hiro.goOffPose()
    
def suspend():
  raw_input("Hiro is going to move.")
  go_initial_position()
  hand.deactivate()
  hiro.goOffPose()

  ret = raw_input("Shutdown? y/[N] : ")
  if ret.lower() == "y":
    hiro.shutdown()
  
## functions for hiro
# arm = "L" or "R"

def _svc_selector(arm):
  try:
    if arm.lower() == "l":
      return hiro.armL_svc
    elif arm.lower() == "r":
      return hiro.armR_svc
    else:
      raise ValueError
  except:
    raise ValueError

def move_arm(arm, frame, rate = 8, wait = True):
  #arm = "L" or "R"
  #frame = geo.FRAME
  svc = _svc_selector(arm)
  x,y,z,roll,pitch,yaw = frame.xyzrpy()
  return hiro.move(svc, x, y, z, roll, pitch, yaw, rate, wait)

def get_arm_position(arm):
  svc = _svc_selector(arm)
  x,y,z,roll, pitch, yaw = hiro.getCurrentConfiguration(svc)
  return FRAME(xyzrpy = [x, y, z, roll, pitch, yaw])

def get_robot_posture():
  angles = hiro.getJointAnglesRad()
  dic = k_hiro.solve_fk(angles)
  dic["Neck"] = FRAME(frm = dic["Neck"])
  dic["L_arm"] = FRAME(frm = dic["L_arm"])
  dic["R_arm"] = FRAME(frm = dic["R_arm"])
  return dic

def solve_ik(joint_angle,
             target_R = None,
             target_L = None,
             t_wrist2hand_R = np.eye(4),
             t_wrist2hand_L = np.eye(4),
             target_CY = None,
             return_continuous_path = True,
             cycle_max = 1000,
             th_lim = 0.05):

  if isinstance(target_R, FRAME):
    target_R = target_R.toarray()
  if isinstance(target_L, FRAME):
    target_L = target_L.toarray()
  if isinstance(t_wrist2hand_R, FRAME):
    t_wrist2hand_R = t_wrist2hand_R.toarray()
  if isinstance(t_wrist2hand_L, FRAME):
    t_wrist2hand_L = t_wrist2hand_L.toarray()

  return k_hiro.solve_ik(joint_angle,
                         target_R,
                         target_L,
                         t_wrist2hand_R,
                         t_wrist2hand_L,
                         target_CY,
                         return_continuous_path,
                         cycle_max,
                         th_lim)


def move_arm_13dof(target_R = None, target_L = None, angular_vel = 0.25, wait = True, target_CY = None):
  #solve ik 13 DOFs
  wrist2hand = np.eye(4)
  ang1st = hiro.getJointAnglesRad()
  angles = solve_ik(ang1st, target_R, target_L, wrist2hand, wrist2hand, target_CY, False, th_lim = 0.5)
  if angles is None:
    return -1
  
  max_diff = np.max(np.abs(np.array(ang1st)[0:15] - np.array(angles[0:15])))
  move_time = max(1, max_diff/angular_vel) #the move_time must be longer than 1 sec
  hiro.setJointAnglesRad(k_hiro.devide_by_part(angles), move_time, wait)
  return move_time


def move_arm2(arm, frame, angular_vel= 0.2, wait = True):
  currentCY = hiro.getJointAnglesRad()[0]
  
  if arm.lower() == "l":
    posR = get_arm_position("r")
    posL = frame
  if arm.lower() == "r":
    posR = frame
    posL = get_arm_position("l")
    
  ret = move_arm_13dof(posR, posL, angular_vel, wait, currentCY)

  if ret < 0:
    ret = move_arm_13dof(posR, posL, angular_vel, wait, None)
    
  return ret


def go_initial_position(arm = "LR", wait = True, target_CY = None):
  if target_CY is None:
    target_CY = hiro.getJointAnglesRad()[0]
  tBase2Body = FRAME(xyzrpy = [0., 0., 0., 0., 0., target_CY])
  arm = arm.lower()
  move_time = [0] #initialize

  if arm.find("l") >= 0:
    pos_L = tBase2Body * FRAME(xyzrpy = [0.2,  0.4, 0.35, pi, 0, pi])
  else:
    pos_L = None
  if arm.find("r") >= 0:
    pos_R = tBase2Body * FRAME(xyzrpy = [0.2, -0.4, 0.35, pi, 0, pi])
  else:
    pos_R = None
    
  return move_arm_13dof(pos_R, pos_L, 0.25, wait, target_CY)

## tabletop image ##
def get_table_height():
  go_initial_position("RL", True, 0)
  pointmap = ensenso.capture("pointMap")[0] * 0.001
  base2neck = get_robot_posture()["Neck"]
  # transform point-map into base coordinate
  pointmap = pointcloud.coordinate_transform(base2neck * tNeck2Depth, pointmap)
  # calc height of table
  z = pointmap[:,:,2]
  table_height = np.median(z[np.logical_not(np.isnan(z))])
  pointcloud.plot_points(pointmap)
  return table_height
  
def capture_tabletop(table_height):
  """
  capture overview of tabletop
  On the image, 1 px is equal to 1 mm.
  And, A point on the image is converted into the robot coodinate by a following equation
  im[y, x] -> ((y+200)/1000 , (x-650)/1000)
  The "table_height" is the height from robot base coordinate. 
  When the displayed height is 88 cm; it is saved on memory 1,
  table_height is about 0.0045 [m]
  """

  ## initialize ##
  go_initial_position("RL", True, 0)
  im_size = (1301, 690)
  ## capture area 1 ##
  x_robot = np.array([0.25, 0.25, 0.45, 0.45])
  y_robot = np.array([-0.1,  0.1, -0.1,  0.1])
  z = [table_height for i in range(4)]  
  xyz = np.array([x_robot,y_robot,z])

  area_list = []
  img_list = []
  for yaw in range(-30, 31, 10):
    #capture
    mat = MATRIX(c = radians(yaw)).toarray()
    xyz_robot = np.dot(mat, xyz)
    pos_to = np.array(robot_xy2tabletop_xy(xyz_robot[0], xyz_robot[1])).T
    vec_from = xyz_robot.T
    hiro.setJointAnglesDeg([[yaw,0,45],[],[],[],[]],2, True)
    time.sleep(0.5)
    origin = ensenso.capture("color")[0]
    white_img = np.ones_like(origin) * 255
    base2cam = get_robot_posture()["Neck"] * tNeck2Color
    cam2base = -base2cam
    tvec = cam2base.vec.toarray()
    rvec = cv2.Rodrigues(cam2base.mat.toarray())[0]
    p_mat = vision.get_perspective_transform(rvec, tvec, color_mtx, color_dist, vec_from, pos_to)
    pers_img = vision.warp_perspective(origin, p_mat, im_size)
    tf = vision.warp_perspective(white_img, p_mat, im_size) == 255
    pers_img[np.where(tf == False)] = 0
    img_list.append(pers_img)
    area_list.append(tf * 1)
    cv2.imshow("", pers_img)
    cv2.waitKey(10)

  ## mix images ##
  cap_time = np.zeros((im_size[1], im_size[0], 3), dtype = np.uint)
  for area in area_list:
    cap_time = cap_time + area
  cap_time[np.where(cap_time < 1)] = 1 #not to devide 0
  ret_img = np.zeros((im_size[1], im_size[0], 3), dtype = np.uint)
  for im in img_list:
    ret_img = ret_img + im

  ret_img = (ret_img/cap_time).astype(np.uint8)
  cv2.imshow("retimg", ret_img)

  hiro.setJointAnglesDeg([[0,0,45],[],[],[],[]],2, False)
  cv2.waitKey(2100)
  cv2.destroyAllWindows()
  return ret_img

def tabletop_xy2robot_xy(img_x, img_y):
  robot_x = (img_y + 200) / 1000.0
  robot_y = (img_x - 650) / 1000.0
  return robot_x, robot_y

def robot_xy2tabletop_xy(robot_x, robot_y):
  img_x = (robot_y * 1000.0) + 650
  img_y = (robot_x * 1000.0) - 200
  return img_x, img_y

def init_force_sensor(load_data = False):
  if load_data:
    param_R = np.load(path_IncludeFiles + "/params/force_R.npy")
    param_L = np.load(path_IncludeFiles + "/params/force_L.npy")

    print param_R
    print param_L
    
  else:
    vel = 0.3
    w2cm = FRAME(xyzrpy = [-0.1, 0.0, 0.0, pi, 0, pi]) #wrist to center of mass
    r_xyz = [0.3, -0.4, 0.35]
    l_xyz = [0.3,  0.4, 0.35]
    roll1 =  [radians(45), 0, 0]
    roll2 =  [radians(-45), 0, 0]
    pitch1 = [0, radians(45), 0]
    pitch2 = [0, radians(-45), 0]
    yaw1 =   [0, 0, radians(45)]
    yaw2 =   [0, 0, radians(-45)]
    
    posR = FRAME(xyzrpy = r_xyz + [0,0,0] ) * -w2cm
    posL = FRAME(xyzrpy = l_xyz + [0,0,0] ) * -w2cm
    move_arm_13dof(posR, posL, vel, True, 0.0)
    myHRP.start_log()
    
    posR = FRAME(xyzrpy = r_xyz + roll1 ) * -w2cm
    posL = FRAME(xyzrpy = l_xyz + roll2 ) * -w2cm
    move_arm_13dof(posR, posL, vel, True, 0.0)
    
    posR = FRAME(xyzrpy = r_xyz + pitch1 ) * -w2cm
    posL = FRAME(xyzrpy = l_xyz + pitch1 ) * -w2cm
    move_arm_13dof(posR, posL, vel, True, 0.0)
    
    posR = FRAME(xyzrpy = r_xyz + yaw1 ) * -w2cm
    posL = FRAME(xyzrpy = l_xyz + yaw2 ) * -w2cm
    move_arm_13dof(posR, posL, vel, True, 0.0)
    
    posR = FRAME(xyzrpy = r_xyz + pitch2 ) * -w2cm
    posL = FRAME(xyzrpy = l_xyz + pitch2 ) * -w2cm
    move_arm_13dof(posR, posL, vel, True, 0.0)
    
    posR = FRAME(xyzrpy = r_xyz + yaw2 ) * -w2cm
    posL = FRAME(xyzrpy = l_xyz + yaw1 ) * -w2cm
    move_arm_13dof(posR, posL, vel, True, 0.0)
    
    posR = FRAME(xyzrpy = r_xyz + roll2 ) * -w2cm
    posL = FRAME(xyzrpy = l_xyz + roll1 ) * -w2cm
    move_arm_13dof(posR, posL, vel, True, 0.0)

    posR = FRAME(xyzrpy = r_xyz + [0,0,0] ) * -w2cm
    posL = FRAME(xyzrpy = l_xyz + [0,0,0] ) * -w2cm
    move_arm_13dof(posR, posL, vel, True, 0.0)
    log = myHRP.return_log()
    
    go_initial_position("RL", True, 0)
    param_R, param_L = myHRP.calc_force_control_init_param(log)
    print param_R
    print param_L
    np.save(path_IncludeFiles + "/params/force_R.npy", param_R)
    np.save(path_IncludeFiles + "/params/force_L.npy", param_L)
    
  myHRP.set_force_control_param(param_R, param_L)
  
# start up
to_ready()

