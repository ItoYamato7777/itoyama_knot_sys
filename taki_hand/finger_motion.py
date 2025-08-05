import numpy as np
import math
from geo import *

class grasping_by_two_fingers():
  """
  Motion generator for grasping motion using two fingers
  """
  def __init__(self, fing_len, base_len, diameter_f_tip = 0.03):
    self.fing_len = fing_len
    self.base_len = base_len
    self.diameter_f_tip = diameter_f_tip
    self.lst_command = np.array(range(-900, 901))
    rad = np.radians(self.lst_command / 10.0)
    sin = np.sin(rad)
    cos = np.cos(rad)
    mat_size = len(self.lst_command)
    
    f1_x =  fing_len[1] * np.dot(cos.reshape((mat_size, 1)), np.ones((1, mat_size))) + fing_len[0]
    f1_y = -fing_len[1] * np.dot(sin.reshape((mat_size, 1)), np.ones((1, mat_size))) + base_len / 2.0
    f2_x =  fing_len[1] * np.dot(np.ones((mat_size, 1)), cos.reshape((1, mat_size))) + fing_len[0]
    f2_y =  fing_len[1] * np.dot(np.ones((mat_size, 1)), sin.reshape((1, mat_size))) - base_len / 2.0

    self.mat_dx = f1_x - f2_x
    self.mat_dy = f1_y - f2_y
    self.mat_x = (f1_x + f2_x)/2.0
    self.mat_y = (f1_y + f2_y)/2.0

  def calc_finger_angles(self, dist, slide, direction=0, opening_angle = 0):
    sin = math.sin(direction)
    cos = math.cos(direction)
    rot_mat = np.array([[cos, -sin],[sin, cos]])
    delta_xy = np.dot(rot_mat, np.array([[slide],[dist]])).reshape(2)
    #print delta_xy
    dx_error = self.mat_dx - delta_xy[0]
    dy_error = self.mat_dy - delta_xy[1]
    error = dx_error ** 2 + dy_error ** 2 + np.abs(self.mat_y) * 0.0001
    f1_i = int(error.argmin() / error.shape[1])
    f2_i = int(error.argmin() % error.shape[1])
    f1_angle_b = self.lst_command[f1_i]
    f2_angle_b = self.lst_command[f2_i]

    x = self.mat_x[f1_i, f2_i]
    y = self.mat_y[f1_i, f2_i]

    deg_direction = math.degrees(direction)
    deg_opening = math.degrees(opening_angle)

    f1_angle_c = round(-f1_angle_b - deg_direction*10 - deg_opening*10)
    f2_angle_c = round(-f2_angle_b + deg_direction*10 - deg_opening*10)

    f_tip_xy = np.array([x,y]) + np.dot(rot_mat, np.array([[self.fing_len[2]],[0]])).reshape(2)

    ## compute the transform matrix from finger's base to finger tip
    f_base2tip = FRAME(xyzrpy = [f_tip_xy[0], f_tip_xy[1], 0, 0 ,0 ,direction])
    return f1_angle_b, f1_angle_c, f2_angle_b, f2_angle_c, f_base2tip

  def calc_finger_angles2(self, dist, slide, direction=0, dist2=0):
    """
    """
    th1 = math.atan2(self.diameter_f_tip, 2 * self.fing_len[2])
    th2 = math.acos((dist-dist2)/(math.sqrt(4*(self.fing_len[2]**2) + self.diameter_f_tip**2)))
    opening_angle = math.radians(-90) + th1 + th2
    print math.degrees(opening_angle)
    return self.calc_finger_angles(dist, slide, direction, opening_angle)
  
  def calc_state(self, f1_angle_b, f1_angle_c, f2_angle_b, f2_angle_c):
    
    f1_angle_b = math.radians(f1_angle_b/10.)
    f2_angle_b = math.radians(f2_angle_b/10.)
    f1_angle_c = math.radians(f1_angle_c/10.)
    f2_angle_c = math.radians(f2_angle_c/10.)
    
    direction = (- f1_angle_b - f1_angle_c + f2_angle_b + f2_angle_c)/2.0
    opening_angle = (- f1_angle_b - f1_angle_c - f2_angle_b - f2_angle_c)/2.0

    print direction
    print opening_angle

    ## making rotation matrix finger_tip2finger_base
    sin = math.sin(direction)
    cos = math.cos(direction)
    rot_mat = np.array([[cos, -sin],[sin, cos]]).T
    
    dx = self.fing_len[1] * ( math.cos(f1_angle_b) - math.cos(f2_angle_b))
    dy = self.fing_len[1] * (-math.sin(f1_angle_b) - math.sin(f2_angle_b)) + self.base_len
    
    slide_dist = np.dot(rot_mat, np.array([[dx],[dy]])).reshape(2)
    print slide_dist
    return slide_dist[1], slide_dist[0], direction, opening_angle


class one_finger_motion():
  def __init__(self, fing_len):
    self.fing_len = fing_len

  def solve_angles_yz(self, y, z, th=0):
    if z > 0:
      th1 = math.atan2(-y, z)
      th2 = -math.asin(math.sqrt(y**2+z**2)/self.fing_len[1])
    else:
      th1 = math.atan2(y, -z)
      th2 = math.asin(math.sqrt(y**2+z**2)/self.fing_len[1])
    th3 = -th2+th
    print math.degrees(th3)
    return rad2command([th1,th2,th3])

  
def rad2command(rad):
  return np.round(np.degrees(rad)*10).astype(np.int).tolist()
