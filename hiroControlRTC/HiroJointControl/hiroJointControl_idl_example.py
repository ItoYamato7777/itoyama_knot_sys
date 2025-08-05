#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file hiroJointControl_idl_examplefile.py
 @brief Python example implementations generated from hiroJointControl.idl
 @date $Date$


"""

import omniORB
from omniORB import CORBA, PortableServer
import myHRP, myHRP__POA
import time
import numpy as np
import math

from kinematics_hiro import *
from geo import VECTOR, MATRIX, FRAME

class MyHiroControlService_i (myHRP__POA.MyHiroControlService):
  """
  @class MyHiroControlService_i
  Example class implementing IDL interface myHRP.MyHiroControlService
  """

  def __init__(self):
    """
    @brief standard constructor
    Initialise member variables here
    """
    #for logging
    self.logging = False
    self.logdata = np.zeros((200*60,15+6+6+1), dtype = np.float64)
    self.log_i = 0
    
    self.is_busy = False

    #for position control
    self.current_jointangles = None
    self.frm_current = None
    self.renew_cmd = False
    self.target_jointangles = []

    #for force control
    #to remove offset
    self.lepR_params = np.zeros((10,1))
    self.lepL_params = np.zeros((10,1))

    #force control axis (from base)
    # f(t)_ctrl_mat_R(L) = r_base2caxis.dot( diag(K) ).dot( r_caxis2base )
    # F_... is for force and torque
    # f_... is for force
    # t_... is for torque
    self.f_ctrl_mat_R = np.zeros((3,3))
    self.t_ctrl_mat_R = np.zeros((3,3))
    self.f_ctrl_mat_L = np.zeros((3,3))
    self.t_ctrl_mat_L = np.zeros((3,3))
    self.F_ctrl_target_R = np.zeros((3,2))
    self.F_ctrl_target_L = np.zeros((3,2))
    self.F_ctrl_origin_R = None
    self.F_ctrl_origin_L = None
    self.F_ctrl_offset_R = np.eye(4)
    self.F_ctrl_offset_L = np.eye(4)
    self.F_ctrl_diff_R = np.zeros((3,2))
    self.F_ctrl_diff_L = np.zeros((3,2))
    self.f_is_ctrl_mat_R = np.zeros((3,3))
    self.t_is_ctrl_mat_R = np.zeros((3,3))
    self.f_is_ctrl_mat_L = np.zeros((3,3))
    self.t_is_ctrl_mat_L = np.zeros((3,3))
    

  def reset_param(self):
    self.renew_cmd = False
    self.target_jointangles = []
    self.f_ctrl_mat_R = np.zeros((3,3))
    self.t_ctrl_mat_R = np.zeros((3,3))
    self.f_ctrl_mat_L = np.zeros((3,3))
    self.t_ctrl_mat_L = np.zeros((3,3))
    self.F_ctrl_target_R = np.zeros((3,2))
    self.F_ctrl_target_L = np.zeros((3,2))
    self.F_ctrl_origin_R = None
    self.F_ctrl_origin_L = None
    self.F_ctrl_offset_R = np.eye(4)
    self.F_ctrl_offset_L = np.eye(4)
    self.f_is_ctrl_mat_R = np.zeros((3,3))
    self.t_is_ctrl_mat_R = np.zeros((3,3))
    self.f_is_ctrl_mat_L = np.zeros((3,3))
    self.t_is_ctrl_mat_L = np.zeros((3,3))
    self.is_busy = False

    
  def busy_wait(self):
    while self.is_busy:
      time.sleep(0.0001) #0.1ms

  def move_wait(self):
    while len(self.target_jointangles)>0:
      time.sleep(0.005)
      
  # void set_joint_angles(in KeyPose pose, in double speed, out double ttm)
  def set_joint_angles(self, pose, speed):
    if self.current_jointangles is None:
      return -1.0
    diff_angles = np.zeros(15) #initialze

    if len(pose[0]) == 3:
      diff_angles[0:3]  = np.array(pose[0]) - self.current_jointangles[0:3] 
    if len(pose[1]) == 6:
      diff_angles[3:9]  = np.array(pose[1]) - self.current_jointangles[3:9] 
    if len(pose[2]) == 6:
      diff_angles[9:15] = np.array(pose[2]) - self.current_jointangles[9:15]
      
    step = int(math.ceil(np.abs(diff_angles).max() / speed * 200) + 1)
    diff = diff_angles * np.dot(np.linspace(0,1,step).reshape((step,1)), np.ones((1,15)))
    angles = self.current_jointangles + diff

    # write
    self.busy_wait()
    self.is_busy = True
    self.renew_cmd = True
    self.target_jointangles = angles.tolist()
    self.F_ctrl_origin_R = np.copy(self.frm_current["R_arm"])
    self.F_ctrl_origin_L = np.copy(self.frm_current["L_arm"])
    self.F_ctrl_offset_R = np.eye(4)
    self.F_ctrl_offset_L = np.eye(4)

    self.is_busy = False
    ttm = step * (1.0 / 200.0) + 0.05
    return ttm
  # *** Implement me
  # Must return: ttm

  def solve_ik(self,
               joint_angle,
               target_R = None,
               target_L = None,
               t_wrist2hand_R = np.eye(4),
               t_wrist2hand_L = np.eye(4),
               target_CY = None,
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
    step = 0.3
    while True:
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
        return True
      
      #if dist is larger than step, resize delta.
      #  if (dist > step): k = step/dist
      #  else: k = 1
      k = min(1, step/dist)
      if not target_R is None:
        diff_pos_R = k * diff_pos_R
        diff_rot_R = k * diff_rot_R
        delta_R = np.array([diff_pos_R[0], diff_pos_R[1], diff_pos_R[2], diff_rot_R[0], diff_rot_R[1], diff_rot_R[2]])
      
      if not target_L is None:
        diff_pos_L = k * diff_pos_L
        diff_rot_L = k * diff_rot_L
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
        print "joint angle is out of range."
        return False
      if self.renew_cmd == False:
        print "renew_cmd become false."
        return False
      self.busy_wait()
      self.is_busy = True
      self.target_jointangles.append(joint_angle.tolist())
      self.is_busy = False
    #print "cannot reach the position"
    return False

  
  # void move_arm_linearly(in params posR, in params posL, in params wrist2handR, in params wrist2handL, in params CY, in double speed, out double ttm)
  def move_arm_linearly(self, posR, posL, wrist2handR, wrist2handL, CY, speed):
    if len(posR) == 6:
      posR = FRAME(xyzrpy = posR).toarray()
    else:
      posR = None
      
    if len(posL) == 6:
      posL = FRAME(xyzrpy = posL).toarray()
    else:
      posL = None
      
    if len(wrist2handR) == 6:
      wrist2handR = FRAME(xyzrpy = wrist2handR).toarray()
    else:
      wrist2handR = np.zeros((4,4))
      
    if len(wrist2handL) == 6:
      wrist2handL = FRAME(xyzrpy = wrist2handL).toarray()
    else:
      wrist2handL = np.zeros((4,4))

    if len(CY) == 1:
      CY = CY[0]
    else:
      CY = None

    step = speed/200.

    
    self.busy_wait()
    self.is_busy = True
    if self.current_jointangles is None:
      print "current_jointangles is None"
      return -2
    else:
      start_angle = np.copy(self.current_jointangles)
    self.target_jointangles = []
    self.F_ctrl_origin_R = np.copy(self.frm_current["R_arm"])
    self.F_ctrl_origin_L = np.copy(self.frm_current["L_arm"])
    self.F_ctrl_offset_R = np.eye(4)
    self.F_ctrl_offset_L = np.eye(4)
    self.renew_cmd = True
    self.is_busy = False
    
    ret = self.solve_ik(start_angle, posR, posL, wrist2handR, wrist2handL, CY, step)

    if ret is False:
      return -1.0
    
    else:
#      self.busy_wait()
#      self.is_busy = True
#      self.target_jointangles = ret
#      self.is_busy = False
      tm = len(self.target_jointangles)/200.
      return tm
  # *** Implement me
  # Must return: ttm

  # void waitPositionControl(out boolean ret)
  def waitPositionControl(self):
    while len(self.target_jointangles)>0:
      time.sleep(0.005)
    time.sleep(0.05)
    return True
  # *** Implement me
  # Must return: ret

  # void waitForceControl(in params threshold)
  def waitForceControl(self, threshold):
    ## threshold:
    #[ force_R,     force_L,
    #  torque_R,    torque_L,
    #  position_R,  position_L,
    #  rotate_R,    rotate_L]
    while np.sum(self.f_is_ctrl_mat_R + self.f_is_ctrl_mat_L + 
                 self.t_is_ctrl_mat_R + self.t_is_ctrl_mat_L ) > 0: # while force control is ON:
      
      f_diff_R = np.sqrt(np.sum(np.dot(self.f_is_ctrl_mat_R, self.F_ctrl_diff_R[:,0:1]) ** 2 ))
      f_diff_L = np.sqrt(np.sum(np.dot(self.f_is_ctrl_mat_L, self.F_ctrl_diff_L[:,0:1]) ** 2 ))
      t_diff_R = np.sqrt(np.sum(np.dot(self.t_is_ctrl_mat_R, self.F_ctrl_diff_R[:,1:2]) ** 2 ))
      t_diff_L = np.sqrt(np.sum(np.dot(self.t_is_ctrl_mat_L, self.F_ctrl_diff_L[:,1:2]) ** 2 ))
      
      tf0 = (threshold[0] < 0 or f_diff_R < threshold[0])
      tf1 = (threshold[1] < 0 or f_diff_L < threshold[1])
      tf2 = (threshold[2] < 0 or t_diff_R < threshold[2])
      tf3 = (threshold[3] < 0 or t_diff_L < threshold[3])
      if tf0 and tf1 and tf2 and tf3:
        return 1 # It became target force.
      
      f_offset_R = np.sqrt(np.sum( self.F_ctrl_offset_R[0:3, 3] ** 2))
      f_offset_L = np.sqrt(np.sum( self.F_ctrl_offset_L[0:3, 3] ** 2))
      t_offset_R = np.sqrt(np.sum(Rodrigues(self.F_ctrl_offset_R[0:3, 0:3])[0] ** 2))
      t_offset_L = np.sqrt(np.sum(Rodrigues(self.F_ctrl_offset_L[0:3, 0:3])[0] ** 2))

      tf4 = (threshold[4] > 0 and f_offset_R > threshold[4])
      tf5 = (threshold[5] > 0 and f_offset_L > threshold[5])
      tf6 = (threshold[6] > 0 and t_offset_R > threshold[6])
      tf7 = (threshold[7] > 0 and t_offset_L > threshold[7])
      if tf4 or tf5 or tf6 or tf7:
        return 2 # Arm position is out of range.
      
      time.sleep(0.01)
      
    return -1 # Force control became OFF

  # *** Implement me
  # Must return: None

  # void start_log(out double start_time)
  def start_log(self):
    self.logdata = np.zeros((200*60,15+6+6+1), dtype = np.float64)
    self.log_i = 0
    self.logging = True
    return time.time()  # *** Implement me
  # Must return: start_time

  # void return_log(out log_data data)
  def return_log(self):
    self.logging = False
    ret = [[l[0:15].tolist(), l[15:21].tolist(), l[21:27].tolist(), [l[27]]] for l in self.logdata[0:self.log_i]]
    return ret
  # *** Implement me
  # Must return: data
      
  # void init_force_control(in params R, in params L)
  def init_force_control(self, R, L):
    self.busy_wait()
    self.is_busy = True

    self.lepR_params = np.array(R).reshape((10,1))
    self.lepL_params = np.array(L).reshape((10,1))

    self.is_busy = False
    return None
  # *** Implement me
  # Must return: None

  # void start_force_control(in params rot_R, in params coefficient_R, in params target_R, in params rot_L, in params coefficient_L, in params target_L)
  def start_force_control(self, rot_R, coefficient_R, target_R, rot_L, coefficient_L, target_L):
    self.busy_wait()
    self.is_busy = True
    self.renew_cmd = True

    if self.F_ctrl_origin_R is None:
      self.F_ctrl_origin_R = np.copy(self.frm_current["R_arm"])
      self.F_ctrl_offset_R = np.eye(4)

    if self.F_ctrl_origin_L is None:
      self.F_ctrl_origin_L = np.copy(self.frm_current["L_arm"])
      self.F_ctrl_offset_L = np.eye(4)
      
    if len(rot_R) == 3:
      tmp_r = MATRIX(a=rot_R[0])
      tmp_p = MATRIX(b=rot_R[1])
      tmp_y = MATRIX(c=rot_R[2])
      r_R = (tmp_y * tmp_p * tmp_r).toarray()  # Rotation matrix from Base to ControlAxis
      if len(coefficient_R) == 6:
        tmp_f = np.diag((coefficient_R[0], coefficient_R[1], coefficient_R[2]))
        tmp_t = np.diag((coefficient_R[3], coefficient_R[4], coefficient_R[5]))
        self.t_ctrl_mat_R = r_R.dot( tmp_t ).dot( r_R.T )
        self.f_ctrl_mat_R = r_R.dot( tmp_f ).dot( r_R.T )
        print tmp_t
        print tmp_f
        self.t_is_ctrl_mat_R = r_R.dot((tmp_t<0)*1.0).dot( r_R.T )
        print self.t_is_ctrl_mat_R
        self.f_is_ctrl_mat_R = r_R.dot((tmp_f<0)*1.0).dot( r_R.T )
        print self.f_is_ctrl_mat_R

        
    if len(target_R)==6:
      self.F_ctrl_target_R = r_R.dot(np.array(target_R).reshape((2,3)).T)

      
    if len(rot_L) == 3:
      tmp_r = MATRIX(a=rot_L[0])
      tmp_p = MATRIX(b=rot_L[1])
      tmp_y = MATRIX(c=rot_L[2])
      r_L = (tmp_y * tmp_p * tmp_r).toarray()
      if len(coefficient_L) == 6:
        tmp_f = np.diag((coefficient_L[0], coefficient_L[1], coefficient_L[2]))
        tmp_t = np.diag((coefficient_L[3], coefficient_L[4], coefficient_L[5]))
        self.t_ctrl_mat_L = r_L.dot( tmp_t ).dot( r_L.T )
        self.f_ctrl_mat_L = r_L.dot( tmp_f ).dot( r_L.T )
        self.t_is_ctrl_mat_L = r_L.dot((tmp_t<0)*1.0).dot( r_L.T )
        self.f_is_ctrl_mat_L = r_L.dot((tmp_f<0)*1.0).dot( r_L.T )
        
    if len(target_L)==6:
      self.F_ctrl_target_L = r_L.dot(np.array(target_L).reshape((2,3)).T)
      
    self.is_busy = False
    time.sleep(0.05)
    return None
  # *** Implement me
  # Must return: None

  # void stop_force_control(in boolean R, in boolean L)
  def stop_force_control(self, R, L):
    if R and L: # R and L is True:
      if len(self.target_jointangles) > 0:
        self.waitPositionControl()
      self.reset_param()
      return None
        
    self.busy_wait()
    self.is_busy= True
    if R is True:
      self.t_ctrl_mat_R *= 0
      self.f_ctrl_mat_R *= 0
      self.t_is_ctrl_mat_R *= 0
      self.f_is_ctrl_mat_R *= 0
      self.F_ctrl_target_R = np.zeros((3,2))
      
    if L is True:
      self.t_ctrl_mat_L *= 0
      self.f_ctrl_mat_L *= 0
      self.t_is_ctrl_mat_R *= 0
      self.f_is_ctrl_mat_R *= 0
      self.F_ctrl_target_L = np.zeros((3,2))

    self.is_busy = False
    return None
  # *** Implement me
  # Must return: None


if __name__ == "__main__":
  import sys
  
  # Initialise the ORB
  orb = CORBA.ORB_init(sys.argv)
  
  # As an example, we activate an object in the Root POA
  poa = orb.resolve_initial_references("RootPOA")

  # Create an instance of a servant class
  servant = MyHiroControlService_i()

  # Activate it in the Root POA
  poa.activate_object(servant)

  # Get the object reference to the object
  objref = servant._this()
  
  # Print a stringified IOR for it
  print orb.object_to_string(objref)

  # Activate the Root POA's manager
  poa._get_the_POAManager().activate()

  # Run the ORB, blocking this thread
  orb.run()

