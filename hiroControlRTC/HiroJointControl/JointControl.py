#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file JointControl.py
 @brief Control joint angle of hiro.
 @date $Date$


"""
import copy
from cv2 import Rodrigues
import numpy as np
import sys
import threading
import time
sys.path.append(".")

from kinematics_hiro import *

# Import RTM module
import RTC
import OpenRTM_aist

import hiroJointControl_idl

# Import Service implementation class
# <rtc-template block="service_impl">
from hiroJointControl_idl_example import *

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
jointcontrol_spec = ["implementation_id", "JointControl", 
		     "type_name",         "JointControl", 
		     "description",       "Control joint angle of hiro.", 
		     "version",           "1.0.0", 
		     "vendor",            "Masaru Takizawa", 
		     "category",          "Category", 
		     "activity_type",     "STATIC", 
		     "max_instance",      "0", 
		     "language",          "Python", 
		     "lang_type",         "SCRIPT",
		     ""]
# </rtc-template>

##
# @class JointControl
# @brief Control joint angle of hiro.
# 
#

GL_Received_JointData = None

LEP_FILTER_SIZE = 30
GL_data_lepR = np.zeros((LEP_FILTER_SIZE, 6)) * np.nan
GL_data_lepL = np.zeros((LEP_FILTER_SIZE, 6)) * np.nan

class JointDataListener(OpenRTM_aist.ConnectorDataListenerT):
  def __init__(self):
    self._name = "ON_RECEIVED"
    self.current_time = RTC.Time(0, 0)
  def __del__(self):
    print "dtor of ", self._name

  def __call__(self, info, cdrdata):
    global GL_Received_JointData
    tmp = OpenRTM_aist.ConnectorDataListenerT.__call__(self, info, cdrdata, RTC.TimedJointData(RTC.Time(0,0),[],[],[],[],[],[],[],[]))
    GL_Received_JointData = tmp

      
class lepR_Listener(OpenRTM_aist.ConnectorDataListenerT):
  def __init__(self):
    self._name = "ON_RECEIVED"
  def __del__(self):
    print "dtor of ", self._name
  def __call__(self, info, cdrdata):
    global GL_data_lepR
    received = OpenRTM_aist.ConnectorDataListenerT.__call__(self, info, cdrdata, RTC.TimedDoubleSeq(RTC.Time(0,0),[]))
    GL_data_lepR[1:] = GL_data_lepR[:-1]
    GL_data_lepR[0] = np.array(received.data)

class lepL_Listener(OpenRTM_aist.ConnectorDataListenerT):
  def __init__(self):
    self._name = "ON_RECEIVED"
  def __del__(self):
    print "dtor of ", self._name
  def __call__(self, info, cdrdata):
    global GL_data_lepL
    received = OpenRTM_aist.ConnectorDataListenerT.__call__(self, info, cdrdata, RTC.TimedDoubleSeq(RTC.Time(0,0),[]))
    GL_data_lepL[1:] = GL_data_lepL[:-1]
    GL_data_lepL[0] = np.array(received.data)

    
RTC_Time_2_float = lambda tm: tm.sec + tm.nsec * (10 ** -9)

class JointControl(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

    jointDatIn_arg = [None] * ((len(RTC._d_TimedJointData) - 4) / 2)
    self._d_jointDatIn = RTC.TimedJointData(*jointDatIn_arg)
    """
    """
    self._jointDatInIn = OpenRTM_aist.InPort("jointDatIn", self._d_jointDatIn)
    self._jointDatInIn.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVED, JointDataListener())
    forceR_arg = [None] * ((len(RTC._d_TimedDoubleSeq) - 4) / 2)
    self._d_forceR = RTC.TimedDoubleSeq(*forceR_arg)
    """
    """
    self._forceRIn = OpenRTM_aist.InPort("forceR", self._d_forceR)
    self._forceRIn.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVED, lepR_Listener())
    forceL_arg = [None] * ((len(RTC._d_TimedDoubleSeq) - 4) / 2)
    self._d_forceL = RTC.TimedDoubleSeq(*forceL_arg)
    """
    """
    self._forceLIn = OpenRTM_aist.InPort("forceL", self._d_forceL)
    self._forceLIn.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVED, lepL_Listener())
    jointDatOut_arg = [None] * ((len(RTC._d_TimedJointData) - 4) / 2)
    self._d_jointDatOut = RTC.TimedJointData(*jointDatOut_arg)
    """
    """
    self._jointDatOutOut = OpenRTM_aist.OutPort("jointDatOut", self._d_jointDatOut)
    
    """
    """
    self._svcPort = OpenRTM_aist.CorbaPort("svc")
    
    """
    """
    self._if = MyHiroControlService_i()
    
    self.wrist2lep = np.array([[0., 0., -1.],
                               [0., 1.,  0.],
                               [1., 0.,  0.]])

    self.current_time = RTC.Time(0, 0)
    
  def onInitialize(self):
    # Bind variables and configuration variable
    
    # Set InPort buffers
    self.addInPort("jointDatIn",self._jointDatInIn)
    self.addInPort("forceR",self._forceRIn)
    self.addInPort("forceL",self._forceLIn)
    
    # Set OutPort buffers
    self.addOutPort("jointDatOut",self._jointDatOutOut)
    
    # Set service provider to Ports
    self._svcPort.registerProvider("if", "myHRP::MyHiroControlService", self._if)
    
    # Set service consumers to Ports
    
    # Set CORBA Service Ports
    self.addPort(self._svcPort)

    
    return RTC.RTC_OK


  def solve_fk(self, joint_angle):
    frms = []
    #body#
    frms.append( tB2CY(    joint_angle[0] ))
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
    dic={}
    dic["R_arm"] = frms[0].dot(frms[1]).dot(frms[2]).dot(frms[3]).dot(frms[ 4]).dot(frms[ 5]).dot(frms[ 6])
    dic["L_arm"] = frms[0].dot(frms[7]).dot(frms[8]).dot(frms[9]).dot(frms[10]).dot(frms[11]).dot(frms[12])
    return dic


  def calc_force_process_R(self):
    force_raw = np.nanmean(GL_data_lepR, 0)
    rot = np.dot(self._if.frm_current["R_arm"][0:3,0:3], self.wrist2lep) # rotation base2leptrino
    grav_vec = np.dot(rot.T, np.array([[0.],[0.],[-1.]])).reshape(3)
    o_mat = np.array([[1, 0, 0, 0, 0, 0, grav_vec[0], 0, 0, 0],
                      [0, 1, 0, 0, 0, 0, grav_vec[1], 0, 0, 0],
                      [0, 0, 1, 0, 0, 0, grav_vec[2], 0, 0, 0],
                      [0, 0, 0, 1, 0, 0, 0,           0          ,  grav_vec[2], -grav_vec[1]],
                      [0, 0, 0, 0, 1, 0, 0,          -grav_vec[2],  0          ,  grav_vec[2]],
                      [0, 0, 0, 0, 0, 1, 0,           grav_vec[1], -grav_vec[0],  0          ]])
    offset = np.dot(o_mat, self._if.lepR_params).reshape(6)
    force = (force_raw - offset).reshape((2,3)).T
    force = np.dot(rot, force)
    self._if.F_ctrl_diff_R = self._if.F_ctrl_target_R - force
    f_ctrl_vec = np.dot(self._if.f_ctrl_mat_R, self._if.F_ctrl_diff_R[:,0:1])
    t_ctrl_vec = np.dot(self._if.t_ctrl_mat_R, self._if.F_ctrl_diff_R[:,1:2])
    self._if.F_ctrl_offset_R[0:3,3:4] = self._if.F_ctrl_offset_R[0:3,3:4] + f_ctrl_vec
    self._if.F_ctrl_offset_R[0:3,0:3] = np.dot(Rodrigues(t_ctrl_vec)[0], self._if.F_ctrl_offset_R[0:3,0:3])
    self.forceR_raw = force_raw
    self.forceR = force


  def calc_force_process_L(self):
    force_raw = np.nanmean(GL_data_lepL, 0)
    rot = np.dot(self._if.frm_current["L_arm"][0:3,0:3], self.wrist2lep) # rotation base2leptrino
    grav_vec = np.dot(rot.T, np.array([[0.],[0.],[-1.]])).reshape(3)
    o_mat = np.array([[1, 0, 0, 0, 0, 0, grav_vec[0], 0, 0, 0],
                      [0, 1, 0, 0, 0, 0, grav_vec[1], 0, 0, 0],
                      [0, 0, 1, 0, 0, 0, grav_vec[2], 0, 0, 0],
                      [0, 0, 0, 1, 0, 0, 0,           0          ,  grav_vec[2], -grav_vec[1]],
                      [0, 0, 0, 0, 1, 0, 0,          -grav_vec[2],  0          ,  grav_vec[2]],
                      [0, 0, 0, 0, 0, 1, 0,           grav_vec[1], -grav_vec[0],  0          ]])

    offset = np.dot(o_mat, self._if.lepL_params).reshape(6)
    force = (force_raw - offset).reshape((2,3)).T
    force = np.dot(rot, force)
    self._if.F_ctrl_diff_L = self._if.F_ctrl_target_L - force
    f_ctrl_vec = np.dot(self._if.f_ctrl_mat_L, self._if.F_ctrl_diff_L[:,0:1])
    t_ctrl_vec = np.dot(self._if.t_ctrl_mat_L, self._if.F_ctrl_diff_L[:,1:2])
    self._if.F_ctrl_offset_L[0:3,3:4] = self._if.F_ctrl_offset_L[0:3,3:4] + f_ctrl_vec
    self._if.F_ctrl_offset_L[0:3,0:3] = np.dot(Rodrigues(t_ctrl_vec)[0], self._if.F_ctrl_offset_L[0:3,0:3])
    self.forceL_raw = force_raw
    self.forceL = force

  
  def onExecute(self, ec_id):
    write_flag = False
    data = GL_Received_JointData
    OpenRTM_aist.setTimestamp(data)
    self.current_time = data.tm
    
    self._if.current_jointangles = np.array(data.qState[0]+data.qState[1]+data.qState[2])
    self._if.frm_current = self.solve_fk(self._if.current_jointangles)

    #wait overwriting parameters
    self._if.busy_wait()
    self._if.is_busy = True

    # When new motion is loaded, update data.cmd
    if self._if.renew_cmd is True:
      self.qCommand_old = np.array(data.qCommand[0]+data.qCommand[1]+data.qCommand[2])
      self._if.renew_cmd = RTC_Time_2_float(self.current_time)
      data.cmd[0:3] = [self.current_time] * 3
      self.cmd_old = data.cmd[0:3]
      data.id = '\x01\x01\x01\xfe\xfe'
      write_flag = True

    elif isinstance(self._if.renew_cmd, float):
      # When pause signal is received, stop moving.
      if data.cmd[0].sec == 0:
        self._if.reset_param()
      # When new motion is executed by other compornent, stop moving.
      elif max([RTC_Time_2_float(x) for x in data.cmd[0:3]]) > self._if.renew_cmd:
         self._if.reset_param()
      # Continue to move
      else:
        data.cmd[0:3] = self.cmd_old #[self.current_time] * 3
        data.id = '\xfd\xfd\xfd\xfe\xfe'        
      
        
    tar_angle = self._if.current_jointangles
    tar_CY = None
    # Update target joint angles (position controll)
    if len(self._if.target_jointangles) > 0:
      tar_angle = self._if.target_jointangles.pop(0)
      tar_CY = tar_angle[0]
      tar_frm = self.solve_fk(tar_angle)
      self._if.F_ctrl_origin_L = np.copy(tar_frm["L_arm"])
      self._if.F_ctrl_origin_R = np.copy(tar_frm["R_arm"])
      write_flag = True

    #force controll
    self.calc_force_process_R()
    self.calc_force_process_L()
    
    if not self._if.F_ctrl_origin_L is None:
      L_pos = np.copy(self._if.F_ctrl_origin_L)
      L_pos[0:3, 3:4] = L_pos[0:3, 3:4] + self._if.F_ctrl_offset_L[0:3,3:4]
      L_pos[0:3, 0:3] = np.dot(self._if.F_ctrl_offset_L[0:3,0:3], L_pos[0:3, 0:3])
      write_flag = True
    else:
      L_pos = None

    if not self._if.F_ctrl_origin_R is None:
      R_pos = np.copy(self._if.F_ctrl_origin_R)
      R_pos[0:3, 3:4] = R_pos[0:3, 3:4] + self._if.F_ctrl_offset_R[0:3,3:4]
      R_pos[0:3, 0:3] = np.dot(self._if.F_ctrl_offset_R[0:3,0:3], R_pos[0:3, 0:3])
      write_flag = True
    else:
      R_pos = None
      
    if write_flag:
      tar_angle = solve_ik(self._if.current_jointangles,
                           R_pos, L_pos,
                           np.eye(4),np.eye(4),
                           tar_CY,
                           False,
                           1000,
                           0.5).tolist()

      # Fail Safe
      diff_max = np.max(np.abs(tar_angle - self.qCommand_old))
      if diff_max * 200 > 0.75:
        print "move speed limitation"
        self._if.reset_param()
        write_flag = False

      self.qCommand_old = np.array(tar_angle)

    self._if.is_busy = False
    
    # Update target joint angle and write.
    if write_flag:
      data.qCommand[0] = tar_angle[0:3]
      data.qCommand[1] = tar_angle[3:9]
      data.qCommand[2] = tar_angle[9:15]
      self._jointDatOutOut.write(data)

      
    # Logging
    if self._if.logging:
      self._if.logdata[self._if.log_i] = np.array(self._if.current_jointangles.tolist()+self.forceR_raw.tolist()+self.forceL_raw.tolist()+[time.time()])
      self._if.log_i = (self._if.log_i + 1) % (200*60)

      
    return RTC.RTC_OK
	


def JointControlInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=jointcontrol_spec)
  manager.registerFactory(profile,
                          JointControl,
                          OpenRTM_aist.Delete)

def MyModuleInit(manager):
  JointControlInit(manager)
  
  # Create a component
  comp = manager.createComponent("JointControl")

def main():
  mgr = OpenRTM_aist.Manager.init(sys.argv)
  mgr.setModuleInitProc(MyModuleInit)
  mgr.activateManager()
  mgr.runManager()
  
if __name__ == "__main__":
  main()

