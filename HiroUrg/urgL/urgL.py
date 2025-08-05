#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file urgL.py
 @brief ModuleDescription
 @date $Date$


"""
import sys
import time

## import additional module  ##
import numpy as np
from kinematics_hiro import *
from pyURG import serial_URG
from geo import VECTOR, MATRIX, FRAME
import file_io
#-----------------------------#

sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

import UrgService_idl

# Import Service implementation class
# <rtc-template block="service_impl">
from UrgService_idl_example import *

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
urgl_spec = ["implementation_id", "urgL", 
		 "type_name",         "urgL", 
		 "description",       "ModuleDescription", 
		 "version",           "1.0.0", 
		 "vendor",            "VenderName", 
		 "category",          "Category", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 ""]
# </rtc-template>

# Global variables
URG_PORT = "/dev/ttyACM1"
URG_BAUDRATE = 900000
BUFFER_SIZE = 30

GL_T_MAT = np.zeros((BUFFER_SIZE, 4, 4)) * np.nan
GL_TMSTAMP = np.zeros(BUFFER_SIZE) * np.nan
GL_IS_BUSY = False


RTC_Time_2_float = lambda tm: tm.sec + tm.nsec * (10 ** -9)

#Data Listener
class JointDataListener(OpenRTM_aist.ConnectorDataListenerT):
  def __init__(self):
    self._name = "ON_RECEIVED"
    self.latest_time = 0.0
    self.t_wrist2urg = np.array([[-1,0,0,-0.054],[0,-1,0,0],[0,0,0,0],[0,0,0,1]])


  def solve_fk(self, joint_angle):
    frms = []
    #body#
    frms.append( tB2CY(    joint_angle[0] ))
    #armR
    #frms.append( np.dot(tCY2RSB, tSB2SY(joint_angle[3] )))
    #frms.append( tRSY2RSP( joint_angle[4] ))
    #frms.append( tSP2EP(   joint_angle[5] ))
    #frms.append( tEP2WY(   joint_angle[6] ))
    #frms.append( tWY2WP(   joint_angle[7] ))
    #frms.append( tWP2WR(   joint_angle[8] ))
    # #armL
    frms.append( np.dot(tCY2LSB, tSB2SY(joint_angle[9] )))
    frms.append( tLSY2LSP( joint_angle[10] ))
    frms.append( tSP2EP(   joint_angle[11] ))
    frms.append( tEP2WY(   joint_angle[12] ))
    frms.append( tWY2WP(   joint_angle[13] ))
    frms.append( tWP2WR(   joint_angle[14] ))
    # dic={}
    # dic["R_arm"] = frms[0].dot(frms[1]).dot(frms[2]).dot(frms[3]).dot(frms[ 4]).dot(frms[ 5]).dot(frms[ 6])
    # dic["L_arm"] = frms[0].dot(frms[7]).dot(frms[8]).dot(frms[9]).dot(frms[10]).dot(frms[11]).dot(frms[12])
    return frms[0].dot(frms[1]).dot(frms[2]).dot(frms[3]).dot(frms[ 4]).dot(frms[ 5]).dot(frms[ 6])

  def __del__(self):
    print "dtor of ", self._name

  def __call__(self, info, cdrdata):
    global GL_T_MAT
    global GL_TMSTAMP
    global GL_IS_BUSY
    tm = time.time()
    data = OpenRTM_aist.ConnectorDataListenerT.__call__(self, info, cdrdata,
                                                        RTC.TimedJointData(RTC.Time(0,0),
                                                                           [],[],[],[],[],[],[],[]))
    timestamp = RTC_Time_2_float(data.tm)
    if timestamp > self.latest_time:
      self.latest_time = timestamp
      jointangles = data.qState[0] + data.qState[1] + data.qState[2]
      base2urg = self.solve_fk(jointangles).dot(self.t_wrist2urg)
      while GL_IS_BUSY is True:
        time.sleep(0.00001)
      GL_IS_BUSY = True
      GL_TMSTAMP[1:] = GL_TMSTAMP[:-1]
      GL_TMSTAMP[0] = tm
      GL_T_MAT[1:] = GL_T_MAT[:-1]
      GL_T_MAT[0] = base2urg
      GL_IS_BUSY = False

##
# @class urgL
# @brief ModuleDescription
# 
# 
class urgL(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    
    JointDatIn_arg = [None] * ((len(RTC._d_TimedJointData) - 4) / 2)
    self._d_JointDatIn = RTC.TimedJointData(*JointDatIn_arg)
    self._JointDatInIn = OpenRTM_aist.InPort("JointDatIn", self._d_JointDatIn)

    #-- Add Data listner --#
    self._JointDatInIn.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVED,
						JointDataListener())
    #----------------------#

    data_arg = [None] * ((len(RTC._d_TimedDoubleSeq) - 4) / 2)
    self._d_data = RTC.TimedDoubleSeq(*data_arg)
    self._dataOut = OpenRTM_aist.OutPort("data", self._d_data)
    
    self._svcPort = OpenRTM_aist.CorbaPort("svc")
    self._if = UrgService_i()
    
    #--- Initialize URG ---#
    self.urg = serial_URG(URG_PORT, URG_BAUDRATE)
    self.count_scan = 0 # scan count after match_timestamp()
    #----------------------#
	 
  def onInitialize(self):
    self.addInPort("JointDatIn",self._JointDatInIn)
    self.addOutPort("data",self._dataOut)
    self._svcPort.registerProvider("if", "svcUrg::UrgService", self._if)
    self.addPort(self._svcPort)

    #---- Connect URG ----#
    self.urg.connect()
    self._if.urg = self.urg
    stp = int(25.0 / 360.0 * float(self.urg.parameter["ARES"]))
    stp_start = int(self.urg.parameter["AFRT"]) - stp
    stp_end   = int(self.urg.parameter["AFRT"]) + stp
    self.urg.set_capture_range(stp_start, stp_end)
    #---------------------#

    return RTC.RTC_OK

  def onActivated(self, ec_id):
    self.urg.init_timestamp2()
    self.count_scan = 0
    return RTC.RTC_OK

  
  def onExecute(self, ec_id):
    global GL_IS_BUSY
    # Capture
    xyz, tm_urg = self.urg.capture_GDGS()
    xyz = xyz * 0.001 # mm -> m
   
    # Get joint angle data
    while GL_IS_BUSY == True:
      time.sleep(0.0001)

    GL_IS_BUSY = True
    tm_hiro = np.copy(GL_TMSTAMP)
    t_mat = np.copy(GL_T_MAT)
    GL_IS_BUSY = False

    # Check logging flag.
    if self._if.log_state == "ON":
      log_flag = True
      self._if.log_count += 1
      
    else:
      log_flag = False
      self._if.log_count = -1

    # Convert points to world coordinates.
    data_for_dataport = []
    
    for p, t in zip(xyz, tm_urg):
      if True in np.isnan(p):
        continue
      diff_tm = np.abs(tm_hiro - t)
      if min(diff_tm) > 1:
        return RTC.RTC_OK
      urg_pos = t_mat[np.nanargmin(diff_tm)]
      pos_gl = np.dot(urg_pos, np.array([[p[0],p[1],p[2],1.0]]).T)
      data_tmp = [t] + FRAME(frm = urg_pos).xyzabc() + p[:2].tolist() + pos_gl[0:3, 0].tolist()
      data_for_dataport += data_tmp
      
      if log_flag:
        self._if.writer.csv_write([self._if.log_count] + data_tmp)

    data = RTC.TimedDoubleSeq(tm = RTC.Time(0,0), data = data_for_dataport )
    OpenRTM_aist.setTimestamp(data)
    self._dataOut.write(data)    
    return RTC.RTC_OK


def urgLInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=urgl_spec)
  manager.registerFactory(profile,
                          urgL,
                          OpenRTM_aist.Delete)

def MyModuleInit(manager):
  urgLInit(manager)

  # Create a component
  comp = manager.createComponent("urgL")

def main():
  mgr = OpenRTM_aist.Manager.init(sys.argv)
  mgr.setModuleInitProc(MyModuleInit)
  mgr.activateManager()
  mgr.runManager()
  
if __name__ == "__main__":
  main()

