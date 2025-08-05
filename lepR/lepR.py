#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file lepR.py
 @brief Leptrino Force Sensor
 @date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

# Import Leptrino Module
from pyLeptrino import Leptrino

# Define Port of Leptrino
GL_PORT_LEPTRINO = "/dev/lepR"

# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
lepr_spec = ["implementation_id", "lepR", 
	     "type_name",         "lepR", 
	     "description",       "Leptrino Force Sensor", 
	     "version",           "1.0.0", 
	     "vendor",            "M. Takizawa", 
	     "category",          "Category", 
	     "activity_type",     "STATIC", 
	     "max_instance",      "1", 
	     "language",          "Python", 
	     "lang_type",         "SCRIPT",
	     ""]
# </rtc-template>

##
# @class lepR
# @brief Leptrino Force Sensor
# 
# 
class lepR(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    
    sens_data_arg = [None] * ((len(RTC._d_TimedDoubleSeq) - 4) / 2)
    self._d_sens_data = RTC.TimedDoubleSeq(*sens_data_arg)
    """
    """
    self._sens_dataOut = OpenRTM_aist.OutPort("sens_data", self._d_sens_data)

    #Open Leptrino
    self.lep = Leptrino(GL_PORT_LEPTRINO)
    self.lep.connect()

    
  def onInitialize(self):
    self.addOutPort("sens_data",self._sens_dataOut)
    return RTC.RTC_OK
    
    
  def onExecute(self, ec_id):
    data = RTC.TimedDoubleSeq(tm = RTC.Time(0,0), data = self.lep.hand_shake())
    OpenRTM_aist.setTimestamp(data)
    self._sens_dataOut.write(data)
    return RTC.RTC_OK

  
  
def lepRInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=lepr_spec)
  manager.registerFactory(profile,
                          lepR,
                          OpenRTM_aist.Delete)

def MyModuleInit(manager):
  lepRInit(manager)

  # Create a component
  comp = manager.createComponent("lepR")

def main():
  mgr = OpenRTM_aist.Manager.init(sys.argv)
  mgr.setModuleInitProc(MyModuleInit)
  mgr.activateManager()
  mgr.runManager()

if __name__ == "__main__":
  main()

