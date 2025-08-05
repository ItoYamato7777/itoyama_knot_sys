#!/usr/bin/env python
# -*- Python -*-

import sys

# Import RTM module
import OpenRTM_aist
import RTC

#
# Holder class is a dirty trick to keep important information.
#  this need to be revised, if you find better way.
#


# This module's spesification
#  you can change :
#    before running the rtc, just change embryonic_rtc_spec string
#    after  running the rtc, change mgr.getComponents()[0].comp._properties
#      mgr.getComponents()[0]._properties.setProperty("type_name","new type")
#
embryonic_rtc_spec = ["implementation_id", "EmbryonicRtc", 
		 "type_name",         "EmbryonicRtc", 
		 "description",       "embryonic rtc", 
		 "version",           "1.0", 
		 "vendor",            "Takashi Suehiro, AIST", 
		 "category",          "General", 
		 "activity_type",     "DataFlowComponent", 
		 "max_instance",      "10", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 "conf.default.conf0", "0",
		 "conf.default.conf1", "1.0",
		 ""]

class EmbryonicRtc(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
#
# you can add Ports and configulations to e_rtc after creating it.
#
  def makeInPort(self, name, data, buf) :
    data_name = "_d_" + name
    setattr(self, data_name, data)
    port_name = "_" + name + "In"
    port = OpenRTM_aist.InPort(name, data, buf)
    setattr(self, port_name, port)
    self.registerInPort(name,  port)
    return port

  def makeOutPort(self, name, data, buf) :
    data_name = "_d_" + name
    setattr(self, data_name, data)
    port_name = "_" + name + "Out"
    port = OpenRTM_aist.OutPort(name, data, buf)
    setattr(self, port_name, port)
    self.registerOutPort(name,  port)
    return port

#
# mk_comp_sample is an example of how to create InPort, Outport,
# ServicePort.
# you can do this after this e_rtc is created.
# 
#def mk_comp_sample(comp) :  # holder.comp contains an e_rtc component
#
#    # make InPorts
#    makeInPort(comp, "th", RTC.TimedDoubleSeq(RTC.Time(0,0),[]), 
#                                                  OpenRTM_aist.RingBuffer(8))
#
#    # make OutPorts
#    makeOutPort(comp, "frame", RTC.TimedDoubleSeq(RTC.Time(0,0),[]), 
#                                                   OpenRTM_aist.RingBuffer(8))
#
#    # Set service Port which contains providers
#    #  skelton has been already imported from _GlobalIDL and
#    # _GlobalIDL__POA
#    # MyService_i must be defined in my_service_impl.py
#    comp._provPort = OpenRTM_aist.CorbaPort("prov")
#    comp._prov1 = MyService_i()
#    comp._provPort.registerProvider("prov1", "MyService", comp._prov1)
#    comp.registerPort(comp._provPort)
#
#    # Set service Port whic contains consumers
#    #  stub is needed here
#    comp._consPort = OpenRTM_aist.CorbaPort("cons")
#    comp._cons1 = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.YourService)
#    comp._consPort.registerConsumer("cons1", "YourService", comp._cons1)
#    comp.registerPort(comp._consPort)
#
#    # initialize of configuration-data.
#    # <rtc-template block="init_conf_param">
#    # data for configulation is set here and used later
#    comp._conf0 = [0]
#    comp._conf1 = [1.0]

#
# configuration set can be set as follows	
#def onInit_sub(comp) :                # onInitialize runs after mk_comp.
#    # Bind variables and configuration variable
#    comp.bindParameter("conf0", comp._conf0, "0")
#    comp.bindParameter("conf1", comp._conf1, "1.0")


def MyModuleInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=embryonic_rtc_spec)
    manager.registerFactory(profile,
                            EmbryonicRtc,
                            OpenRTM_aist.Delete)

    # Create a component
    comp = manager.createComponent("EmbryonicRtc")

def main(ll=True):
  mgr = OpenRTM_aist.Manager.init(len(sys.argv), sys.argv)
  mgr.setModuleInitProc(MyModuleInit)
  mgr.activateManager()
  mgr.runManager(True)
  return mgr
#
if __name__ == "__main__":
  main(False)

