#!/usr/bin/env python
# -*- Python -*-
import os
import sys
import time
import subprocess
import math

def read_txt(fileName):
  f = open(fileName)
  return f.read()

#
# set up user environment
#  RtmToolsDir, MyRtcDir, etc.
#
#from set_env import *
RtmToolsDir = read_txt("./rtmtools.txt").replace("\n", "")

#
# import user tools
#

#sys.path.append(".")
#save_path = sys.path[:]
sys.path.append(RtmToolsDir+'/rtc_handle')
from rtc_handle import *
from rtc_handle_util import *
sys.path.append(RtmToolsDir+'/embryonic_rtc')
from EmbryonicRtc import *
#sys.path = save_path

#
#
#
import bodyinfo
#
# import stub files
# 
#import _GlobalIDL
import OpenHRP
from OpenHRP import RobotHardwareService as RHS
SwitchStatus=RHS

class HiroEnv(RtmEnv) :
  def __init__(self, orb_args, ns_hiro, orb=None, naming=None) :
    self.ns_hiro=ns_hiro
    nserver_names=[ns_hiro]
    RtmEnv.__init__(self, orb_args, nserver_names, orb, naming)
    
  def init(self) :
    self.init0()
    self.init1()
    
  def init0(self) :
    ns = self.name_space[self.ns_hiro]
    ns.list_obj()

    self.hiro_manager =ns.obj_list[self.ns_hiro+'.host_cxt/manager.mgr']

    self.adm=ns.rtc_handles['SystemAdmin0.rtc']
    self.adm_svc=self.adm.services['SystemAdminService'].provided['service0'].ref
    self.adm.activate()

    self.rh=ns.rtc_handles['RobotHardware0.rtc']
    self.rh_svc=self.rh.services['RobotHardwareService'].provided['service0'].ref

  def init1(self) :
    print "creating components"
    self.rtcList=self.createComps()

    print "connect components"
    self.connectComps()

    print "activating components"
    self.activateComps(self.rtcList)

    print "initialized succesfully"

  def createComps(self) :

    self.seq = self.initRTC("SequencePlayer", "seq")
    self.seq_svc = self.seq.services['SequencePlayerService'].provided['service0'].ref

    self.armR = self.initRTC("ArmControl","armR")
    self.armR_svc = self.armR.services['ArmControlService'].provided['service0'].ref

    self.armL = self.initRTC("ArmControl","armL")
    self.armL_svc = self.armL.services['ArmControlService'].provided['service0'].ref

    self.grsp = self.initRTC("Grasper", "grsp")
    self.grsp_svc = self.grsp.services['GrasperService'].provided['service0'].ref

    self.sa = self.initRTC("StateArbitrator", "sa")
    self.tk_svc = self.sa.services['TimeKeeperService'].provided['service1'].ref

    self.log = self.initRTC("DataLogger", "log")
    self.log_svc = self.log.services['DataLoggerService'].provided['service0'].ref

    self.gf = self.initRTC("GazeFixer", "gf")

    self.ca = self.initRTC("CollisionAvoider", "ca")

    return [self.rh, self.seq, self.armR, self.armL, self.sa, self.grsp, self.gf, self.ca, self.log]

#  ns_hiro.list_obj()

  def initRTC(self, module, name) :
    ns = self.name_space[self.ns_hiro]
    ms = self.hiro_manager

    if name+".rtc" in ns.rtc_handles.keys() :
      print name + ".rtc" + " already exists."
      return ns.rtc_handles[name+".rtc"]
    else :
      ms.load_module( module + ".so", module + "Init")
      time.sleep(1)
      ref=ms.create_component(module + "?instance_name=" + name)
      time.sleep(1)
    #    ref = ns_hiro.get_object_by_name(name+".rtc")
      ns.obj_list[name+".rtc"]=ref
      try :
        ns.rtc_handles[name+".rtc"]=RtcHandle(name+".rtc", ns, ref)
        print "handle for " + name + ".rtc has been created."
        return ns.rtc_handles[name+".rtc"]
      except :
        print name + ".rtc could not created.", sys.exc_info()[0]
        return None

  def connectComps(self) :
    self.connectors={}
    hiro_c_prop={
        'dataport.subscription_type':'new',
        'dataport.publisher.push_policy':'new',
        'dataport.inport.buffer.length':'1',
        'dataport.inport.buffer.read.empty_policy':'block',
        'dataport.inport.buffer.write.full_policy':'block',
        'dataport.outport.buffer.length' : '1',
        'dataport.outport.buffer.write.full_policy':'block',
        'dataport.outport.buffer.read.empty_policy':'block'
    }

    con=IOConnector([self.rh.outports['jointDatOut'],self.seq.inports['jointDatIn']],prop_dict=hiro_c_prop)
    self.connectors['rh-seq']=con
    check_connection(con)

    con=IOConnector([self.rh.outports['jointDatOut'],self.armR.inports['jointDatIn']],prop_dict=hiro_c_prop)
    self.connectors['rh-armR']=con
    check_connection(con)

    con=IOConnector([self.rh.outports['jointDatOut'],self.armL.inports['jointDatIn']],prop_dict=hiro_c_prop)
    self.connectors['rh-armL']=con
    check_connection(con)

    con=IOConnector([self.seq.outports['jointDatOut'],self.sa.inports['jointDatIn0']],prop_dict=hiro_c_prop)
    self.connectors['seq-sa']=con
    check_connection(con)

    con=IOConnector([self.armR.outports['jointDatOut'],self.sa.inports['jointDatIn1']],prop_dict=hiro_c_prop)
    self.connectors['armR-sa']=con
    check_connection(con)

    con=IOConnector([self.armL.outports['jointDatOut'],self.sa.inports['jointDatIn2']],prop_dict=hiro_c_prop)
    self.connectors['armL-sa']=con
    check_connection(con)

    con=IOConnector([self.sa.outports['jointDatOut'],self.grsp.inports['jointDatIn']],prop_dict=hiro_c_prop)
    self.connectors['sa-grsp']=con
    check_connection(con)

    con=IOConnector([self.grsp.outports['jointDatOut'],self.gf.inports['jointDatIn']],prop_dict=hiro_c_prop)
    self.connectors['grsp-gf']=con
    check_connection(con)

    con=IOConnector([self.gf.outports['jointDatOut'],self.ca.inports['jointDatIn']],prop_dict=hiro_c_prop)
    self.connectors['gf-ca']=con
    check_connection(con)

    con=IOConnector([self.ca.outports['jointDatOut'],self.rh.inports['jointDatIn']],prop_dict=hiro_c_prop)
    self.connectors['ca-rf']=con
    check_connection(con)

    connect_seq(self)

  def activateComps(self, rtcs) :
    serializeComponents(rtcs)
    # activate_seq(self, rtcs)
    for r in rtcs:
      r.ec.activate_component(r.rtc_ref)

  def setJointAnglesDeg(self, pose, tm, wait=True) :
    ret,ttm = self.seq_svc.setJointAngles(bodyinfo.deg2radPose(pose), tm)
    mask = 31
    if wait :
      self.seq_svc.isEmpty(mask, True)
    return ret,ttm

  def setJointAnglesRad(self, pose, tm, wait=True) :
    ret,ttm = self.seq_svc.setJointAngles(pose, tm)
    mask = 31
    if wait :
      self.seq_svc.isEmpty(mask, True)
    return ret,ttm

  def goInitial(self, tm=bodyinfo.timeToInitialPose, wait = True):
    self.setJointAnglesDeg(bodyinfo.initialPose, tm, wait=wait)


  def goOffPose(self, tm=bodyinfo.timeToOffPose, wait = True):
    self.setJointAnglesDeg(bodyinfo.offPose, tm, wait=wait)
    if wait:
      self.servoOff(doConfirm=False)


  def servoOn(self, part = 'all', doConfirm = True):
    if doConfirm:
      #waitInputConfirm("!! Robot Motion Warning !! \n Push [OK] to Servo ON "+part)
      raw_input("!! Robot Motion Warning !! \n Push [OK] to Servo ON "+part)
    rh_svc=self.rh_svc
    if rh_svc != None:
      if part == 'all':
        rh_svc.servo('BODY', SwitchStatus.SWITCH_ON)
        rh_svc.servo('RARM', SwitchStatus.SWITCH_ON)
        rh_svc.servo('LARM', SwitchStatus.SWITCH_ON)
        rh_svc.servo('RHAND', SwitchStatus.SWITCH_ON)
        rh_svc.servo('LHAND', SwitchStatus.SWITCH_ON)
            # setup logging
        self.setupLogger()
      else:
        rh_svc.servo(part, SwitchStatus.SWITCH_ON)


  def servoOff(self, part = 'all',doConfirm=True):
    if doConfirm:
      #waitInputConfirm("!! Robot Motion Warning !! \n Push [OK] to Servo OFF "+part)
      raw_input("!! Robot Motion Warning !! \n Push [OK] to Servo OFF "+part)
    rh_svc=self.rh_svc
    if rh_svc != None:
      # save log
      self.saveLog()
      rh_svc.servo(part, SwitchStatus.SWITCH_OFF)
      if part == 'all':
        rh_svc.servo('RHAND', SwitchStatus.SWITCH_OFF)
        rh_svc.servo('LHAND', SwitchStatus.SWITCH_OFF)

  def getRobotState(self):
#    rstt = RobotStateHolder()
#    rh_svc.getStatus(rstt)
    rstt=self.rh_svc.getStatus()
    return rstt

  def getJointAnglesDeg(self):
    rstt = self.getRobotState()
    return [v/math.pi*180.0 for v in rstt.angle]

  def getJointAnglesRad(self):
    rstt = self.getRobotState()
    return rstt.angle
  
  def showJointAnglesDeg(self):
    print self.getJointAnglesDeg()

  def loadPattern(self, basename, tm=3.0):
    self.seq_svc.loadPattern(basename, tm)


  def testPattern(self):
    #waitInputConfirm("!! Robot Motion Warning !! \n Push [OK] to execute "+ bodyinfo.testPatternName)
    raw_input("!! Robot Motion Warning !! \n Push [OK] to execute "+ bodyinfo.testPatternName)
    # dblHolder = org.omg.CORBA.DoubleHolder()
    for p in bodyinfo.testPattern:
        print setJointAnglesDeg(p[0], p[1])
    # waitInputConfirm("finished")
    raw_input("finished")

  def setupLogger(self):
    self.log_svc.add("TimedJointData", "jointDatServo")
    self.log_svc.add("TimedJointData", "jointDatSeq")

    time.sleep(1)
    self.log.retrieve_info()
    
    self.log_connectors={}
    con=IOConnector([self.rh.outports['jointDatOut'],self.log.inports['jointDatServo']])
    self.log_connectors['rh-log']=con
    con=IOConnector([self.seq.outports['jointDatOut'],self.log.inports['jointDatSeq']])
    self.log_connectors['seq-log']=con

    self.log_connectors['rh-log'].connect()
    self.log_connectors['seq-log'].connect()

  def dateString(self):
    tm = time.localtime()
    return '%04d%02d%02d%02d%02d%02d'%(tm.tm_year, 
                                       tm.tm_mon,
                                       tm.tm_mday,
                                       tm.tm_hour,
                                       tm.tm_min,
                                       tm.tm_sec)

  def saveLog(self):
    if self.log_svc == None:
      #waitInputConfirm("Setup DataLogger RTC first.")
      raw_input("Setup DataLogger RTC first.")
      return
    self.log_svc.save('/opt/grx/'+bodyinfo.modelName+'/log/logger-'+self.dateString())
    print 'saved'
    self.log_connectors['rh-log'].disconnect()
    self.log_connectors['seq-log'].disconnect()

  def calibrateJoint(self):
    print('calibrating joints ...'),
    if self.rh_svc.calibrateJoint('all') == 0:
      print('Finished.')
    else:
      print('Failed. servoOff() and try again.')

  def servoOnHands(self):
    self.servoOn('RHAND')
    self.servoOn('LHAND')

  def servoOffHands(self):
    self.servoOff('RHAND')
    self.servoOff('LHAND')
    
  def EngageProtectiveStop(self):
    self.rh_svc.engageProtectiveStop()
  
  def DisengageProtectiveStop(self):
    self.rh_svc.disengageProtectiveStop()

  def reboot(self):
    if self.adm_svc != None:
        raw_input("Reboot the robot host ?")
        self.adm_svc.reboot("")

  def shutdown(self):
    if self.adm_svc != None:
        raw_input("Shutdown the robot host ?")
        self.adm_svc.shutdown("")

  def getVersion(self):
    if self.adm_svc != None:
        currentVersion = self.adm_svc.getVersion("")
        # waitInputConfirm("Robot system version is: %s"%(currentVersion))
        raw_input("Robot system version is: " + currentVersion)

  #
  # move arms using Inverse Kinematics
  #
  def moveRelativeR(self, dx=0, dy=0, dz=0, dr=0, dp=0, dw=0, rate=30, wait=True):
    x,y,z,r,p,w = self.getCurrentConfiguration(self.armR_svc)
    return self.move(self.armR_svc, x+dx, y+dy, z+dz, r+dr, p+dp, w+dw, rate, wait) 

  def moveRelativeL(self, dx=0, dy=0, dz=0, dr=0, dp=0, dw=0, rate=30, wait=True):
    x,y,z,r,p,w = self.getCurrentConfiguration(self.armL_svc)
    return self.move(self.armL_svc, x+dx, y+dy, z+dz, r+dr, p+dp, w+dw, rate, wait) 

  def moveR(self, x, y, z, r, p, w, rate=30, wait=True):
    return self.move(self.armR_svc, x, y, z, r, p, w, rate, wait)

  def moveL(self, x, y, z, r, p, w, rate=30, wait=True):
    return self.move(self.armL_svc, x, y, z, r, p, w, rate, wait)

  def move(self, armsvc, x, y, z, r, p, w, rate, wait):
    ret = armsvc.setTargetAngular(x, y, z, r, p, w, rate)
    while wait:
      self.tk_svc.sleep(0.2)
#      if armsvc.checkStatus() != OpenHRP.ArmState.ArmBusyState:
      if armsvc.checkStatus() != OpenHRP.ArmBusyState:
        break
    return ret

  def getCurrentConfiguration(self, armsvc, show=False):
    ret= armsvc.getCurrentConfiguration()
    ret=ret[1:]   # cut return value. it maybe "True".
    if show:
      print '\n(x,y,z,r,p,w) = '
      for v in ret:
        print '%6.3f,'%v,
    return ret

  def gazeFixerTest(self):
    self.gazeFixerOn()
    self.setJointAnglesDeg([[20,0,0], [], [], [], []], 5)
    self.gazeFixerOff()

  def gazeFixerOn(self):
    self.gf.setProperty("isEnabled", "1")

  def gazeFixerOff(self):
    self.gf.setProperty("isEnabled", "0")

  def collisionCheckOn(self):
    self.ca.setProperty("isEnabled", "1")

  def collisionCheckOff(self):
    self.ca.setProperty("isEnabled", "0")


def serializeComponents(rtcs) :
  if len(rtcs) < 2 :
    return
  ec = rtcs[0].execution_contexts[0]
  rtcs[0].ec = ec #
  for rtc in rtcs[1:] :
    if rtc.execution_contexts[0] != ec :
      rtc.execution_contexts[0].stop()
      if ec.add_component(rtc.rtc_ref) == RTC.RTC_OK :
        rtc.ec=ec
      else :
        print 'error in add_commponent()'
    else :
      print 'already serialized'

def check_connection(con) :
  port0=con.plist[0]
  rslt=False
  for r_con in port0.get_connections() :
    r_port_list=r_con.ports
    rslt=True
    for i in range(len(con.port_reflist)) :
      if not r_port_list[i]._is_equivalent(con.port_reflist[i]) :
        rslt=False
        break
    if rslt :
      con.profile=r_con
      con.prop_nvlist=con.profile.properties
      con.prop_dict=nvlist2dict(con.prop_nvlist)
      break
  con.connectp=rslt
  return rslt
    

#env= RtmEnv(sys.argv,[NS0,NS1])
#hiro= HiroEnv(None, "hiro022", env.orb)

# to setup hiro022
#  >>> hiro.init()
# to shutdown hiro022
#  >>> hiro.adm_srv.shutdown("")
