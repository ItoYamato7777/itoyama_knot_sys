# -*- coding: utf-8 -*-

ns_hiro = "hiro022"
ns_urg = "localhost:2809"
ns_list = [ns_hiro, ns_urg]

import sys
import subprocess
import os
import time
import numpy as np
from math import radians, pi

#from geo import FRAME, MATRIX, VECTOR
#from kinematics_hiro import solve_ik, solve_fk

def read_txt(fileName):
  f = open(fileName)
  return f.read()

path_rtmtools =  read_txt("./rtmtools.txt").replace("\n", "")
path_IncludeFiles = read_txt("./includefiles.txt").replace("\n", "")

## run rtc ##
# urg components
# os.popen2("cd " + path_IncludeFiles + "/HiroUrg/urgR; xterm -e python urgR.py")
# os.popen2("cd " + path_IncludeFiles + "/HiroUrg/urgL; xterm -e python urgL.py")

## run rtc ##
# urg components

urgR_dir = os.path.join(path_IncludeFiles, "HiroUrg", "urgR")
urgL_dir = os.path.join(path_IncludeFiles, "HiroUrg", "urgL")

# urgR.pyをバックグラウンドで起動し、ログを 'urgR.log' に出力
log_file_R = open("urgR.log", "w")
process_R = subprocess.Popen(
    ["python", "urgR.py"],
    cwd=urgR_dir,
    stdout=log_file_R,
    stderr=subprocess.STDOUT
)
# Python 2.7のprint文と文字列フォーマット
print "urgR.py をプロセスID: %d で起動しました。ログは urgR.log を参照してください。" % process_R.pid

# urgL.pyをバックグラウンドで起動し、ログを 'urgL.log' に出力
log_file_L = open("urgL.log", "w")
process_L = subprocess.Popen(
    ["python", "urgL.py"],
    cwd=urgL_dir,
    stdout=log_file_L,
    stderr=subprocess.STDOUT
)
print "urgL.py をプロセスID: %d で起動しました。ログは urgL.log を参照してください。" % process_L.pid

time.sleep(3)

sys.path.append(path_rtmtools + "/rtc_handle")
sys.path.append(path_rtmtools + "/embryonic_rtc")
 
import rtc_handle
import rtc_handle_util
import EmbryonicRtc

import svcUrg


def run_embryonic_rtc():
  mgr = EmbryonicRtc.main()
  comp = mgr.getComponents()[0]
  return comp


def get_rtc_env():
  argv = sys.argv + ["-ORBgiopMaxMsgSize", "104857600000"]
  env = rtc_handle.RtmEnv(argv, ns_list)
  for ns in env.name_space:
    env.name_space[ns].list_obj()
  return env


class C_hiro_urg():
  def __init__(self, env, comp):
    
    rtc_urgR = env.name_space[ns_urg].rtc_handles["urgR0.rtc"]
    rtc_urgL = env.name_space[ns_urg].rtc_handles["urgL0.rtc"]
    self.rtc_urg_dic = {"r": rtc_urgR, "l": rtc_urgL}
    
    
    rtc_rh = env.name_space[ns_hiro].rtc_handles["RobotHardware0.rtc"]
    rtc_handle.make_pipe(comp, rtc_urgR)
    rtc_handle.make_pipe(comp, rtc_urgL)

    pdict = {'dataport.dataflow_type' : 'pull'}
    for rtc_urg in [rtc_urgR, rtc_urgL]:
      rtc_urg.out_pipe['data'].con = rtc_handle.IOConnector([rtc_urg.out_pipe['data'].pipe_port,
                                                             rtc_urg.out_pipe['data'].port],
                                                            prop_dict = pdict)
      
      for port in rtc_urg.out_pipe:
        rtc_urg.out_pipe[port].connect()

  
    con = rtc_handle.IOConnector([rtc_rh.outports["jointDatOut"], rtc_urgR.inports["JointDatIn"]])
    con.connect()
    con = rtc_handle.IOConnector([rtc_rh.outports["jointDatOut"], rtc_urgL.inports["JointDatIn"]])
    con.connect()
    
    time.sleep(0.1)
    rtc_urgR.activate()
    rtc_urgL.activate()
    
    rtc_urgR.services["svc"].provided["if"].narrow_ref(globals())
    urgR_provider = rtc_urgR.services["svc"].provided["if"].ref

    rtc_urgL.services["svc"].provided["if"].narrow_ref(globals())
    urgL_provider = rtc_urgR.services["svc"].provided["if"].ref

    self.urg_provider_dic = {"r": urgR_provider, "l": urgL_provider}

  def start_log(self, LR):
    self.urg_provider_dic[LR.lower()].start_log()

      
  def return_log(self, LR):
    return self.urg_provider_dic[LR.lower()].return_log()


  def capture(self, LR):
    data = np.array(self.rtc_urg_dic[LR.lower()].out_pipe["data"].pipe.read().data)
    num_scan = int(len(data) / 12)
    return data.reshape((num_scan, 12))
