# -*- coding: utf-8 -*-
ns_hiro = "hiro022"
ns_jc = "localhost:2809"
ns_list = [ns_hiro, ns_jc]

import sys
import subprocess
import os
import time
import numpy as np
from math import radians, pi

from geo import FRAME, MATRIX, VECTOR
from kinematics_hiro import solve_ik, solve_fk

def read_txt(fileName):
  f = open(fileName)
  return f.read()

path_rtmtools =  read_txt("./rtmtools.txt").replace("\n", "")
path_IncludeFiles = read_txt("./includefiles.txt").replace("\n", "")

## run rtc ##
## JointControl  ## 
# os.popen2("cd " + path_IncludeFiles + "/hiroControlRTC/HiroJointControl; xterm -e python JointControl.py")

# JointControl.py があるディレクトリパス
jc_dir = os.path.join(path_IncludeFiles, "hiroControlRTC", "HiroJointControl")

# JointControl.pyをバックグラウンドプロセスとして起動
# stdoutとstderr（標準出力と標準エラー出力）をログファイルにリダイレクトする
log_file = open("joint_control.log", "w")
process = subprocess.Popen(
    ["python", "JointControl.py"],
    cwd=jc_dir,  # 作業ディレクトリを指定
    stdout=log_file,
    stderr=subprocess.STDOUT
)

print "JointControl.py をプロセスID: %d で起動しました。ログは urgR.log を参照してください。" % process.pid

time.sleep(3)

sys.path.append(path_rtmtools + "/rtc_handle")
sys.path.append(path_rtmtools + "/embryonic_rtc")
 
import rtc_handle
import rtc_handle_util
import EmbryonicRtc

import myHRP

argv = sys.argv + ["-ORBgiopMaxMsgSize", "104857600000"]
env = rtc_handle.RtmEnv(argv, ns_list)

for ns in env.name_space:
  env.name_space[ns].list_obj()

rtc_jc = env.name_space[ns_jc].rtc_handles["JointControl0.rtc"]
rtc_lepR = env.name_space[ns_jc].rtc_handles["lepR0.rtc"]
rtc_lepL = env.name_space[ns_jc].rtc_handles["lepL0.rtc"]
rtc_sa = env.name_space[ns_hiro].rtc_handles["sa.rtc"]
rtc_rh = env.name_space[ns_hiro].rtc_handles["RobotHardware0.rtc"]

#hiro_c_prop={
#  'dataport.subscription_type':'new',
#  'dataport.publisher.push_policy':'new',
#  'dataport.inport.buffer.length':'1',
#  'dataport.inport.buffer.read.empty_policy':'block',
#  'dataport.inport.buffer.write.full_policy':'block',
#  'dataport.outport.buffer.length' : '1',
#  'dataport.outport.buffer.write.full_policy':'block',
#  'dataport.outport.buffer.read.empty_policy':'block'
#}

hiro_c_prop={
  'dataport.subscription_type':'new',
  'dataport.publisher.push_policy':'new',
  'dataport.inport.buffer.length':'1',
  'dataport.outport.buffer.length' : '1',
}


con = rtc_handle.IOConnector([rtc_lepR.outports["sens_data"], rtc_jc.inports["forceR"]])
con.connect()
con = rtc_handle.IOConnector([rtc_lepL.outports["sens_data"], rtc_jc.inports["forceL"]])
con.connect()
con = rtc_handle.IOConnector([rtc_rh.outports["jointDatOut"], rtc_jc.inports["jointDatIn"]], prop_dict = hiro_c_prop)
con.connect()
con = rtc_handle.IOConnector([rtc_jc.outports["jointDatOut"], rtc_sa.inports["jointDatIn3"]], prop_dict = hiro_c_prop)

con.connect()
rtc_lepR.activate()
rtc_lepL.activate()
rtc_jc.activate()

rtc_jc.services["svc"].provided["if"].narrow_ref(globals())
jc_provider = rtc_jc.services["svc"].provided["if"].ref


def start_log():
  return jc_provider.start_log()
  
def return_log():
  return jc_provider.return_log()

def calc_force_control_init_param(log):
  wrist2lep = np.array([[0., 0., -1.],
                        [0., 1.,  0.],
                        [1., 0.,  0.]])
  # Ax = b
  A_R = []
  b_R = []
  A_L = []
  b_L = []
  for d in log:
    frms = solve_fk(d[0])
    rotR = np.dot(frms["R_arm"][0:3,0:3], wrist2lep) # rotation base2lepR
    rotL = np.dot(frms["L_arm"][0:3,0:3], wrist2lep) # rotation base2lepL
    grav_vecR = np.dot(rotR.T, np.array([[0],[0],[-1]])).reshape(3)
    grav_vecL = np.dot(rotL.T, np.array([[0],[0],[-1]])).reshape(3)
    A_R.append([1, 0, 0, 0, 0, 0, grav_vecR[0], 0, 0, 0])
    A_R.append([0, 1, 0, 0, 0, 0, grav_vecR[1], 0, 0, 0])
    A_R.append([0, 0, 1, 0, 0, 0, grav_vecR[2], 0, 0, 0])
    A_R.append([0, 0, 0, 1, 0, 0,     0,        0,         grav_vecR[2],  -grav_vecR[1]])
    A_R.append([0, 0, 0, 0, 1, 0,     0,   -grav_vecR[2],      0,          grav_vecR[0]])
    A_R.append([0, 0, 0, 0, 0, 1,     0,    grav_vecR[1], -grav_vecR[0],        0     ])
    
    A_L.append([1, 0, 0, 0, 0, 0, grav_vecL[0], 0, 0, 0])
    A_L.append([0, 1, 0, 0, 0, 0, grav_vecL[1], 0, 0, 0])
    A_L.append([0, 0, 1, 0, 0, 0, grav_vecL[2], 0, 0, 0])
    A_L.append([0, 0, 0, 1, 0, 0,     0,        0,         grav_vecL[2],  -grav_vecL[1]])
    A_L.append([0, 0, 0, 0, 1, 0,     0,   -grav_vecL[2],      0,          grav_vecL[0]])
    A_L.append([0, 0, 0, 0, 0, 1,     0,    grav_vecL[1], -grav_vecL[0],        0     ])
    
    b_R.append([d[1][0]])
    b_R.append([d[1][1]])
    b_R.append([d[1][2]])
    b_R.append([d[1][3]])
    b_R.append([d[1][4]])
    b_R.append([d[1][5]])
    
    b_L.append([d[2][0]])
    b_L.append([d[2][1]])
    b_L.append([d[2][2]])
    b_L.append([d[2][3]])
    b_L.append([d[2][4]])
    b_L.append([d[2][5]])

  AT_R = np.linalg.pinv(A_R)
  param_R = np.dot(AT_R, b_R).reshape(10)
  AT_L = np.linalg.pinv(A_L)
  param_L = np.dot(AT_L, b_L).reshape(10)
  return param_R, param_L


def set_force_control_param(param_R, param_L):
  jc_provider.init_force_control(list(param_R), list(param_L))


def start_force_control_R(rot = np.eye(3), coefficient=[3,3,3,0,0,0], target=[0,0,0,0,0,0]):
  if isinstance(rot, FRAME):
    rot = rot.mat
  #elif isinstance(rot, MATRIX): Do nothing
  elif type(rot) is np.ndarray:
    rot = rot[0:3,0:3]
  rot_rpy = MATRIX(rot).rpy()

  # make coefficient in range 0 - 10:
  tmp = -0.05 * 0.001 * 0.1 * np.array([0.7, 0.7, 0.7, 10, 10, 10])
  
  coefficient = (np.array(coefficient) * tmp).tolist()
  jc_provider.start_force_control( rot_rpy, coefficient, target, [], [], [])

  
def start_force_control_L(rot = np.eye(3), coefficient=[3,3,3,0,0,0], target=[0,0,0,0,0,0]):
  if isinstance(rot, FRAME):
    rot = rot.mat
  #elif isinstance(rot, MATRIX): Do nothing
  elif type(rot) is np.ndarray:
    rot = rot[0:3,0:3]
  rot_rpy = MATRIX(rot).rpy()

  # make coefficient in range 0 - 10:
  tmp = -0.05 * 0.001 * 0.1 * np.array([0.7, 0.7, 0.7, 10, 10, 10])
  
  coefficient = (np.array(coefficient) * tmp).tolist()
  jc_provider.start_force_control([], [], [], rot_rpy, coefficient, target)


def wait_force_control(f_R=None, f_L=None, t_R=None, t_L=None, p_R=None, p_L=None, r_R=None, r_L=None):
  '''
  f_R, f_L: tolerance of "f"orce  [N]
  t_R, t_L: tolerance of "t"orque [Nm]
  p_R, p_L: limit of "p"osition   [m]
  r_R, r_L: limit of "r"otation   [rad]
  '''
  lst = []
  for tmp in [f_R, f_L, t_R, t_L, p_R, p_L, r_R, r_L]:
    if tmp is None:
      tmp = -1
    lst.append(tmp)
  print lst
  return jc_provider.waitForceControl(lst)

  
def stop_force_control():
  jc_provider.stop_force_control(True, True)

def move_arm_linearly(target_R = None,
                      target_L = None,
                      t_wrist2hand_R=FRAME(xyzrpy=[0,0,0,0,0,0]),
                      t_wrist2hand_L=FRAME(xyzrpy=[0,0,0,0,0,0]),
                      target_CY = None,
                      speed=0.2,
                      wait=True):

  if isinstance(target_R, FRAME):
    r = target_R.xyzrpy()
  else:
    r = []
    
  if isinstance(target_L, FRAME):
    l = target_L.xyzrpy()
  else:
    l = []
  w2h_r = t_wrist2hand_R.xyzrpy()
  w2h_l = t_wrist2hand_L.xyzrpy()
  if target_CY is None:
    cy = []
  else:
    cy = [target_CY]
  tm = jc_provider.move_arm_linearly(r, l, w2h_r, w2h_l, cy, speed)
  if wait is True:
    jc_provider.waitPositionControl()
