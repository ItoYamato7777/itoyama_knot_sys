#!/usr/bin/env python
# -*- Python -*-

import sys
import time
import subprocess

#
# set up user environment
#  RtmToolsDir, MyRtcDir, etc.
#
from set_env import *   

#
# import user tools
#
sys.path.append(".")
save_path = sys.path[:]
sys.path.append(RtmToolsDir+'/rtc_handle')
from rtc_handle import *
from rtc_handle_util import *
sys.path.append(RtmToolsDir+'/embryonic_rtc')
from EmbryonicRtc import *
sys.path = save_path

#
# import stub files
# 
import _GlobalIDL

#
# define util functions
#
def make_starters(env) :
  env.starters={}
  env.starters['cin']="cd "+MyRtcDir+"/cin; xterm -e python cin.py"
  env.starters['cout']="cd "+MyRtcDir+"/cout; xterm -e python cout.py"
  env.starters['relplace']="cd "+MyRtcDir+"/replace; xterm -e python replace.py"
#
def make_rtc_dict(env) :
  env.rtc_dict={}
  env.rtc_dict['cin'] = env.name_space[NS0].rtc_handles['cin0.rtc']
  env.rtc_dict['cout'] = env.name_space[NS0].rtc_handles['cout0.rtc']
  env.rtc_dict['replace'] = env.name_space[NS0].rtc_handles['replace0.rtc']
#
#
def assign_variables(env) :
  # rtc
  global cin, cout, replace, cc
  cin = env.rtc_dict['cin']
  cout = env.rtc_dict['cout']
  replace = env.rtc_dict['replace']
  # narrow services
  replace.services['sv_replace'].provided['com_replace'].narrow_ref(globals())
  cc =replace.services['sv_replace'].provided['com_replace'].ref
#
def make_connectors(env):
  env.connectors={}
  con = IOConnector([cin.outports['str_out'], replace.inports['str_in']])
  env.connectors['cin-replace']=con
  con = IOConnector([replace.outports['str_out'], cout.inports['str_in']])
  env.connectors['replace-cout']=con
#
# user program 
# 
replace_conf='aaa bbb'
#
env = RtmEnv(sys.argv,[NS0])
make_starters(env)
run_components(env)
time.sleep(10)
raw_input("are you sure that each rtc needed is running?")
for ns in env.name_space :
  env.name_space[ns].list_obj()
make_rtc_dict(env)
assign_variables(env)
replace.set_conf_activate('default','replace_str',replace_conf)
make_connectors(env)
#
#
#
mgr=main()
comp=mgr.getComponents()[0]
make_pipe(comp,cin)
make_pipe(comp,cout)
make_pipe(comp,replace)
