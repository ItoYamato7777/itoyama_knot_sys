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
#import _GlobalIDL

#
# define util functions
#
def make_starters(env) :
  env.starters={}
#
def make_rtc_dict(env) :
  env.rtc_dict={}
#
#
def assign_variables(env) :
  global the_rtc
# rtc
#
def make_connectors(env):
  env.connectors={}
#
# user program 
# 

#
env = RtmEnv(sys.argv,[NS0,NS1])
#make_starters(env)
#run_components(env)
#time.sleep(10)
raw_input("are you sure that each rtc needed is running?")
for ns in env.name_space :
  env.name_space[ns].list_obj()
make_rtc_dict(env)
#assign_variables(env)
#make_connectors(env)
#
#
#
mgr=main()
comp=mgr.getComponents()[0]
#make_pipe(comp,cin)
#make_pipe(comp,cout)
