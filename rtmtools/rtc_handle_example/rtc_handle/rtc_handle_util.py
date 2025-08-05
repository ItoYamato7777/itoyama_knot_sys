#!/usr/bin/env python
# -*- Python -*-

# import sys
import time
import subprocess

#
# import user tools
#    this utility uses an instance of RtmEnv (env) as the data keeper of the system
#
from rtc_handle import *

#
# run components
#
#  c_seq:seq of key for os_command, sleep time or python expressions.
#
def run_components(env,c_seq=None) :
  if c_seq :
    for tmp in c_seq :
      if tmp in env.starters.keys() :
        subprocess.Popen(env.starters[tmp], shell=True)
      elif isinstance(tmp, int) :
        time.sleep(tmp)
      elif isinstance(tmp, float) :
        time.sleep(tmp)
    else :
      eval(tmp)
  else :
    for tmp in env.starters.keys() :
      subprocess.Popen(env.starters[tmp], shell=True)
#
# connection control
#
#  con_seq:seq of key for connertor, connector, sleep time or python expression.
#
def connect_seq(env,con_seq=None) :
  if con_seq :
    for tmp in con_seq :
      if tmp in env.connectors.keys() :
        env.connectors[tmp].connect()
      elif isinstance(tmp, Connector) :
        tmp.connect()
      elif isinstance(tmp, int) :
        time.sleep(tmp)
      elif isinstance(tmp, float) :
        time.sleep(tmp)
      else :
        eval(tmp)
  else :
    for tmp in env.connectors.keys() :
      env.connectors[tmp].connect()
#
def disconnect_seq(env,con_seq=None) :
  if con_seq :
    for tmp in con_seq :
      if tmp in env.connectors.keys() :
        env.connectors[tmp].disconnect()
      elif isinstance(tmp, Connector) :
        tmp.disconnect()
      elif isinstance(tmp, int) :
        time.sleep(tmp)
      elif isinstance(tmp, float) :
        time.sleep(tmp)
      else :
        eval(tmp)
  else :
    for tmp in env.connectors.keys() :
      env.connectors[tmp].disconnect()
#
# activation
#
#  ac_seq:seq of key for component, component, sleep time or python expression.
#
def activate_seq(env, a_seq=None) :
  if a_seq :
    for tmp in a_seq :
      if tmp in env.rtc_dict.keys() :
        env.rtc_dict[tmp].activate()
      elif isinstance(tmp,RtcHandle) :
        tmp.activate()
      elif isinstance(tmp, int) :
        time.sleep(tmp)
      elif isinstance(tmp, float) :
        time.sleep(tmp)
      else :
        eval(tmp)
  else :
    for tmp in env.rtc_dict.keys() :
      env.rtc_dict[tmp].activate()
#
def deactivate_seq(env, a_seq=None) :
  if a_seq :
    for tmp in a_seq :
      if tmp in env.rtc_dict.keys() :
        env.rtc_dict[tmp].deactivate()
      elif isinstance(tmp,RtcHandle) :
        tmp.deactivate()
      elif isinstance(tmp, int) :
        time.sleep(tmp)
      elif isinstance(tmp, float) :
        time.sleep(tmp)
      else :
        eval(tmp)
  else :
    for tmp in env.rtc_dict.keys() :
      env.rtc_dict[tmp].deactivate()
