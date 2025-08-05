ns_ensenso = "DT99B03:2809"
# ns_ensenso = "192.168.7.193:2809"
ns_list = [ns_ensenso]

import sys
import os
import time
import numpy as np
import cv2
import threading

def read_txt(fileName):
  f = open(fileName)
  return f.read()

path_rtmtools =  read_txt("./rtmtools.txt").replace("\n", "")

sys.path.append(path_rtmtools + "/rtc_handle")
sys.path.append(path_rtmtools + "/embryonic_rtc")

import rtc_handle
import rtc_handle_util
import EmbryonicRtc

import svcEnsenso

def run_embryonic_rtc():
  mgr = EmbryonicRtc.main()
  comp = mgr.getComponents()[0]
  return comp


def get_rtm_env():
  argv = sys.argv + ["-ORBgiopMaxMsgSize", "104857600"]
  env = rtc_handle.RtmEnv(argv, ns_list)
  for ns in env.name_space:
    env.name_space[ns].list_obj()
  return env

class C_ensenso():
  def __init__(self, env, comp):
    # make rtc list
    print(env.name_space[ns_ensenso].rtc_handles)
    self.rtc_ensenso = env.name_space[ns_ensenso].rtc_handles["Ensenso0.rtc"]
      
    env.rtc_dict = {}
    env.rtc_dict["Ensenso"] = self.rtc_ensenso
    
    rtc_handle.make_pipe(comp, self.rtc_ensenso)
    
    #set IOConnector dataflow_type push -> pull 
    pdict = {'dataport.dataflow_type' : 'pull'}
    for p in ['color_out',
              'ir_L_out',
              'ir_R_out',
              'pointMap_out',
              'renderPointMap_out',
              'renderPointMapTexture_out',
              'pointCloud_out']:
      self.rtc_ensenso.out_pipe[p].con = rtc_handle.IOConnector([self.rtc_ensenso.out_pipe[p].pipe_port,
                                                                 self.rtc_ensenso.out_pipe[p].port],
                                                                prop_dict = pdict)
    
    # connect_pipe
    for port in self.rtc_ensenso.out_pipe:
      self.rtc_ensenso.out_pipe[port].connect()
      
    self.rtc_ensenso.activate()
    
    self.rtc_ensenso.services["EnsensoProviderPort"].provided["EnsensoProvider"].narrow_ref(globals())
    self.ensenso_provider = self.rtc_ensenso.services["EnsensoProviderPort"].provided["EnsensoProvider"].ref
    
    #read port
    # dictionaries
    # dtype
    self.dic_dtype = {'color_out': np.uint8,
                 'ir_L_out' : np.uint8,
                 'ir_R_out' : np.uint8,
                 'renderPointMapTexture_out': np.uint8,
                 'pointMap_out':       np.float32,
                 'renderPointMap_out': np.float32
    }
    # channels
    self.dic_ch = {'color_out': 3,
              'ir_L_out' : 1,
              'ir_R_out' : 1,
              'renderPointMapTexture_out': 4,
              'pointMap_out': 3,
              'renderPointMap_out': 3
    }
      
  def capture(self, port):
    #initialize
    if isinstance(port, str):
      port = [port]
      str_flag = True
    else:
      str_flag = False
    for i in range(len(port)):
      if port[i][-4:] != "_out":
        port[i]+="_out"
    timestamp_list = [None for i in port]
    timestamp_list.append(time.time())
    #print time.time()
    data = [None for i in port]
    #getting data
    while True:
      for i in range(len(port)):
        if data[i] is None or timestamp_list[i] != max(timestamp_list):
          while True:
            data[i] = self.rtc_ensenso.out_pipe[port[i]].pipe.read()
            timestamp_list[i] = data[i].tm.sec + data[i].tm.nsec/1000000000.
            if isinstance(data[i].pixels, str):
              break
            else:
              time.sleep(0.01)
      #check if timestamps are same value.
      ts_flag = True #initialize
      for tm in timestamp_list[0:-1]:
        ts_flag = ts_flag and (tm == max(timestamp_list))
      if ts_flag:
        break
    #convert img into np.array
    img_list = []
    for i in range(len(port)):
      img = np.fromstring(data[i].pixels, self.dic_dtype[port[i]]).reshape(data[i].height,
                                                                           data[i].width,
                                                                           self.dic_ch[port[i]])
      if port[i] == "color_out" or port[i] == "renderPointMapTexture_out":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
      img_list.append(img)
    if str_flag:
      return img_list[0], timestamp_list[0]
    else:
      return img_list, timestamp_list[0]
  
  
  def show_all_images(self):
    color = self.capture("color")[0]
    ir = self.capture(["ir_L", "ir_R"])[0]
    pm = self.capture("pointMap")[0]
    rpm = self.capture(["renderPointMap", "renderPointMapTexture"])[0]
    cv2.imshow("color", color)
    cv2.imshow("ir_L", ir[0])
    cv2.imshow("ir_R", ir[1])
    cv2.imshow("pointMap",self.cvt_zaxis2color(pm))
    cv2.imshow("renderPointMap",self.cvt_zaxis2color(rpm[0]))
    cv2.imshow("renderPointMapTexture", rpm[1])
    cv2.waitKey(0)
    cv2.destroyAllWindows()
  
    
  def cvt_zaxis2color(self, pm):
    zaxis = pm[:,:,2]
    zmax = np.nanmax(zaxis)
    zmin = np.nanmin(zaxis)
    gray = (zaxis - zmin)/(zmax - zmin) * 255
    gray = gray.astype(np.uint8)
    color = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
    color[np.isnan(pm)] *= 0
    return color
  
  
  def set_projection_mode(self, mode):
    if mode.lower() == 'off':
      self.ensenso_provider.projectorConf(0)
    elif mode.lower() == 'projector' or mode.lower() == 'p':
      self.ensenso_provider.projectorConf(1)
    elif mode.lower() == 'led' or mode.lower() == 'l':
      self.ensenso_provider.projectorConf(2)
    else:
      print 'mode == "off" or "p"(projector) or "l"(led)'
      raise ValueError
  
    
  def switch_monitor(self, mode):
    if mode.lower() == 'off':
      self.ensenso_provider.switchMonitor(0)
    elif mode.lower() == 'on':
      self.ensenso_provider.switchMonitor(1)
    else:
      print 'mode == "on" or "off"'
      raise ValueError
  
  
  def get_rpm_params(self):
    param = self.ensenso_provider.getRpmParameters()
    return {"width": param.size0, "height": param.size1, "pixelsize": param.pixelsize}
  
  def set_rpm_params(self, pixelsize, width, height):
    param = svcEnsenso.RpmParameters(pixelsize, width, height)
    self.ensenso_provider.setRpmParameters(param)
  
  def set_target_brightness(self, param):
    ret = self.ensenso_provider.setTargetBrightness(param)
  
env = get_rtm_env()
#comp = run_embryonic_rtc()
#ensenso = C_ensenso(env, comp)
