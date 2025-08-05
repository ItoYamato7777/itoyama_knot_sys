#!/usr/bin/env python
#python module
import threading
import math
import numpy as np
from time import sleep, time

#local python file
import file_io
from robot_servo import serial_converter, futaba_servo, servo_cluster
from geo import FRAME
from math import pi, radians
from kinematics_hand import *
import finger_motion

#motion generator
mg_2fing_grasp = finger_motion.grasping_by_two_fingers(fing_len, b_len)
mg = finger_motion.one_finger_motion(fing_len)
default_zero_angle = np.array([0, -54, -120, 310, -54, -120, -310, -54, -120])

_hand_motion_folder = file_io.read_txt("hand_motion_lib.txt").replace("\n", "")


class hiroHand(servo_cluster):
  def __init__(self, servo_list):
    super(hiroHand, self).__init__(servo_list, [[0,3,6],[1,4,7],[2,5,8]])

    
  def set_compliance_mode(self, mode, finger=[0,1,2]):
    """
    set compliance parameter of fingertip servos.
    mode: 0 -> Position control
          1, 2 -> Compliance Control
    fing: list of finger id to apply the mode.
    """
    servo_id = (np.array(finger)*3 +2).tolist()
    if mode == 0:
      self.set_compliance_param(servo_id, 2, 2 ,10, 10, 180)
      
    elif mode == 1:
      self.set_compliance_param(servo_id, 3, 3 ,30, 30, 205)
      
    elif mode == 2:
      self.set_compliance_param(servo_id, 3, 3 ,100, 100, 205)
    
  
  def load_motion(self, motion, waitflag = False):
    def _thread():
      _file = _hand_motion_folder + '/' + motion + ".txt"
      data = file_io.read_csv(_file, dataType = int)
      for l in data:
        if (len(l) == 9) or not(isinstance(l[9], int)):
          self.move(l[0:9],20)
          sleep(0.2)
          
        elif isinstance(l[9], int):
          self.move(l[0:9], l[9])
          wait_time = l[9]/100.0
          sleep(wait_time)

    th=threading.Thread(target = _thread)
    th.start()
    if waitflag:
      th.join()

  def calc_param_2fing_grasp(self, fingers, dist, slide, direction, opening_angle):
    """
    fingers: "01" or "12" or "20"
    dist[m]
    slide[m]
    direction[rad]
    opening_angle[rad]
    """
    th1b, th1c, th2b, th2c, frm = mg_2fing_grasp.calc_finger_angles(dist, slide, direction, opening_angle)
    if str(fingers) == "01":
      t_wrist2tip = t_wrist2fing01 * frm
      return [300, th1b, th1c, -300, th2b, th2c, None, None, None], t_wrist2tip
    elif str(fingers) == "12":
      t_wrist2tip = t_wrist2fing12 * frm
      return [None, None, None, 300, th1b, th1c, -300, th2b, th2c], t_wrist2tip
    elif str(fingers) == "20":
      t_wrist2tip = t_wrist2fing20 * frm
      return [-300, th2b, th2c, None, None, None, 300, th1b, th1c], t_wrist2tip
    else:
      raise ValueError

  def calc_param_2fing_grasp2(self, fingers, dist, slide, direction, dist2):
    """
    fingers: "01" or "12" or "20"
    dist[m]
    slide[m]
    direction[rad]
    dist2[m]
    """
    th1b, th1c, th2b, th2c, frm = mg_2fing_grasp.calc_finger_angles2(dist, slide, direction, dist2)
    if str(fingers) == "01":
      t_wrist2tip = t_wrist2fing01 * frm
      return [300, th1b, th1c, -300, th2b, th2c, None, None, None], t_wrist2tip
    elif str(fingers) == "12":
      t_wrist2tip = t_wrist2fing12 * frm
      return [None, None, None, 300, th1b, th1c, -300, th2b, th2c], t_wrist2tip
    elif str(fingers) == "20":
      t_wrist2tip = t_wrist2fing20 * frm
      return [-300, th2b, th2c, None, None, None, 300, th1b, th1c], t_wrist2tip
    else:
      raise ValueError

    
ttyUSB0 = serial_converter(Port = '/dev/ttyUSB0', Baudrate = 115200)
ttyUSB1 = serial_converter(Port = '/dev/ttyUSB1', Baudrate = 115200)
ttyUSB2 = serial_converter(Port = '/dev/ttyUSB2', Baudrate = 115200)

r_hand_servos = [futaba_servo(ttyUSB0, i) for i in range(1,10)]
l_hand_servos = [futaba_servo(ttyUSB1, i) for i in range(10,19)]
t_hand_servos = [futaba_servo(ttyUSB2, i) for i in range(19,28)]

#r_hand_urg = serial_URG(port ='/dev/ttyACM0', baudrate = 900000)
#l_hand_urg = serial_URG(port ='/dev/ttyACM1', baudrate = 900000)

r_hand = hiroHand(r_hand_servos)#, r_hand_urg)
l_hand = hiroHand(l_hand_servos)#, l_hand_urg)
t_hand = hiroHand(t_hand_servos)
                 
r_hand.set_zero_position(default_zero_angle + np.array([ 40,-36, 10, 40,  4,  6, 60,-16, 10]))
l_hand.set_zero_position(default_zero_angle + np.array([  0,  0,  0, 20, 25,-15, 10,-25,  5]))
t_hand.set_zero_position(np.array([  0, 450, -450,  0, 450, -450,  0, 450, -450]))

hand_list = [r_hand, l_hand]

def activate():
  for hand in hand_list:
    hand.deactivate()
  thread_list = []
  for hand in hand_list:
    thread_list.append(threading.Thread(target = hand.activate))
  for th in thread_list:
    th.start()
    
def deactivate():
  r_hand.torque(1)
  l_hand.torque(1)
  r_hand.move([0,0,0,0,0,0,0,0,0], 300)
  l_hand.move([0,0,0,0,0,0,0,0,0], 300)
  sleep(3.0)
  r_hand.move([ 600, -900, -900, 1200, -900, -900, -1050, -900, -900], 50)
  l_hand.move([-600, -900, -900, 1050, -900, -900, -1200, -900, -900], 50)
  sleep(1.0)
  r_hand.torque(0)
  l_hand.torque(0)
  
def teach_motion(hand_teaching, hand_playback = None, teaching_time = 10):
  _logger = file_io.writer(_hand_motion_folder+"/motion.txt", True)

  if hand_playback != None:
    hand_teaching.torque(1)
    hand_teaching.move(hand_playback.get_status()["Angle"], 200)
    sleep(2.0)

  hand_teaching.torque(2)

  def _thread():
    thread_start_time = time()
    current_time = time()
    i = 1
    

    while time()-thread_start_time < teaching_time:
      print i
      i = i+1
      _list = hand_teaching.get_status()["Angle"]


      if hand_playback != None:
        hand_playback.move(_list, 20)
        old_time = current_time
        current_time = time()
        wait_time = 0.2 - current_time + old_time

        if wait_time > 0:
          sleep(wait_time)
          
      _list.append('')
      _logger.csv_write(_list)
      

  th = threading.Thread(target = _thread)
  th.start()
  sleep(teaching_time)
  th.join()
  if hand_playback != None:
    hand_teaching.torque(1)



### for Twisting Movement ###
def calc_twisting_finger_angle(dist, slide): 
  """
  dist: distance between each finger tips (center to center)
  slide: z_finger3 - z_finger2

  when twisting a rope (diameter: D)
  theta_rope = D * slide
  dist = D + Diameter_of_fingertip
  """
  print "----------!!WARNING!!----------"
  print "This function will be removed."
  print "use hand.calc_param_2fing_grasp"
  dist = dist * 0.001 #mm -> m
  slide = slide * 0.001 #mm -> m
  th1b, th1c, th2b, th2c, frm = mg_2fing_grasp.calc_finger_angles(dist, slide, 0, 0)
  return [None, None, None, 300, th1b, th1c, -300, th2b, th2c], frm
