#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file UrgService_idl_examplefile.py
 @brief Python example implementations generated from UrgService.idl
 @date $Date$


"""

import omniORB
from omniORB import CORBA, PortableServer
import svcUrg, svcUrg__POA

import time
import file_io


class UrgService_i (svcUrg__POA.UrgService):
  """
  @class UrgService_i
  Example class implementing IDL interface svcUrg.UrgService
  """
  
  def __init__(self):
    """
    @brief standard constructor
    Initialise member variables here
    """
    self.urg = None
    self.log_state = "OFF"
    self.writer = None # initialize
    self.logfile_name = "URG_R_log.csv"


  # void start_log(out double start_time)
  def start_log(self):
    self.writer = file_io.writer(self.logfile_name, overwrite = True)
    self.log_state = "ON"
    self.log_count = -1
    while self.log_count == -1:
      time.sleep(0.001)
    return time.time()

  
  # void return_log(out log_data data)
  def return_log(self):
    self.log_state = "OFF"
    while self.log_count != -1:
      time.sleep(0.001)
    return file_io.read_csv(self.logfile_name, float)

  
  # void init_timestamp(out boolean finished)
  def init_timestamp(self):
    if self.urg is None:
      return False

    print "init timestamp"
    self.urg.init_timestamp()
    return True


if __name__ == "__main__":
  import sys
  
  # Initialise the ORB
  orb = CORBA.ORB_init(sys.argv)
  
  # As an example, we activate an object in the Root POA
  poa = orb.resolve_initial_references("RootPOA")

  # Create an instance of a servant class
  servant = UrgService_i()

  # Activate it in the Root POA
  poa.activate_object(servant)

  # Get the object reference to the object
  objref = servant._this()
  
  # Print a stringified IOR for it
  print orb.object_to_string(objref)

  # Activate the Root POA's manager
  poa._get_the_POAManager().activate()

  # Run the ORB, blocking this thread
  orb.run()

