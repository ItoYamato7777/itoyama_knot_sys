#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 \file replace.py
 \brief replace substring
 \date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

# Import Service implementation class
# <rtc-template block="service_impl">
# from com_replace_idl_example import *
from com_replace_impl import *

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
replace_spec = ["implementation_id", "replace", 
		 "type_name",         "replace", 
		 "description",       "replace substring", 
		 "version",           "1.0.0", 
		 "vendor",            "sue", 
		 "category",          "filte", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 "conf.default.replace_str", "aho baka",
		 "conf.__widget__.replace_str", "text",
		 ""]
# </rtc-template>

class replace(OpenRTM_aist.DataFlowComponentBase):
	
	"""
	\class replace
	\brief replace substring
	
	"""
	def __init__(self, manager):
		"""
		\brief constructor
		\param manager Maneger Object
		"""
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		self._d_str_in = RTC.TimedString(RTC.Time(0,0),0)
		"""
		"""
		self._str_inIn = OpenRTM_aist.InPort("str_in", self._d_str_in)
		self._d_str_out = RTC.TimedString(RTC.Time(0,0),0)
		"""
		"""
		self._str_outOut = OpenRTM_aist.OutPort("str_out", self._d_str_out)

		"""
		"""
		self._sv_replacePort = OpenRTM_aist.CorbaPort("sv_replace")

		"""
		"""
		self._com_replace = ComReplace_i(self)
		self.repl_count=0


		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		"""
		
		 - Name:  replace_str
		 - DefaultValue: aho baka
		"""
		self._replace_str = ['aho baka']
		
		# </rtc-template>


		 
	def onInitialize(self):
		"""
		
		The initialize action (on CREATED->ALIVE transition)
		formaer rtc_init_entry() 
		
		\return RTC::ReturnCode_t
		
		"""
		# Bind variables and configuration variable
		self.bindParameter("replace_str", self._replace_str, "aho, baka")
		
		# Set InPort buffers
		self.addInPort("str_in",self._str_inIn)
		
		# Set OutPort buffers
		self.addOutPort("str_out",self._str_outOut)
		
		# Set service provider to Ports
		self._sv_replacePort.registerProvider("com_replace", "ComReplace", self._com_replace)
		
		# Set service consumers to Ports
		
		# Set CORBA Service Ports
		self.addPort(self._sv_replacePort)
		
		return RTC.RTC_OK
	
	#def onFinalize(self, ec_id):
	#	"""
	#
	#	The finalize action (on ALIVE->END transition)
	#	formaer rtc_exiting_entry()
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onStartup(self, ec_id):
	#	"""
	#
	#	The startup action when ExecutionContext startup
	#	former rtc_starting_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onShutdown(self, ec_id):
	#	"""
	#
	#	The shutdown action when ExecutionContext stop
	#	former rtc_stopping_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	def onActivated(self, ec_id):
		"""
	
		The activated action (Active state entry action)
		former rtc_active_entry()
	
		\param ec_id target ExecutionContext Id
	
		\return RTC::ReturnCode_t
	
		"""
		self.repl_count=0	
		tmp=self._replace_str[0].split()
		self.repl_str=[tmp[0],tmp[-1]]
		print self.repl_str
		return RTC.RTC_OK
	
	#def onDeactivated(self, ec_id):
	#	"""
	#
	#	The deactivated action (Active state exit action)
	#	former rtc_active_exit()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	def onExecute(self, ec_id):
		"""
	
		The execution action that is invoked periodically
		former rtc_active_do()
	
		\param ec_id target ExecutionContext Id
	
		\return RTC::ReturnCode_t
	
		"""
		if self._str_inIn.isNew() :
			tmp=self._str_inIn.read().data
			self.repl_count=self.repl_count+tmp.count(self.repl_str[0])
			self._d_str_out.data=tmp.replace(self.repl_str[0],self.repl_str[1])
			self._str_outOut.write()
		return RTC.RTC_OK
	
	#def onAborting(self, ec_id):
	#	"""
	#
	#	The aborting action when main logic error occurred.
	#	former rtc_aborting_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onError(self, ec_id):
	#	"""
	#
	#	The error action in ERROR state
	#	former rtc_error_do()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onReset(self, ec_id):
	#	"""
	#
	#	The reset action that is invoked resetting
	#	This is same but different the former rtc_init_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onStateUpdate(self, ec_id):
	#	"""
	#
	#	The state update action that is invoked after onExecute() action
	#	no corresponding operation exists in OpenRTm-aist-0.2.0
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onRateChanged(self, ec_id):
	#	"""
	#
	#	The action that is invoked when execution context's rate is changed
	#	no corresponding operation exists in OpenRTm-aist-0.2.0
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	



def replaceInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=replace_spec)
    manager.registerFactory(profile,
                            replace,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    replaceInit(manager)

    # Create a component
    comp = manager.createComponent("replace")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()

