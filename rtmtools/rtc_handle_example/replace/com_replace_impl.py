#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 \file com_replace_idl_examplefile.py
 \brief Python example implementations generated from com_replace.idl
 \date $Date$


"""

import omniORB
from omniORB import CORBA, PortableServer
import _GlobalIDL, _GlobalIDL__POA


class ComReplace_i (_GlobalIDL__POA.ComReplace):
    """
    \class ComReplace_i
    Example class implementing IDL interface ComReplace
    """

    def __init__(self, repl_rtc):
        """
        \brief standard constructor
        Initialise member variables here
        """
        self.rtc=repl_rtc

    # int count_of_replaced_substring()
    def replace_count(self):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result
        return self.rtc.repl_count

if __name__ == "__main__":
    import sys
    
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv)
    
    # As an example, we activate an object in the Root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Create an instance of a servant class
    servant = ComReplace_i()

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

