import os
import sys
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(os.path.join(dir_path, "./lib")))

# import importlib
# py_major_version = sys.version_info[0]
# py_minor_version = sys.version_info[1]
# globals()["nmotion_transport"] = importlib.import_module(f'libnmotion_transport_python_{py_major_version}{py_minor_version}')
# globals()["nmotion_transport"] = importlib.import_module(f'libnmotion_transport_python')

import libnmotion_transport_python as nmotion_transport
from typing import Any

class CANInterface():
    """! Interface class for working with native CAN Interfaces."""    
    
    def __init__(self, interface_name: str):
        """! CANInterface class constructor.
        
        @param interface_name Interface identifier string.
        """
        self.__iface = nmotion_transport.CANInterface()
        try:
            status = self.__iface.initInterface(interface_name)
            if(not status):
                raise Exception("")
        except Exception as error:
            print(f'Error during interface initialisation, please recheck the port {interface_name} and restart')
            return
        self.__name = interface_name
        
    def getInterface(self) -> Any:
        """!
        Get initialialised interface object.
        
        @return Low-Level interface object which is the wrapper for C++ interface object.
        """
        return self.__iface
    
    def getName(self) -> str:
        """!
        Get initialised interface name.
        
        @return Name of the initialised interface.
        """
        return self.__name
    
    def getConnectedDevices(self) -> 'list[int]':
        """!
        Get connected devices on the interface.
        
        @return List of CAN Node IDs of connected devices.
        """
        if(self.__iface):
            return self.__iface.getConnectedDevices()
        else:
            return []
    
    def close(self):
        """!
        Close the interface
        """
        if(self.__iface):
            self.__iface.closeInterface()    