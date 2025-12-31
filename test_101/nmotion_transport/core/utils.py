import os
import sys
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(os.path.join(dir_path, "./lib")))

import libnmotion_transport_python as nmotion_transport

def getCurrentLibraryVersion():
    return nmotion_transport.__version__