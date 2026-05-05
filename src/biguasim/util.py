"""Helpful Utilities"""
import math
import os
import biguasim
import json
import subprocess

import numpy as np

from numpy.typing import NDArray

from typing import TypeVar
from multiprocessing import Process, Event

try:
    unicode        # Python 2
except NameError:
    unicode = str  # Python 3


def get_biguasim_version():
    """Gets the current version of biguasim

    Returns:
        (:obj:`str`): the current version
    """
    return biguasim.__version__

def _get_biguasim_folder():
    if "HOLODECKPATH" in os.environ and os.environ["HOLODECKPATH"] != "":
        return os.environ["HOLODECKPATH"]

    if os.name == "posix":
        return os.path.expanduser("~/.local/share/biguasim")

    if os.name == "nt":
        return os.path.expanduser("~\\AppData\\Local\\biguasim")

    raise NotImplementedError("biguasim is only supported for Linux and Windows")

def get_biguasim_path():
    """Gets the path of the biguasim environment

    Returns:
        (:obj:`str`): path to the current biguasim environment
    """

    return os.path.join(_get_biguasim_folder(), get_biguasim_version())


def convert_unicode(value):
    """Resolves python 2 issue with json loading in unicode instead of string

    Args:
        value (:obj:`str`): Unicode value to be converted

    Returns:
        (:obj:`str`): Converted string

    """
    if isinstance(value, dict):
        return {convert_unicode(key): convert_unicode(value)
                for key, value in value.iteritems()}

    if isinstance(value, list):
        return [convert_unicode(item) for item in value]

    if isinstance(value, unicode):
        return value.encode('utf-8')

    return value


def get_os_key():
    """Gets the key for the OS.

    Returns:
        :obj:`str`: ``Linux`` or ``Windows``. Throws ``NotImplementedError`` for other systems.
    """
    if os.name == "posix":
        return "Linux"
    if os.name == "nt":
        return "Windows"

    raise NotImplementedError("BiguaSim is only supported for Linux and Windows")


def human_readable_size(size_bytes):
    """Gets a number of bytes as a human readable string.

    Args:
        size_bytes (:obj:`int`): The number of bytes to get as human readable.

    Returns:
        :obj:`str`: The number of bytes in a human readable form.
    """
    if size_bytes == 0:
        return "0B"
    size_name = ("B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB")
    base = int(math.floor(math.log(size_bytes, 1024)))
    power = math.pow(1024, base)
    size = round(size_bytes / power, 2)
    return "%s %s" % (size, size_name[base])

def gpu():
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.total,memory.free", "--format=csv,nounits,noheader"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        # Parse the result to get total and free VRAM in MB
        vram_info = [
            int(free) * 100 / int(total) for total, free in 
            (line.split(",") for line in result.stdout.strip().split("\n"))
        ]
        vram_ordered = list(map(lambda idx, pct: (str(idx), pct), range(len(vram_info)), vram_info))
        return sorted(vram_ordered, key= lambda pct: pct[-1])[0][0]
    
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

HyData = TypeVar('HyData')
class HyData:
    def __init__(self, sensor_data : np.ndarray | dict) -> None:
        print(sensor_data)
        if isinstance(sensor_data, np.ndarray):
            rbytes = sensor_data.tobytes()
            jstr = rbytes.split(b'\x00', 1)[0].decode('utf-8')
            
            if not len(jstr):
                self.update(json.loads('{}'))
            else:
                self.update(json.loads(jstr))
        else:
            self.update(sensor_data)


    @property
    def keys(self):
        return self.__dict__.keys()
    
    def __repr__(self):
        return str(self.__dict__)
    
    def __getitem__(self, index):
        return self.__dict__[index]
    
    def update(self, dictionary) -> HyData:
        if isinstance(dictionary, dict):
            for key, value in dictionary.items():
                if isinstance(value, dict):
                    setattr(self, key, HyData(value))
                else:
                    setattr(self, key, value)

            return self
        return self
