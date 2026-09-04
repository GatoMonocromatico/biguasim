"""Shared memory with memory mapping"""
import ctypes
import mmap
import os

import numpy as np

from functools import reduce
from biguasim.exceptions import BiguaSimException
from biguasim.util import HyData


class Shmem:
    """Implementation of shared memory


    Args:
        name (:obj:`str`): Name the points to the beginning of the shared memory block
        shape (:obj:`int`): Shape of the memory block
        dtype (type, optional): data type of the shared memory. Defaults to np.float32
        uuid (:obj:`str`, optional): UUID of the memory block. Defaults to ""
    """
    _map_types = {
        np.float32: ctypes.c_float,
        np.uint8: ctypes.c_uint8,
        np.uint32: ctypes.c_uint32,
        np.bool_: ctypes.c_bool,
        np.byte: ctypes.c_byte,
        HyData : np.uint8
    }

    def __init__(self, name, shape, dtype=np.float32, uuid=""):
        self.shape = shape
        self.dtype = dtype
        size = 0 if Shmem._map_types[dtype] == np.uint8 else reduce(lambda x, y: x * y, shape)
        size_bytes = 1024 if Shmem._map_types[dtype] == np.uint8 else np.dtype(dtype).itemsize * size  

        self._mem_path = None
        self._mem_pointer = None
        if os.name == "nt":
            self._mem_path = "/HOLODECK_MEM" + uuid + "_" + name
            self._mem_pointer = mmap.mmap(0, size_bytes, self._mem_path)
        elif os.name == "posix":
            self._mem_path = "/dev/shm/HOLODECK_MEM" + uuid + "_" + name
            # f = os.open(self._mem_path, os.O_CREAT | os.O_RDWR)

            # self._file_size = os.fstat(f).st_size    
            # if self._file_size > 0:        
            f = os.open(self._mem_path, os.O_CREAT | os.O_TRUNC | os.O_RDWR)
            self._mem_file = f
            os.ftruncate(f, size_bytes)
            os.fsync(f)

            # TODO - I think we are leaking a file object here. Unfortunately, we
            #        can't just .close() it since numpy acquires a reference to it
            #        below and I can't find a way to release it in __linux_unlink__()
            self._mem_pointer = mmap.mmap(f, size_bytes)
            # print('TESTE: ', self._mem_pointer.read().decode("utf-8").strip("\x00"))
        else:
            raise BiguaSimException("Currently unsupported os: " + os.name)


        if Shmem._map_types[dtype] == np.uint8:
            base = np.ndarray(shape=(1024, ), dtype=np.uint8, buffer=self._mem_pointer)
        else:
            base = np.ndarray(shape, dtype=dtype, buffer=(Shmem._map_types[dtype] * size).from_buffer(self._mem_pointer))

        self.np_array = base




    def unlink(self):
        """unlinks the shared memory"""
        if os.name == "posix":
            self.__linux_unlink__()
        elif os.name == "nt":
            self.__windows_unlink__()
        else:
            raise BiguaSimException("Currently unsupported os: " + os.name)

    def __linux_unlink__(self):
        os.close(self._mem_file)
        os.remove(self._mem_path)

    def __windows_unlink__(self):
        pass
