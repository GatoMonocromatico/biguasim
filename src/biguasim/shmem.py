"""Shared memory with memory mapping"""
import ctypes
import gc
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
        self._mem_file = None
        self._base = None
        self._size_bytes = size_bytes
        if os.name == "nt":
            self._mem_path = "/HOLODECK_MEM" + uuid + "_" + name
            self._mem_pointer = mmap.mmap(0, size_bytes, self._mem_path)
        elif os.name == "posix":
            self._mem_path = "/dev/shm/HOLODECK_MEM" + uuid + "_" + name
            # O_TRUNC resizes in place and keeps the inode, so an engine that already
            # mapped this path stays attached. Only unlink() breaks that sharing.
            f = os.open(self._mem_path, os.O_CREAT | os.O_TRUNC | os.O_RDWR)
            os.ftruncate(f, size_bytes)
            os.fsync(f)

            # mmap() dups the descriptor, so the original is dead weight once the
            # mapping exists. Closing it is what stops a long-lived world process
            # burning two fds for every sensor it allocates.
            self._mem_pointer = mmap.mmap(f, size_bytes)
            os.close(f)
            # print('TESTE: ', self._mem_pointer.read().decode("utf-8").strip("\x00"))
        else:
            raise BiguaSimException("Currently unsupported os: " + os.name)


        if Shmem._map_types[dtype] == np.uint8:
            base = np.ndarray(shape=(1024, ), dtype=np.uint8, buffer=self._mem_pointer)
        else:
            # Held so unlink() can drop it; the ctypes buffer exports the mmap and
            # mmap.close() fails while any export is outstanding.
            self._base = (Shmem._map_types[dtype] * size).from_buffer(self._mem_pointer)
            base = np.ndarray(shape, dtype=dtype, buffer=self._base)

        self.np_array = base




    def clear(self):
        """Zero the buffer in place, keeping the mapping alive.

        This is what a world reset wants. It deliberately does not unlink: the
        engine holds its own mapping of this block, and replacing the file would
        silently detach the two sides from each other.
        """
        if self._mem_pointer is None or self.np_array is None:
            return
        self.np_array[...] = 0

    def close(self):
        """Drop this process's mapping, leaving the backing file in place.

        Safe while the engine still holds the block: the file, and therefore the
        inode both sides agreed on, survives. Used when a block is being
        reallocated at a new shape.
        """
        # The numpy array and the ctypes buffer beneath it export the mmap, and
        # mmap.close() raises BufferError while any export is outstanding.
        self.np_array = None
        self._base = None
        gc.collect()

        if self._mem_pointer is not None:
            try:
                self._mem_pointer.close()
            except BufferError:
                # Someone else still holds a view of this buffer. Leave the
                # mapping in place rather than corrupting their read.
                pass
            self._mem_pointer = None

        if self._mem_file is not None:
            try:
                os.close(self._mem_file)
            except OSError:
                pass
            self._mem_file = None

    def unlink(self):
        """Release the mapping and remove the backing file.

        Only safe once the engine has dropped its own mapping of this block --
        on the Python side that means after the tick which carries the matching
        RemoveSensor command. Unlinking early leaves the engine writing into an
        orphaned inode while Python reads a fresh, permanently empty one.
        """
        self.close()
        if os.name == "posix" and self._mem_path is not None:
            try:
                os.remove(self._mem_path)
            except FileNotFoundError:
                pass
