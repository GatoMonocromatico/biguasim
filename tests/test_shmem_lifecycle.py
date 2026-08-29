"""Shared memory lifecycle.

These run without the engine: they exercise Shmem and BiguaSimClient directly.
Two families of invariant are covered, and they pull in opposite directions:

  * blocks that are released must actually go away (files, fds, table entries)
  * blocks that are *not* being released must keep their inode, because the
    engine holds its own mapping of that same file. Swapping the inode detaches
    the two sides silently -- Python then reads a fresh, permanently zero buffer
    while the engine writes into an orphan.
"""
import glob
import os

import numpy as np
import pytest

from biguasim.biguasimclient import BiguaSimClient
from biguasim.shmem import Shmem

CYCLES = 200
CAM_SHAPE = [256, 256, 4]  # RGBCamera-sized block, ~256KB


def shm_blocks():
    return len(glob.glob("/dev/shm/HOLODECK_MEM*"))


def open_fds():
    return len(os.listdir("/proc/{}/fd".format(os.getpid())))


def shm_path(key, uuid=""):
    return "/dev/shm/HOLODECK_MEM" + uuid + "_" + key


@pytest.fixture
def client():
    """A client with no engine behind it.

    __init__ opens the engine's semaphores, which do not exist here, so the
    allocation table is set up directly.
    """
    c = BiguaSimClient.__new__(BiguaSimClient)
    c._uuid = ""
    c._memory = dict()
    c._pending_free = []
    yield c
    for key in list(c._memory):
        c.free(key)


@pytest.fixture(autouse=True)
def _cleanup():
    yield
    for path in glob.glob("/dev/shm/HOLODECK_MEM*_lifecycle_*"):
        try:
            os.remove(path)
        except OSError:
            pass


def test_shmem_unlink_releases_file_and_fd():
    Shmem("lifecycle_warmup", CAM_SHAPE, np.uint8, "").unlink()

    blocks, fds = shm_blocks(), open_fds()
    for i in range(CYCLES):
        Shmem("lifecycle_A_{}".format(i), CAM_SHAPE, np.uint8, "").unlink()

    assert shm_blocks() == blocks
    assert open_fds() == fds


def test_client_free_releases_everything(client):
    """The path remove_sensors ultimately takes."""
    client.malloc("lifecycle_warmup_sensor_data", CAM_SHAPE, np.uint8)
    client.free("lifecycle_warmup_sensor_data")

    blocks, fds = shm_blocks(), open_fds()
    for i in range(CYCLES):
        key = "lifecycle_B_{}_sensor_data".format(i)
        client.malloc(key, CAM_SHAPE, np.uint8)
        client.free(key)

    assert shm_blocks() == blocks
    assert open_fds() == fds
    assert client._memory == {}


def test_realloc_at_new_shape_keeps_inode(client):
    """Reallocation must drop the old mapping without disturbing the file.

    close() rather than unlink(), so the inode the engine mapped survives.
    """
    key = "lifecycle_C_sensor_data"
    client.malloc(key, [64, 64, 4], np.uint8)
    inode = os.stat(shm_path(key)).st_ino
    blocks, fds = shm_blocks(), open_fds()

    client.malloc(key, [128, 128, 4], np.uint8)

    assert shm_blocks() == blocks
    assert open_fds() == fds
    assert os.stat(shm_path(key)).st_ino == inode


def test_clear_zeroes_buffer_but_keeps_mapping(client):
    """The env.reset() path.

    This is the regression guard. If clear() ever unlinks, every sensor in the
    simulator silently reads zeros after the first reset.
    """
    key = "lifecycle_D_sensor_data"
    array = client.malloc(key, [16, 16, 4], np.uint8)
    array[:] = 7
    inode = os.stat(shm_path(key)).st_ino

    client.clear(key)

    assert client._memory[key].np_array.sum() == 0
    assert os.stat(shm_path(key)).st_ino == inode
    assert key in client._memory


def test_free_can_be_deferred_to_end_of_tick(client):
    """The engine only drops its mapping when it handles RemoveSensor, so the
    release waits for the tick carrying that command to complete."""
    key = "lifecycle_E_sensor_data"
    client.malloc(key, CAM_SHAPE, np.uint8)

    client.defer_free(key)
    client.defer_free(key)
    assert key in client._memory, "defer_free must not release immediately"

    assert client.drain_pending_frees() == 1
    assert client._memory == {}
    assert not os.path.exists(shm_path(key))
