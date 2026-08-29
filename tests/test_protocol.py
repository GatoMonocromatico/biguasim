"""The wire format. No engine, no sockets."""
import numpy as np
import pytest

from biguasim.server import protocol as proto


def test_messages_survive_a_round_trip():
    message = {"tick": 7, "agents": {"uav0": {"position": [1.0, 2.0, 3.0]}}}
    assert proto.unpack(proto.pack(message)) == message


def test_binary_payloads_survive_intact():
    """Sensor data goes over as raw bytes plus a dtype and shape."""
    array = np.arange(12, dtype=np.float32).reshape(3, 4)
    packed = proto.pack({"dtype": array.dtype.str, "shape": list(array.shape),
                         "data": array.tobytes()})
    got = proto.unpack(packed)
    restored = np.frombuffer(got["data"], dtype=np.dtype(got["dtype"])).reshape(got["shape"])
    assert np.array_equal(restored, array)


def test_sensor_topics_are_namespaced_per_agent():
    assert proto.sensor_topic("uav0", "cam") == b"sensor/uav0/cam"
    assert proto.sensor_topic("uav1", "cam") != proto.sensor_topic("uav0", "cam")


@pytest.mark.parametrize("changed", [
    {"package_name": "Other", "world": "Pier-Harbor"},
    {"package_name": "SkyDive", "world": "Bridge"},
])
def test_build_id_distinguishes_package_and_world(changed):
    """A client rendering a different world must not be allowed to connect."""
    base = proto.build_id({"package_name": "SkyDive", "world": "Pier-Harbor"})
    assert proto.build_id(changed) != base


def test_build_id_is_stable():
    cfg = {"package_name": "SkyDive", "world": "Pier-Harbor"}
    assert proto.build_id(cfg) == proto.build_id(dict(cfg))
