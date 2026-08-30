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


@pytest.mark.parametrize("address, expected", [
    ("127.0.0.1", "tcp://127.0.0.1:8770"),
    ("100.101.40.22", "tcp://100.101.40.22:8770"),
    ("example.com", "tcp://example.com:8770"),
    ("*", "tcp://*:8770"),
])
def test_endpoints_without_colons_are_left_alone(address, expected):
    assert proto.endpoint(address, 8770) == expected


@pytest.mark.parametrize("address, expected", [
    ("::", "tcp://[::]:8770"),
    ("::1", "tcp://[::1]:8770"),
    ("2804:60:114:8b00::1", "tcp://[2804:60:114:8b00::1]:8770"),
    ("2804:60:114:8b00:48e5:b380:dbff:bd2f",
     "tcp://[2804:60:114:8b00:48e5:b380:dbff:bd2f]:8770"),
])
def test_ipv6_literals_are_bracketed(address, expected):
    """An IPv6 address is all colons, and so is the host:port separator.

    Without brackets ``tcp://2804:60:114::1:8770`` cannot be parsed at all.
    """
    assert proto.endpoint(address, 8770) == expected


def test_already_bracketed_addresses_are_not_bracketed_twice():
    assert proto.endpoint("[2804::1]", 8770) == "tcp://[2804::1]:8770"


def test_the_state_port_is_one_above_the_request_port():
    assert proto.endpoint("::1", 8771) == "tcp://[::1]:8771"


def _install(tmp_path, monkeypatch, worlds, version="1.0.0", indent=None):
    """Write a fake installed package config and point build_id at it."""
    import json
    root = tmp_path / ".local" / "share" / "biguasim" / "1.0.0" / "worlds" / "SkyDive"
    root.mkdir(parents=True, exist_ok=True)
    (root / "config.json").write_text(json.dumps(
        {"name": "SkyDive", "platform": "Linux", "version": version,
         "path": "Linux/Biguasim/Binaries/Linux/Holodeck", "worlds": worlds},
        indent=indent))
    monkeypatch.setattr(proto.os.path, "expanduser",
                        lambda p: str(tmp_path / ".local" / "share" / "biguasim")
                        if "biguasim" in p else p)
    return {"package_name": "SkyDive", "world": "Pier-Harbor"}


PIER = {"name": "Pier-Harbor", "pre_start_steps": 20,
        "env_min": [-500.0, -500.0, -100.0], "env_max": [500.0, 500.0, 100.0]}
BRIDGE = {"name": "Bridge", "pre_start_steps": 20,
          "env_min": [-200, -200, -50.0], "env_max": [200.0, 200.0, 50.0]}
EXTRA = {"name": "Aquatec_full", "pre_start_steps": 20,
         "env_min": [-500.0, -500.0, -100.0], "env_max": [500.0, 500.0, 100.0]}


def test_an_unrelated_extra_world_does_not_change_the_build_id(tmp_path, monkeypatch):
    """Having another world installed says nothing about this one's geometry.

    Hashing the whole config file made this a false alarm, which refused
    connections between machines whose Pier-Harbor was byte-identical.
    """
    cfg = _install(tmp_path, monkeypatch, [BRIDGE, PIER])
    without = proto.build_id(cfg)
    cfg = _install(tmp_path, monkeypatch, [BRIDGE, PIER, EXTRA])
    assert proto.build_id(cfg) == without


def test_formatting_does_not_change_the_build_id(tmp_path, monkeypatch):
    cfg = _install(tmp_path, monkeypatch, [PIER], indent=None)
    compact = proto.build_id(cfg)
    cfg = _install(tmp_path, monkeypatch, [PIER], indent=4)
    assert proto.build_id(cfg) == compact


def test_a_changed_world_definition_does_change_the_build_id(tmp_path, monkeypatch):
    """The case the check exists for: different geometry for the same name."""
    cfg = _install(tmp_path, monkeypatch, [PIER])
    original = proto.build_id(cfg)
    moved = dict(PIER, env_max=[9999.0, 9999.0, 100.0])
    cfg = _install(tmp_path, monkeypatch, [moved])
    assert proto.build_id(cfg) != original


def test_a_different_package_version_does_change_the_build_id(tmp_path, monkeypatch):
    cfg = _install(tmp_path, monkeypatch, [PIER], version="1.0.0")
    original = proto.build_id(cfg)
    cfg = _install(tmp_path, monkeypatch, [PIER], version="1.1.0")
    assert proto.build_id(cfg) != original


def test_a_missing_world_does_change_the_build_id(tmp_path, monkeypatch):
    cfg = _install(tmp_path, monkeypatch, [PIER])
    present = proto.build_id(cfg)
    cfg = _install(tmp_path, monkeypatch, [BRIDGE])
    assert proto.build_id(cfg) != present
