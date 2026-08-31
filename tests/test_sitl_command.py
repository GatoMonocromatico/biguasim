"""Composing the SITL command. No engine, no subprocesses.

Everything here is about the seam between what the world knows and what the
client asks for. The world's half is not negotiable -- home has to match the
bridge's GPS origin, ports have to match the instance -- and the client's half
has to stay open enough that a flag nobody anticipated still works.
"""
import os

import pytest

from biguasim.server import sitl


def argv_of(config=None, **kwargs):
    kwargs.setdefault("instance", 0)
    kwargs.setdefault("gps_origin", sitl.RATBEACH)
    kwargs.setdefault("ardupilot_vehicle", "ArduCopter")
    return sitl.build_command(config or {}, **kwargs)[0]


# ------------------------------------------------------------ what the world owns

def test_the_backend_and_home_are_always_set():
    argv = argv_of()
    assert "-f" in argv and "JSON:127.0.0.1" in argv
    # RATBeach is exactly the bridge's GPS origin, which is the entire point.
    assert argv[argv.index("-L") + 1] == "RATBeach"


def test_an_unusual_origin_falls_back_to_an_explicit_home():
    """Rather than naming a location that means somewhere else."""
    argv = argv_of(gps_origin=(-32.07, -52.17))
    assert not any(part == "-L" for part in argv)
    assert "--custom-location=-32.07,-52.17,0,270" in argv


def test_ports_follow_the_instance():
    for instance, fdm, mavlink in [(0, 9002, 5760), (1, 9012, 5770), (4, 9042, 5800)]:
        ports = sitl.ports_for(instance)
        assert (ports["fdm"], ports["mavlink"]) == (fdm, mavlink)


def test_the_gcs_endpoint_listens_rather_than_dials():
    """So the world never needs to know a client's address."""
    argv = argv_of(instance=2)
    assert "--out=tcpin:0.0.0.0:14571" in argv


@pytest.mark.parametrize("key", ["f", "-f", "I", "instance", "L",
                                 "custom-location", "vehicle", "sim_address"])
def test_flags_the_world_owns_are_refused(key):
    """Merging them would silently produce two homes, or two instances."""
    with pytest.raises(sitl.SitlError, match="set by the world"):
        argv_of({key: "anything"})


# ---------------------------------------------------------- what the client owns

def test_an_unrecognised_key_becomes_a_flag():
    argv = argv_of({"speedup": 2})
    assert argv[argv.index("--speedup") + 1] == "2"


def test_true_is_a_bare_flag_and_false_is_nothing():
    argv = argv_of({"console": True, "map": True, "no-rebuild": False})
    assert "--console" in argv and "--map" in argv
    assert "--no-rebuild" not in argv


def test_underscores_are_accepted_for_dashes():
    """A JSON config is more likely to be written with underscores."""
    assert "--no-mavproxy" in argv_of({"no_mavproxy": True})


def test_a_single_character_key_takes_one_dash():
    assert "-N" in argv_of({"N": True})


def test_a_list_repeats_the_flag():
    argv = argv_of({"out": ["udp:10.0.0.2:14550", "udp:10.0.0.3:14550"]})
    assert argv.count("--out") == 2
    assert "udp:10.0.0.3:14550" in argv


def test_the_ardupilot_comes_from_the_vehicle_registry():
    """A BlueROV2 needs ArduSub, and the client already said BlueROV2.

    Asking for it twice is a chance to disagree with the vehicle actually
    being spawned, which nothing downstream would notice.
    """
    assert argv_of(ardupilot_vehicle="ArduSub")[2] == "ArduSub"
    assert argv_of(ardupilot_vehicle="ArduCopter")[2] == "ArduCopter"


# -------------------------------------------------------------- parameter files

def test_a_parameter_file_is_resolved_inside_the_allowed_directory(tmp_path):
    (tmp_path / "holybro.parm").write_text("SCHED_LOOP_RATE 100\n")
    argv = argv_of({"params": "holybro.parm"}, params_dir=str(tmp_path))
    assert "--add-param-file={}".format(tmp_path / "holybro.parm") in argv


def test_a_parameter_file_cannot_escape_that_directory(tmp_path):
    outside = tmp_path / "secret.parm"
    outside.write_text("x\n")
    allowed = tmp_path / "params"
    allowed.mkdir()
    with pytest.raises(sitl.SitlError, match="outside"):
        argv_of({"params": "../secret.parm"}, params_dir=str(allowed))


def test_a_symlink_out_of_the_directory_is_refused_too(tmp_path):
    """realpath before the containment check, or a link walks straight out."""
    outside = tmp_path / "secret.parm"
    outside.write_text("x\n")
    allowed = tmp_path / "params"
    allowed.mkdir()
    os.symlink(outside, allowed / "innocent.parm")
    with pytest.raises(sitl.SitlError, match="outside"):
        argv_of({"params": "innocent.parm"}, params_dir=str(allowed))


def test_a_missing_parameter_file_says_where_it_looked(tmp_path):
    with pytest.raises(sitl.SitlError, match="no parameter file"):
        argv_of({"params": "absent.parm"}, params_dir=str(tmp_path))


def test_parameter_files_are_refused_when_none_are_configured():
    with pytest.raises(sitl.SitlError, match="--sitl-params-dir"):
        argv_of({"params": "anything.parm"})


# --------------------------------------------------------------------- shape

def test_the_command_is_a_list_never_a_string():
    """It is executed without a shell, so a value with metacharacters is
    an argument and cannot become anything else."""
    argv = argv_of({"speedup": "2; rm -rf /"})
    assert isinstance(argv, list)
    assert "2; rm -rf /" in argv
    assert not any(";" in part for part in argv if part != "2; rm -rf /")


def test_describe_is_readable_and_quoted():
    argv = argv_of({"speedup": "a b"})
    assert "'a b'" in sitl.describe(argv)
