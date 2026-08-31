"""Starting ArduPilot SITL beside the world, on request.

A client that wants a vehicle flown by a real flight controller should not have
to log into the server, work out which ports are free, remember that the home
location has to match the bridge's GPS origin, and start two processes in the
right order. It already told the world everything that matters -- which world,
which vehicle, where -- so the world can do the rest.

What the world composes, and what it does not
=============================================

Everything the world already knows is filled in and cannot be overridden:

``-v``
    Which ArduPilot the vehicle needs, taken from its own registry entry --
    a BlueROV2 is ArduSub, a HolybroX500 is ArduCopter, and the client already
    said which vehicle it wanted when it asked for the agent.
``-f JSON:<address>``
    The FDM backend, pointed at the bridge the world is about to start.
``-L`` / ``--custom-location``
    Home. This is not a convenience: ArduPilot's home and the bridge's GPS
    origin have to be the same point, or the EKF places the vehicle thousands
    of kilometres from where the world says it is.
``-I``
    The SITL instance, which shifts every port by ten and is how a second
    vehicle avoids the first.
``--out``
    A listening MAVLink endpoint, so a GCS on another machine can connect
    without the server needing to know where it is.

Everything else is the caller's. Named keys are handled explicitly; anything
else becomes a flag:

.. code-block:: python

    {"console": True, "map": True, "speedup": 2, "wipe-eeprom": True}
    # -> --console --map --speedup 2 --wipe-eeprom

so a flag this module has never heard of still works, without anyone sending a
command line to be executed.

On execution
============

The command is built as an argument list and run without a shell, so a value
containing shell metacharacters is an argument and nothing more. That is the
whole of the isolation: a caller who can reach the world can still ask
``sim_vehicle.py`` to do anything ``sim_vehicle.py`` can do. The feature is off
unless the world was started with it on, and parameter files resolve inside one
configured directory rather than anywhere on disk.
"""
import os
import shlex
import signal
import subprocess
import sys
import time

#: Flags the world owns. Passing one of these is refused rather than merged,
#: because the failure of a silently-ignored home location is a vehicle that
#: looks broken in ways that have nothing to do with the flag.
RESERVED = {
    "f", "frame", "I", "instance", "L", "location", "custom-location",
    "custom_location", "v", "vehicle", "sim-address", "sim_address",
}

#: Where sim_vehicle.py sends servo output and expects state back, before the
#: instance offset. Mirrors biguasim.ardubridge.remote_runner.
FDM_BASE_PORT = 9002

#: ArduPilot's own MAVLink TCP port, before the instance offset.
MAVLINK_BASE_PORT = 5760

#: Where the world asks MAVProxy to listen for a GCS, before the offset.
GCS_BASE_PORT = 14551

#: Ports move by this much per SITL instance (``SITL_cmdline.cpp:426``).
PORTS_PER_INSTANCE = 10

#: ArduPilot ships this location as exactly the latitude and longitude the
#: bridge uses for the world origin, so the two agree by name.
RATBEACH = (33.810313, -118.393867)


class SitlError(Exception):
    """A SITL request the world will not carry out."""


def _flag(key):
    """Turn a config key into a command-line flag.

    Underscores become dashes so a caller can write either, and a single
    character takes one dash, since that is what ArduPilot's short options use.
    """
    key = str(key).replace("_", "-")
    return ("-" + key) if len(key) == 1 else ("--" + key)


def resolve_params(name, params_dir):
    """Find a parameter file by name, inside the directory the world allows.

    Args:
        name (:obj:`str`): File name, as the client asked for it.
        params_dir (:obj:`str`): The only directory parameter files may come
            from.

    Returns:
        :obj:`str`: An absolute path.

    Raises:
        SitlError: If no directory is configured, the name escapes it, or the
            file is not there.
    """
    if not params_dir:
        raise SitlError(
            "this world accepts no parameter files: it was started without "
            "--sitl-params-dir")
    root = os.path.realpath(os.path.expanduser(params_dir))
    # realpath first, then containment: a name like ../../etc/passwd, or a
    # symlink pointing out of the directory, has to fail the same way.
    path = os.path.realpath(os.path.join(root, name))
    if path != root and not path.startswith(root + os.sep):
        raise SitlError(
            "parameter file {!r} is outside {}".format(name, root))
    if not os.path.isfile(path):
        raise SitlError("no parameter file {!r} in {}".format(name, root))
    return path


def home_argument(gps_origin):
    """The sim_vehicle argument putting SITL's home at the world origin.

    A named location reads better and is what a person would type, so one is
    used when it fits; anything else falls back to the explicit form rather
    than being quietly wrong.
    """
    lat, lon = (float(v) for v in gps_origin[:2])
    if (round(lat, 6), round(lon, 6)) == RATBEACH:
        return ["-L", "RATBeach"]
    return ["--custom-location={},{},0,270".format(lat, lon)]


def ports_for(instance):
    """Every port one SITL instance occupies.

    Returns:
        :obj:`dict`: ``fdm``, ``mavlink`` and ``gcs`` port numbers.
    """
    shift = PORTS_PER_INSTANCE * int(instance)
    return {"fdm": FDM_BASE_PORT + shift,
            "mavlink": MAVLINK_BASE_PORT + shift,
            "gcs": GCS_BASE_PORT + shift}


def build_command(config, *, instance, gps_origin, ardupilot_vehicle,
                  params_dir=None, sim_vehicle="sim_vehicle.py",
                  address="127.0.0.1", gcs_bind="0.0.0.0"):
    """Compose the sim_vehicle.py command for one agent.

    Args:
        config (:obj:`dict`): What the client asked for. Recognised keys are
            ``params`` (a file name inside ``params_dir``) and ``gcs_port``.
            Every other key becomes a flag: ``True`` a bare one, a list a
            repeated one, anything else a flag and a value.
        instance (:obj:`int`): SITL instance number.
        gps_origin (sequence of :obj:`float`): Latitude and longitude the world
            origin maps to. SITL's home is set to match.
        ardupilot_vehicle (:obj:`str`): Which ArduPilot to run, from the
            vehicle's registry entry.
        params_dir (:obj:`str`, optional): The only directory ``params`` may
            name a file in.
        sim_vehicle (:obj:`str`, optional): Path to sim_vehicle.py.
        address (:obj:`str`, optional): Where the bridge is listening.
        gcs_bind (:obj:`str`, optional): Interface for the listening MAVLink
            endpoint.

    Returns:
        :obj:`tuple`: ``(argv, ports)`` -- the command as a list, never a
        string, and the ports it will occupy.

    Raises:
        SitlError: If the config names a flag the world owns, or a parameter
            file it will not read.
    """
    config = dict(config or {})
    ports = ports_for(instance)

    for key in list(config):
        if str(key).lstrip("-") in RESERVED:
            raise SitlError(
                "{!r} is set by the world and cannot be given here -- it is "
                "derived from the world's own origin, ports and vehicle "
                "registry".format(key))

    params = config.pop("params", None)
    gcs_port = int(config.pop("gcs_port", ports["gcs"]))
    ports["gcs"] = gcs_port

    argv = [sim_vehicle, "-v", str(ardupilot_vehicle),
            "-f", "JSON:{}".format(address)]
    argv += home_argument(gps_origin)
    argv += ["-I", str(int(instance))]

    if params is not None:
        argv.append("--add-param-file={}".format(
            resolve_params(params, params_dir)))

    # Listening rather than dialling, so the world never needs to know where a
    # GCS is and the link survives that address changing.
    argv.append("--out=tcpin:{}:{}".format(gcs_bind, gcs_port))

    for key, value in config.items():
        flag = _flag(key)
        if value is True:
            argv.append(flag)
        elif value is False or value is None:
            continue
        elif isinstance(value, (list, tuple)):
            for item in value:
                argv += [flag, str(item)]
        else:
            argv += [flag, str(value)]

    return argv, ports


def describe(argv):
    """The command as a person would type it, for logs and error messages."""
    return " ".join(shlex.quote(part) for part in argv)


class SitlSupervisor:
    """Starts and reaps the SITL and pilot a provisioned vehicle needs.

    Two child processes per vehicle, both on this machine:

    * **SITL** -- the flight controller, talking to the bridge over loopback.
      Nothing else would do: its FDM protocol is a blocking lockstep handshake,
      so a network in the middle would cap the whole flight loop at ``1/RTT``.
    * **the pilot** -- ``biguasim.ardubridge.pilot_cli``, which connects back to
      this world as an ordinary client, binds the bridge, and spawns the agent
      once SITL says hello.

    The pilot is a separate process rather than a thread on purpose. It blocks
    on a UDP socket for up to ten milliseconds per tick, which inside the tick
    loop would stall the world for every other client; and a flight controller
    that wedges should not be able to take the simulation down with it.

    It is also why the world does not create the agent itself when a spawn asks
    for ArduPilot. The pilot does, when its SITL is up -- an agent that exists
    before something is stabilising it does not wait politely, it falls.

    Args:
        package (:obj:`str`), world (:obj:`str`): Which world the pilot should
            connect to. Must match this world's own, since the pilot is checked
            against it like any other client.
        port (:obj:`int`): This world's request port.
        gps_origin (sequence of :obj:`float`): The bridge's origin, which SITL's
            home is set to match.
        address (:obj:`str`, optional): How the pilot should reach this world.
        params_dir (:obj:`str`, optional): The only directory a requested
            parameter file may come from.
        sim_vehicle (:obj:`str`, optional): Path to ``sim_vehicle.py``.
        log_dir (:obj:`str`, optional): Where to write each child's output.
            Without it they inherit the world's own streams, which interleaves
            several flight controllers into one terminal.
        enabled (:obj:`bool`, optional): Whether to do any of this at all.
            Off unless the world was started with it on, because it runs
            processes on behalf of whoever can reach the port.
    """

    def __init__(self, *, package, world, port, gps_origin,
                 address="127.0.0.1", params_dir=None,
                 sim_vehicle="sim_vehicle.py", log_dir=None, enabled=False):
        self.enabled = bool(enabled)
        self._package = package
        self._world = world
        self._port = int(port)
        self._address = address
        self._gps_origin = tuple(gps_origin)
        self._params_dir = params_dir
        self._sim_vehicle = sim_vehicle
        self._log_dir = log_dir
        self._vehicles = {}         # agent -> record of what was started

    # ------------------------------------------------------------ accounting

    @property
    def vehicles(self):
        """:obj:`dict`: What is provisioned, by agent name."""
        return {name: dict(entry) for name, entry in self._vehicles.items()}

    def next_instance(self):
        """The lowest instance number nothing is using.

        Numbered rather than arbitrary because every ArduPilot port derives
        from it, so a gap left by a departed vehicle is worth reusing.
        """
        taken = {entry["instance"] for entry in self._vehicles.values()}
        instance = 0
        while instance in taken:
            instance += 1
        return instance

    # ------------------------------------------------------------- lifecycle

    def provision(self, agent, agent_type, config):
        """Start a flight controller and a pilot for one agent.

        Args:
            agent (:obj:`str`): Agent name, as the world will know it.
            agent_type (:obj:`str`): Vehicle type, used for its registry entry.
            config (:obj:`dict`): The client's ``ardupilot`` block.

        Returns:
            :obj:`dict`: What was started -- instance, ports, and the command.

        Raises:
            SitlError: If the feature is off, the vehicle is unknown, the agent
                already has one, or the command cannot be composed.
        """
        from biguasim.ardubridge import VEHICLE_REGISTRY

        if not self.enabled:
            raise SitlError(
                "this world does not start flight controllers: it was started "
                "without --allow-sitl")
        if agent in self._vehicles:
            raise SitlError("{!r} already has a flight controller".format(agent))

        match = next((k for k in VEHICLE_REGISTRY
                      if k.upper() == str(agent_type).upper()), None)
        if match is None:
            raise SitlError(
                "no ArduPilot profile for {!r}; known vehicles are {}".format(
                    agent_type, ", ".join(sorted(VEHICLE_REGISTRY))))
        profile = VEHICLE_REGISTRY[match]

        config = dict(config or {})
        instance = config.pop("instance", None)
        instance = self.next_instance() if instance is None else int(instance)

        argv, ports = build_command(
            config, instance=instance, gps_origin=self._gps_origin,
            ardupilot_vehicle=profile.ardupilot_vehicle,
            params_dir=self._params_dir, sim_vehicle=self._sim_vehicle,
            address=self._address)

        pilot_argv = [
            sys.executable, "-m", "biguasim.ardubridge.pilot_cli",
            "--package", self._package, "--world", self._world,
            "--address", self._address, "--port", str(self._port),
            "--vehicle", match, "--agent", agent,
            "--instance", str(instance),
        ]

        entry = {"instance": instance, "ports": ports,
                 "command": describe(argv), "agent_type": match,
                 "sitl": None, "pilot": None}
        try:
            # The pilot goes up first so the bridge is listening before SITL
            # starts talking to it. SITL resends servos until answered, so the
            # other order also works -- this one just wastes less time.
            entry["pilot"] = self._launch(pilot_argv, agent, "pilot")
            entry["sitl"] = self._launch(argv, agent, "sitl")
        except Exception as exc:                                  # noqa: BLE001
            self._stop_entry(entry)
            raise SitlError("could not start {!r}: {}".format(agent, exc))

        self._vehicles[agent] = entry
        return entry

    def _launch(self, argv, agent, role):
        """Start one child, in its own process group.

        The group matters for SITL: ``sim_vehicle.py`` is a launcher, and
        killing it alone leaves the ArduPilot binary and MAVProxy behind,
        holding the ports the next vehicle wants.
        """
        stream = None
        if self._log_dir:
            os.makedirs(self._log_dir, exist_ok=True)
            stream = open(os.path.join(
                self._log_dir, "{}-{}.log".format(agent, role)), "wb")
        return {
            "process": subprocess.Popen(
                argv, stdout=stream, stderr=subprocess.STDOUT if stream else None,
                start_new_session=True),
            "log": stream,
        }

    def poll(self):
        """Report any child that has exited on its own.

        Returns:
            :obj:`list`: ``(agent, role, returncode)`` for each one, so the
            world can tell the owner rather than leaving a vehicle that quietly
            stopped being flown.
        """
        gone = []
        for agent, entry in self._vehicles.items():
            for role in ("sitl", "pilot"):
                child = entry.get(role)
                if child is None or child.get("reported"):
                    continue
                code = child["process"].poll()
                if code is not None:
                    child["reported"] = True
                    gone.append((agent, role, code))
        return gone

    def release(self, agent):
        """Stop whatever was started for an agent, if anything was."""
        entry = self._vehicles.pop(agent, None)
        if entry is not None:
            self._stop_entry(entry)

    @staticmethod
    def _stop_entry(entry, grace=5.0):
        """End both children, politely and then not.

        The whole group is signalled, since what was started is a launcher and
        what needs to stop is everything it started.
        """
        for role in ("sitl", "pilot"):
            child = entry.get(role)
            if child is None:
                continue
            process = child["process"]
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    pass
            deadline = time.time() + grace
            while process.poll() is None and time.time() < deadline:
                time.sleep(0.05)
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
                process.wait(timeout=2)
            if child.get("log"):
                child["log"].close()

    def close(self):
        """Stop everything."""
        for agent in list(self._vehicles):
            self.release(agent)
