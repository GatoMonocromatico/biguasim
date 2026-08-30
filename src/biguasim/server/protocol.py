"""What clients and the world say to each other.

Deliberately small. Two sockets do all the work:

* a request socket, where a client submits actions and gets told which tick
  each one landed on, and
* a publish socket, carrying state every tick and sensor data on demand.

Sensor streams are separate topics rather than part of the state message
because they differ in cost by three orders of magnitude. A viewer that only
wants to watch subscribes to state and costs the world a few kilobytes a tick;
one that wants an imaging sonar subscribes to that topic alone and pays for it.

Every message is msgpack. Field names are spelled out rather than packed into
positions: this is not the bandwidth that matters, and a readable log is worth
more than the bytes.
"""
import hashlib
import os

import msgpack

#: Bumped when the shape of these messages changes incompatibly.
PROTOCOL_VERSION = 1

#: Published every tick. Small enough that viewers are effectively free.
TOPIC_STATE = b"state"
#: Prefix for per-sensor streams: ``sensor/<agent>/<sensor>``.
TOPIC_SENSOR = "sensor"

# Client -> world
OP_HELLO = "hello"
OP_SUBMIT = "submit"
OP_BYE = "bye"
OP_PING = "ping"


def endpoint(address, port):
    """Build a ZeroMQ ``tcp://`` endpoint, bracketing IPv6 literals.

    An IPv6 address is full of colons, and so is the ``host:port`` separator.
    ``tcp://2804:60:114::1:8770`` is unparseable for that reason, so the address
    part has to be wrapped in brackets. Hostnames and IPv4 addresses are left
    alone, and an address that is already bracketed is not bracketed twice.

    Args:
        address (:obj:`str`): Host, IPv4 literal, IPv6 literal, or ``*``.
        port (:obj:`int`): Port number.

    Returns:
        :obj:`str`: The endpoint.
    """
    address = str(address)
    if ":" in address and not address.startswith("["):
        address = "[{}]".format(address)
    return "tcp://{}:{}".format(address, port)


def sensor_topic(agent, sensor):
    """The topic a given sensor publishes on.

    Args:
        agent (:obj:`str`): Base agent name.
        sensor (:obj:`str`): Sensor name.

    Returns:
        :obj:`bytes`: The topic to subscribe to.
    """
    return "{}/{}/{}".format(TOPIC_SENSOR, agent, sensor).encode()


def pack(obj):
    """Serialize a message."""
    return msgpack.packb(obj, use_bin_type=True)


def unpack(raw):
    """Deserialize a message."""
    return msgpack.unpackb(raw, raw=False, strict_map_key=False)


def build_id(scenario_cfg):
    """Identify the world build a process is running.

    A client renders with its own copy of the world, so if its package differs
    from the world's, its collision geometry disagrees with the authoritative
    one -- and nothing about that failure looks like a version problem. Cheaper
    to refuse the connection.

    Args:
        scenario_cfg (:obj:`dict`): The scenario the world was built from.

    Returns:
        :obj:`str`: A digest of the package, world and installed world config.
    """
    package = scenario_cfg.get("package_name", "")
    world = scenario_cfg.get("world", "")
    digest = hashlib.sha256()
    digest.update(package.encode())
    digest.update(world.encode())

    config = os.path.join(
        os.path.expanduser("~/.local/share/biguasim"), "1.0.0", "worlds",
        package, "config.json")
    try:
        with open(config, "rb") as handle:
            digest.update(handle.read())
    except OSError:
        # Not installed here, or installed somewhere else. The package and
        # world names still have to match, which is the common failure.
        digest.update(b"<no local config>")

    return "{}:{}:{}".format(package, world, digest.hexdigest()[:12])
