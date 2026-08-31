"""Ask the world to fly a HolybroX500 with a real ArduPilot.

The world starts SITL and the bridge itself, so this only says which vehicle,
where, and the few sim_vehicle options it cannot work out on its own.

Start the world with the feature on, and with the directory holding the
parameter file:

    python tools/serve_world.py --package Competition --world CompetionMap \
        --port 8770 --rate 200 --allow-sitl \
        --sitl-params-dir /home/nautec/Documents/joao_pessoa_2026/src/hydrone_bringup/config/params \
        --sitl-log-dir ~/sitl-logs

--rate 200 matters: the tick rate is the rate ArduPilot's attitude loop runs
at, and arming needs it at 1.8x SCHED_LOOP_RATE. At serve_world's default of 20
the vehicle will not arm.

The command this produces on the server is:

    sim_vehicle.py -v ArduCopter -f JSON:127.0.0.1 -L RATBeach -I 0 \
        --add-param-file=<params-dir>/holybro_sitl_gps.parm \
        --out=tcpin:0.0.0.0:14551 --console --map

Only --console, --map and the parameter file are named below. Everything else
comes from the world: -v from the HolybroX500 registry entry, -f from the
bridge it is about to start, -L from that bridge's GPS origin, -I from the
lowest free instance, --out from the same.
"""
import time

from biguasim.client import RemoteError, RemoteWorld

AGENT = "uav3"
scenario = {"package_name": "Competition", "world": "CompetionMap"}

with RemoteWorld(address="100.119.211.18", port=8770, client_id="tester",
                 scenario_cfg=scenario) as world:
    world.watch_state()

    try:
        info = world.spawn_ardupilot_agent(
            AGENT, "HolybroX500",
            location=(18.0, -92.0, 6.0),
            ardupilot={
                "params": "holybro_sitl_gps.parm",
                "console": True,
                "map": True,
            })
    except RemoteError as exc:
        raise SystemExit("the world would not do it: {}".format(exc))

    print("SITL instance {}".format(info["instance"]))
    print("  ran: {}".format(info["command"]))
    print("  MAVLink on the server : tcp/{}".format(info["ports"]["mavlink"]))
    print("  QGroundControl here   : TCP to 100.119.211.18:{}".format(
        info["ports"]["gcs"]))
    print()

    # The request started a flight controller; it did not create the agent.
    # The pilot does that once SITL is up, so that the vehicle never exists
    # without something stabilising it -- which for a quadrotor means the few
    # seconds SITL takes to boot are seconds it would otherwise spend falling.
    print("waiting for {} to appear (SITL is booting)...".format(AGENT))
    deadline = time.time() + 120
    for state in world.states(timeout=5.0):
        if AGENT in state.get("agents", {}):
            print("{} is in the world at tick {}\n".format(AGENT, state["tick"]))
            break
        if time.time() > deadline:
            raise SystemExit(
                "{} never appeared. Check the pilot and SITL logs in "
                "--sitl-log-dir on the server.".format(AGENT))

    for state in world.states(timeout=10.0):
        agent = state.get("agents", {}).get(AGENT)
        if agent is None:
            print("{} left the world".format(AGENT))
            break
        if state["tick"] % 100 == 0:
            print("tick {:6d}  {}".format(
                state["tick"], [round(v, 2) for v in agent["position"]]))
