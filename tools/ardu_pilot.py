"""Fly one ArduPilot SITL vehicle in a world running as a service.

    python tools/ardu_pilot.py --package Competition --world CompetionMap \
        --vehicle HolybroX500 --agent uav0 --location 0,0,1

The implementation lives in :mod:`biguasim.ardubridge.pilot_cli` so that a
world asked to provision a vehicle can start exactly the same pilot, as
``python -m biguasim.ardubridge.pilot_cli``, rather than reaching for a script
path that is not installed with the package.
"""
from biguasim.ardubridge.pilot_cli import main

if __name__ == "__main__":
    main()
