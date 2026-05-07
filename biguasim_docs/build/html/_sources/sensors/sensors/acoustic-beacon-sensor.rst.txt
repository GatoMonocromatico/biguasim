===============
Acoustic Beacon
===============

An accoustic modem for communications between underwater agents.

The command :meth:`biguasim.environments.BiguaSimEnvironment.send_acoustic_message` is used to send 
messages between acoustic beacons. There's a number of message types that can be sent, all with 
varying functionality. See :class:`biguasim.sensors.AcousticBeaconSensor` for details.

Further, a few helper functions exist if needed:

- :py:attr:`biguasim.environments.BiguaSimEnvironment.beacons` returns all beacons.
- :py:attr:`biguasim.environments.BiguaSimEnvironment.beacons_id` returns all beacons' ids.
- :py:attr:`biguasim.environments.BiguaSimEnvironment.beacons_status` returns all beacons' status 
  (whether it's transmitting or not).

See :ref:`multi-agent-comms_example` for an example of how to use this sensor.

See :py:class:`~biguasim.sensors.AcousticBeaconSensor` for the Python API.

.. TO DO: Add a bunch of detail on message types, etc. 

Example sensor definition::

    {
        "sensor_type": "AcousticBeaconSensor",
        "location": [0,0,0],
        "configuration": {
            "id": 1
        }
    }