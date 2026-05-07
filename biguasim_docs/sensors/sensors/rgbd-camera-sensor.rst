====================
RGBD (Depth) Camera
====================

Capture a high-fidelity Z-buffer depth image from the camera. Each pixel represents the precise linear distance (in meters) from the camera plane to the nearest geometry.

See :ref:visualizing_depthcamera_output for an example of how to use this sensor for point cloud generation.

See :py:class:~biguasim.sensors.DepthCamera for the Python API.

Example sensor definition (from a DjiMatrice)::

    {
        "sensor_type": "DepthCamera",
        "sensor_name": "depth",
        "socket": "CameraSocket",
        "Hz": 30,
        "configuration": {
            "Width": 512,
            "Height": 512,
            "FOV": 90.0,
            "TicksPerCapture": 1,
            "MaxDistance": 10.0,
            "MinDistance": 0.1,
            "ShowDisplay": false
        }
    }