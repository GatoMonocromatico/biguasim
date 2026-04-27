============================
Semantic Secmentation Camera
============================

See :ref:visualizing_annotation_output for an example of how to map these colors back to object labels in Python.

See :py:class:~biguasim.sensors.AnnotationComponent for the Python API.

Example sensor definition (from a BlueROV2)::

    {
        "sensor_type": "AnnotationComponent",
        "sensor_name": "semantic_seg",
        "socket": "CameraSocket",
        "Hz": 30,
        "configuration": {
            "Width": 640,
            "Height": 480,
            "FOV": 90.0,
            "TicksPerCapture": 1
        }
    }
