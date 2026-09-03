"""Abre o simulador com o KopisX8 e faz ele subir. So isso."""
import uuid

import numpy as np
import biguasim

# a primeira tick compila shader e estoura o timeout de 60s
biguasim.environments.BiguaSimEnvironment._timeout = property(lambda self: None)

config = {
    "package_name": "Competition",
    "world": "CompetionMap",
    "main_agent": "uav0",
    "ticks_per_sec": 20,
    "frames_per_sec": False,
    "octree_min": 0.02,
    "octree_max": 5.0,
    "agents": [
        {
            "agent_name": "uav0",
            "agent_type": "KopisX8",
            "sensors": [
                {
                    "sensor_type": "DynamicsSensor",
                    "socket": "IMUSocket",
                    "configuration": {
                        "UseCOM": True,
                        "UseRPY": False,
                    },
                },
                {
                    "sensor_type": "RGBCamera",
                    "sensor_name": "MinhaCam",
                    "socket": "CameraSocket",
                    # location/rotation sao chaves DO SENSOR, nao vao dentro
                    # de configuration (e sao minusculas)
                    "location": [1, 0, 0],
                    "rotation": [0, 0, 0],
                    "configuration": {
                        "CaptureWidth": 256 * 3,
                        "CaptureHeight": 256 * 3,
                        "ExposureMethod": "AEM_Manual",
                        "ExposureCompensation": -5.0,
                    },
                },
            ],
            "dynamics": {
                "batch_size": 1,
            },
            "control_abstraction": "accel",
            "location": [0, 0, 2],  
            "rotation": [0.0, 0.0, 180],
        }
    ],
    "window_width": 1280,
    "window_height": 720,
}

env = biguasim.environments.BiguaSimEnvironment(
    scenario=config,
    binary_path=biguasim.packagemanager.get_binary_path_for_package("Competition"),
    show_viewport=True,
    verbose=False,
    uuid=str(uuid.uuid4()),
    ticks_per_sec=30,
)

env.reset()

alvo = [0, 0, 10, 0]              # ir para x=0 y=0 z=10, heading 0
for i in range(600):              # 20 segundos a 30 Hz
    estado = env.step(alvo, 1)
    if i % 30 == 0:
        pos = np.asarray(estado["uav0"][0]["DynamicsSensor"], dtype=float)[6:9]
        print(f"t={i/30:4.1f}s   pos={np.round(pos, 2)}")

print("fim")
