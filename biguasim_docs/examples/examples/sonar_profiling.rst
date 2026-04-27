.. _visualizing_profiling_sonar:

Visualizing Profiling Sonar
============================

It can be useful to visualize the output of the sonar sensor during a simulation. This script will do 
that, plotting each time sonar data is received.

Note, running this script will create octrees while running and may cause some pauses. See 
:ref:`octree` for workarounds and more info.
::

    import biguasim
    import numpy as np
    import matplotlib.pyplot as plt

    class SonarPolarVisualizer:
        def __init__(self, config):
            self.azi = config['Azimuth']
            self.minR = config['RangeMin']
            self.maxR = config['RangeMax']
            self.binsR = config['RangeBins']
            self.binsA = config['AzimuthBins']

            self.half_elev = config.get('Elevation', 1.0) / 2.0
            
            plt.ion()
            self.fig, self.axes = plt.subplots(1, 3, subplot_kw=dict(projection='polar'), figsize=(18, 6))
            
            titles = ["Raw Signal (Noise)", "GT Intensity", "GT Elevation"]
            cmaps = ['inferno', 'magma', 'RdBu_r'] # RdBu_r é ótimo para ver desvios de elevação
            
            theta = np.linspace(np.radians(-self.azi/2), np.radians(self.azi/2), self.binsA)
            r = np.linspace(self.minR, self.maxR, self.binsR)
            self.T, self.R = np.meshgrid(theta, r)
            
            self.plots = []
            for i, ax in enumerate(self.axes):
                ax.set_title(titles[i], pad=20, fontsize=12, fontweight='bold')
                ax.set_theta_zero_location("N")
                ax.set_theta_direction(-1) # Direção padrão para Sonar (Starboard = positivo)
                ax.set_thetamin(-self.azi/2)
                ax.set_thetamax(self.azi/2)
                
                z_init = np.zeros((self.binsR, self.binsA))
                
                vmin = -self.half_elev if i == 2 else 0
                vmax = self.half_elev if i == 2 else 1
                
                mesh = ax.pcolormesh(self.T, self.R, z_init, cmap=cmaps[i], shading='nearest', vmin=vmin, vmax=vmax)
                self.plots.append(mesh)
                plt.colorbar(mesh, ax=ax, fraction=0.046, pad=0.04)

            self.fig.tight_layout()

        def update(self, noise, gt, elev):
            self.plots[0].set_array(noise.ravel())
            self.plots[1].set_array(gt.ravel())

            elev_masked = np.ma.masked_where(elev < -90, elev)
            self.plots[2].set_array(elev_masked.ravel())

            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    if __name__ == '__main__': 
        config = {
            "package_name": "SkyDive",
            "world": "Bridge",
            "main_agent": "auv0",
            "ticks_per_sec": 30,
            "frames_per_sec": True,
            "octree_min": 0.02,
            "octree_max": 5.0,
            "agents":[
                {
                    "agent_name": "auv0",
                    "agent_type": "BlueROV2",
                    "sensors": [
                        {
                            "sensor_type": "DynamicsSensor",
                            "socket": "IMUSocket",
                            "configuration": {
                                "UseCOM": True,
                                "UseRPY": False  
                            }
                        },
                        {
                            "sensor_type": "ProfilingSonar",
                            "socket": "SonarSocket",
                            "Hz": 2,
                            "configuration": {
                                "Azimuth": 120,
                                "Elevation": 1,
                                "RangeMin": 1,
                                "RangeMax": 60,
                                "RangeBins": 512,
                                "AzimuthBins": 512,
                                "AddSigma": 0.15,
                                "MultSigma": 0.2,
                                "MultiPath": True,
                                "ClusterSize": 5,
                                "ScaleNoise": True,
                                "AzimuthStreaks": -1,
                                "RangeSigma": 0.1,
                                "ShowWarning": True,
                                "InitOctreeRange": 70,
                                "ViewRegion": True,
                                "ViewOctree": -10,
                                "SendGTIntensity" : True,
                                "SendGTElevation" : True
                            }
                        }
                    ],
                    "dynamics" : {
                        "batch_size" : 1,
                    },
                    "control_abstraction": "cmd_vel",
                    "location": [35,20,-8.5],
                    "rotation": [0.0, 0.0, 90]
                }
            ]
        }

    
        sonar_cfg = None
        for agent in config['agents']:
            for sensor in agent['sensors']:
                if sensor['sensor_type'] == 'ProfilingSonar':
                    sonar_cfg = sensor['configuration']
                    break

        vis = SonarPolarVisualizer(sonar_cfg)

        with biguasim.make(scenario_cfg=config) as env:
    
            while True:

                command = [0, 0, 0]    
                state = env.step(command)["auv0"][0]
                print(state)
                if "ProfilingSonar" in state:

                    sonar_packet = state["ProfilingSonar"]

                    noise = sonar_packet["raw"]
                    gt_int = sonar_packet["gt_intensity"] 
                    gt_elev = sonar_packet["gt_elevation"]  

                    vis.update(noise, gt_int, gt_elev)