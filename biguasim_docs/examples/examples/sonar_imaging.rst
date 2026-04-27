.. _visualizing_imaging_sonar:

==========================
Visualizing Imaging Sonar
==========================

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
            self.config = config
            
            self.azi = config['Azimuth']
            self.minR = config['RangeMin']
            self.maxR = config['RangeMax']
            self.binsR = config['RangeBins']
            self.binsA = config['AzimuthBins']
            
            self.half_elev = config.get('Elevation', 20) / 2.0
            
            plt.ion()
            self.fig, self.axes = plt.subplots(1, 3, subplot_kw=dict(projection='polar'), figsize=(18, 6))
            
            self.axes[0].set_title("Real (Raw)")
            self.axes[1].set_title("GT (Intensity)")
            self.axes[2].set_title("GT (Elevation)")

            theta = np.linspace(np.radians(-self.azi/2), np.radians(self.azi/2), self.binsA)
            r = np.linspace(self.minR, self.maxR, self.binsR)
            self.T, self.R = np.meshgrid(theta, r)
            
            z_empty = np.zeros_like(self.T)
            self.plots = []
            
            cmaps = [plt.get_cmap('inferno'), plt.get_cmap('magma'), plt.get_cmap('gray')]
            cmaps[2].set_bad(color='magenta') 
            
            vmins = [0, 0, -self.half_elev]
            vmaxs = [1, 1, self.half_elev]

            for i, ax in enumerate(self.axes):
                ax.set_theta_zero_location("N")
                ax.set_thetamin(-self.azi/2)
                ax.set_thetamax(self.azi/2)
                ax.grid(False)
                ax.set_yticklabels([])
                
                mesh = ax.pcolormesh(self.T, self.R, z_empty, cmap=cmaps[i], shading='nearest', vmin=vmins[i], vmax=vmaxs[i])
                self.plots.append(mesh)
                
                cbar = plt.colorbar(mesh, ax=ax, fraction=0.046, pad=0.04)
                if i == 2:
                    cbar.set_label("Elevation (°)")

            self.fig.tight_layout()

        def update(self, noise, gt, elev):

            self.plots[0].set_array(noise.ravel())

            self.plots[1].set_array(gt.ravel())
    
            valid_data = elev[elev > -900]
            if valid_data.size > 0:
                pass

            elev_viz_masked = np.ma.masked_where(elev < -90, elev)
            self.plots[2].set_array(elev_viz_masked.ravel())

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
                            "sensor_type": "ImagingSonar",
                            "sensor_name": "ImagingSonar",
                            "socket": "SonarSocket",
                            "Hz": 5,
                            "configuration": {
                                "RangeBins": 394,
                                "AzimuthBins": 768,
                                "RangeMin": 0.5,
                                "RangeMax": 10,
                                "InitOctreeRange": 20,
                                "Elevation": 20,
                                "Azimuth": 130,
                                "MultiPath": True,
                                "ViewRegion": True,
                                "AddSigma": 0.5,
                                "MultSigma": 0.5,
                                "ScaleNoise": True,
                                "ViewOctree" : -10,
                                "SendGTIntensity": True, 
                                "SendGTElevation": True, 
                                "SendGTPointCloud": False 
                            },
                        }
                    ],
                    "dynamics" : {
                        "batch_size" : 1,
                    },
                    "control_abstraction": "cmd_vel",
                    "location": [35,20,-7],
                    "rotation": [0.0, 0.0, 90]
                }
            ]
        }

    
        sonar_cfg = None
        for agent in config['agents']:
            for sensor in agent['sensors']:
                if sensor['sensor_type'] == 'ImagingSonar':
                    sonar_cfg = sensor['configuration']
                    break

        vis = SonarPolarVisualizer(sonar_cfg)

        with biguasim.make(scenario_cfg=config) as env:
    
            while True:

                command = [0, 0, 0]    
                state = env.step(command)["auv0"][0]

                if "ImagingSonar" in state:

                    sonar_packet = state["ImagingSonar"]

                    noise = sonar_packet["raw"]
                    gt_int = sonar_packet["gt_intensity"] 
                    gt_elev = sonar_packet["gt_elevation"]  

                    vis.update(noise, gt_int, gt_elev)