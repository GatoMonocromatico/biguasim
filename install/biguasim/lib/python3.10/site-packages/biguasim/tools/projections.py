import numpy as np

from utils import PreProcessingConfig


cfg = {                                                       
    "package_name": "SkyDive",               
    "world": "Relief-Generic-Bridge-Vehicles",                            
    "main_agent": "uav0",                               
    "ticks_per_sec": 20,                                
    "frames_per_sec": True,                             
    "octree_min": 0.02,                                 
    "octree_max": 1.0,                                  
    "agents":[                                          
        {                                               
            "agent_name": "uav0",                       
            "agent_type": "DjiMatrice",                
            "sensors": [                                
                {                                       
                    "sensor_type": "PoseSensor",        
                    "socket": "IMUSocket"               
                },                                      
                {                                       
                    "sensor_type": "IMUSensor",         
                    "socket": "IMUSocket",              
                    "Hz": 20,                           
                    "configuration": {                  
                        "AccelSigma": 0.00277,          
                        "AngVelSigma": 0.00123,         
                        "AccelBiasSigma": 0.00141,      
                        "AngVelBiasSigma": 0.00388,     
                        "ReturnBias": True           
                    }                                   
                },                                      
                {   "sensor_type": "AnnotationComponent",         
                    "sensor_name": "RightCamera",       
                    "socket": "CameraSocket",      
                    "configuration": {                  
                        "convertToDistance" : False,
                        "CaptureWidth": 256,        #Precisa ser 256    
                        "CaptureHeight": 256           
                    }           
                },                                      
                {                                       
                    "sensor_type": "FOVDetection",      
                    "socket": "CameraSocket",      
                    "configuration":{                   
                        "TargetDetection" : "Bus*"        
                    }                                   
                                                        
                },
                {                                       
                    "sensor_type": "RGBCamera",         
                    "sensor_name": "Camera",       
                    "socket": "CameraSocket",      
                    "configuration": {                  
                        "CaptureWidth": 256,            
                        "CaptureHeight": 256            
                    }                                   
                },
                {                                       
                    "sensor_type": "RGBDCamera",         
                    "sensor_name": "RGBDCamera",       
                    "socket": "CameraSocket",      
                    "configuration": {                
                        "convertToDistance" : True,  
                        "CaptureWidth": 32,            
                        "CaptureHeight": 32           
                    }                                   
                }                      
                                                        
            ],                                          
            "control_scheme": 0,                        
            "location": [-50, 0, 4],                     
            "rotation": [0.0, 0.0, 0]                 
        }                                               
    ],                                                  
                                                        
    "window_width":  1280,                              
    "window_height": 720                                
} 

class Projections:

    @staticmethod
    def _round_to_nearest_multiple_of_2(x):

        return round(x / 2) * 2 if x % 2 == 1 else x
    
    
    @staticmethod
    def _is_prime(x : int) -> bool:
        for i in range(2, x):
            if x % i == 0:
                return False
        return True
    
    @staticmethod
    def _max_odd(x : int) -> int:
        div = 2
        v = []
        while x / div >= 1:
            if x % div == 0:
                x = x / div
                v.append(div)
            else:
                div += 1

        return int(max(v))
    
    @classmethod
    def _closest_to(cls, x):
        return cls._round_to_nearest_multiple_of_2(x)
       
    
    @classmethod
    def _clustering(cls, x):        
        log = np.log2(x)
        if log.is_integer():
            while x % log != 0:
               log += 1

            return int(log)
        
        return cls._max_odd(x)
    
    @classmethod
    def _image_clustering(cls, h, w):
        return (cls._clustering(h), cls._clustering(w))


    @classmethod
    def imaging_sonar_2_depth_camera(cls, cfg : dict, agent : str | int  = None, sensor : str | int = None) -> dict:
        PreProcessingConfig.setup_cfg(cfg)
        _agent = PreProcessingConfig.get_agent(agent)
        _sonar = PreProcessingConfig.get_sensor(agent, sensor or 'ImagingSonar')
        _sonar_cfg = _sonar['configuration']

        _capture_height  =  ((np.tan(np.deg2rad(_sonar_cfg["Elevation"]/2))*2*_sonar_cfg["RangeMax"]) /
                            ((np.tan(np.deg2rad(_sonar_cfg["Azimuth"]/2))*2*_sonar_cfg["RangeMax"])/
                            _sonar_cfg["AzimuthBins"]))

        _fov_angle = np.arctan((np.deg2rad(_sonar_cfg["Azimuth"]/2))/(np.tan(np.deg2rad(_sonar_cfg["Elevation"]/2))))
        
        _camera = {
            "sensor_type":"RGBDCamera",
            "socket": _sonar['socket'],
            "sensor_name":"DepthGroundTruth",
            "rotation": _agent['rotation'] if not 'rotation' in _sonar.keys() else _sonar['rotation'],
            "configuration":{
                "CaptureWidth":_sonar_cfg["AzimuthBins"],
                "CaptureHeight":int(_capture_height),
                "FovAngle":np.rad2deg(_fov_angle),
                "convertToDistance":True,
            }
        }

        PreProcessingConfig.add_sensor(agent, _camera)
        return PreProcessingConfig.get_configuration()
    
    @classmethod
    def imaging_sonar_ground_truth(cls, cfg : dict, agent : str | int  = None, sensor : str | int = None)-> dict:
        PreProcessingConfig.setup_cfg(cfg)
        _agent = PreProcessingConfig.get_agent(agent)
        _sonar = PreProcessingConfig.get_sensor(agent, sensor or 'ImagingSonar')
        _sonar_cfg = _sonar['configuration']

        _rotation = _agent['rotation'] if not 'rotation' in _sonar.keys() else _sonar['rotation']
        
        for t in range(int(_sonar_cfg["AzimuthBins"])):
            for p in range(int(_sonar_cfg["Elevation"])):
                rotation = [0,_rotation[1],(t*(_sonar_cfg["Azimuth"]/_sonar_cfg["AzimuthBins"]))-_sonar_cfg["Azimuth"]/2]
                _gt_sensor = { "sensor_type":"RangeFinderSensor",
                                "sensor_name":(f"{t} {p}"),
                                "socket": _sonar['socket'],
                                "rotation":rotation,
                                "configuration":{
                                    "LaserMaxDistance": _sonar_cfg["RangeMax"],
                                    "LaserCount": 1,
                                    "LaserAngle":p-int(_sonar_cfg["Elevation"])/2,
                                    "LaserDebug": True,
                                }
                            }
                
                PreProcessingConfig.add_sensor(agent, _gt_sensor)

        return PreProcessingConfig.get_configuration()
    
    @classmethod
    def camera_ground_truth(cls, cfg : dict, agent : str | int  = None, sensor : str | int = None)-> dict:
        PreProcessingConfig.setup_cfg(cfg)
        _agent = PreProcessingConfig.get_agent(agent)
        _camera = PreProcessingConfig.get_sensor(agent, sensor)
        _camera_cfg = _camera['configuration']
        h = cls._closest_to(_camera_cfg["CaptureHeight"])
        w = cls._closest_to(_camera_cfg["CaptureWidth"])

        _camera_cfg["CaptureHeight"] = h
        _camera_cfg["CaptureWidth"] = w
        _camera_cfg = PreProcessingConfig.update_sensor_config(agent, sensor, _camera_cfg)

        _rotation = _agent['rotation'] if not 'rotation' in _camera.keys() else _camera['rotation']

        _fov = _camera_cfg['FovAngle'] if 'FovAngle' in _camera_cfg.keys() else 90
        _fov_rad = np.deg2rad(_fov)

        fy = (h / 2) / np.tan(_fov_rad / 2)
        fx = (w / 2) / np.tan(_fov_rad / 2)

        theta_y = np.linspace(fy / 2, -fy / 2, h)
        theta_x = np.linspace(fx / 2, -fx / 2, w)


        cluster_t, cluster_p = cls._image_clustering(h, w)        
        pixel_t = int(cluster_t / 2) - 1 
        pixel_p = int(cluster_p / 2) - 1 

        h += 1
        w += 1

        for t in range(0, h , cluster_t):
            for p in range(0, w, cluster_p):
                if t > 0 and p > 0:
                    rotation = [0,_rotation[1],theta_x[p - pixel_p]]
                    _gt_sensor = { "sensor_type":"RangeFinderSensor",
                                    "sensor_name":(f"{t} {p}"),
                                    "socket": _camera['socket'],
                                    "rotation":rotation,
                                    "configuration":{
                                        "LaserMaxDistance": 100,
                                        "LaserCount": 1,
                                        "LaserAngle":theta_y[t - pixel_t],
                                        "LaserDebug": True,
                                    }
                                }
                    PreProcessingConfig.add_sensor(agent, _gt_sensor)

        return PreProcessingConfig.get_configuration()

#  True if f"{t} {p}" == "8 8" or f"{t} {p}" == "8 32" or f"{t} {p}" == "32 8" or f"{t} {p}" == "32 32" else False,
if __name__ == "__main__":
    a = Projections.camera_ground_truth(cfg, 'uav0', 'RGBDCamera')

    print(a)