import gzip
import pickle
import os
import bz2file as bz2

class PreProcessingConfig:
    _cfg : dict = {}

    @classmethod
    def setup_cfg(cls, cfg : dict) -> None:
        cls._cfg = cfg

    @classmethod
    def get_configuration(cls) -> dict:
        return cls._cfg
    
    @classmethod
    def get_agent(cls, agent : str | int, also_id : bool = False) -> dict:     
        if isinstance(agent, int):
           return cls._cfg['agents'][agent]
        
        for i in range(len(cls._cfg['agents'])):
            if cls._cfg['agents'][i]['agent_type'] == agent:
                return cls._cfg['agents'][i] if also_id else cls._cfg['agents'][i]
            
            if 'agent_name' in cls._cfg['agents'][i].keys():
                if cls._cfg['agents'][i]['agent_name'] == agent:
                    return (i, cls._cfg['agents'][i]) if also_id else cls._cfg['agents'][i]
            
        return {}

        
    @classmethod
    def get_sensor(cls, agent : str | int  = None, sensor : str | int = None, also_id : bool = False) -> dict:
        _agent = cls.get_agent(agent, False)
        if _agent:
            if isinstance(sensor, int):
               return _agent['sensors'][sensor]
        
            elif isinstance(sensor, str):
                for i in range(len(_agent['sensors'])):
                    _sensor = _agent['sensors'][i]
                    if _sensor['sensor_type'] == sensor:
                        return (i, _sensor) if also_id else _sensor
                    if 'sensor_name' in _sensor.keys():
                        if _sensor['sensor_name'] == sensor:
                            return (i, _sensor) if also_id else _sensor

        return dict()
    
    @classmethod
    def add_sensor(cls, agent : str | int, sensor : dict) -> bool:
        try:
            _id, _agent = cls.get_agent(agent, True)
            _agent['sensors'].append(sensor)
            cls._cfg['agents'][_id] = _agent
            return True
        
        except:
            return False
        
    @classmethod
    def update_agent_sensor(cls, agent : str | int, sensor : dict) -> dict:
        try:
            _id_agent, _ = cls.get_agent(agent, True)

            _sensor = sensor['sensor_name'] if 'sensor_name' in sensor.keys() else sensor['sensor_type']
            _id_sensor, _ = cls.get_sensor(agent, _sensor)

            cls._cfg['agents'][_id_agent]['sensors'][_id_sensor].update(sensor)

            return cls._cfg['agents'][_id_agent]['sensors'][_id_sensor]

        except:
            return dict()

    @classmethod
    def update_sensor_config(cls, agent : str | int  = None, sensor : str | int = None, config : dict = {}) -> bool:
        try:
            _id_agent, _ = cls.get_agent(agent, True)

            _sensor = sensor['sensor_name'] if 'sensor_name' in sensor.keys() else sensor['sensor_type']
            _id_sensor, _ = cls.get_sensor(agent, _sensor)

            cls._cfg['agents'][_id_agent]['sensors'][_id_sensor]['configuration'].update(config)

            return cls._cfg['agents'][_id_agent]['sensors'][_id_sensor]['configuration']

        except:
            return dict()


class Storage:
    db  = "db_states.pkl"

    @classmethod
    def store_states(cls, state : dict, file_name : str = "") -> None:
        file = file_name or cls.db
        with open(file, "ab") as f:
            pickle.dump(state, f)

    @classmethod
    def load_states(cls, file_name : str = '') -> list:
        file = file_name or cls.db
        states = []
        with open(file, "rb") as f:
            while True:
                try:
                    state = pickle.load(f)
                    states.append(state)
                except EOFError:
                    break
        return states
        
    
    @classmethod
    def compress_db(cls, file_name : str = "") -> None:
        file = file_name or cls.db        
        with bz2.BZ2File(file + '.pbz2', 'wb') as f:
             with open(file, "rb") as dt:
                data = cls.load_states()
                os.remove(file)
                pickle.dump(data, f)

    @classmethod
    def decompress_db(cls, file_name : str = "") -> None:
        file = file_name or cls.db        
        states = []
    
        data = bz2.BZ2File(file + '.pbz2', 'rb')
        while True:
            try:
                states = pickle.load(data)
            except EOFError:
                break
        return states
       