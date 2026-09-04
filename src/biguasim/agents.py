"""Definitions for different agents that can be controlled from BiguaSim"""
from functools import reduce

import numpy as np

from biguasim.spaces import ContinuousActionSpace
from biguasim.sensors import SensorDefinition, SensorFactory, RGBCamera
from biguasim.command import AddSensorCommand, RemoveSensorCommand




class BiguaSimAgent:
    """A learning agent in BiguaSim

    Agents can act, receive rewards, and receive observations from their sensors.
    Examples include the Android, UAV, and SphereRobot.

    Args:
        client (:class:`~biguasim.biguasimclient.BiguaSimClient`): The BiguaSimClient that this
            agent belongs with.
        name (:obj:`str`, optional): The name of the agent. Must be unique from other agents in
            the same environment.
        sensors (:obj:`dict` of (:obj:`str`, :class:`~biguasim.sensors.BiguaSimSensor`)): A list
            of BiguaSimSensors to read from this agent.

    Attributes:
        name (:obj:`str`): The name of the agent.
        sensors (dict of (string, :class:`~biguasim.sensors.BiguaSimSensor`)): List of
            BiguaSimSensors on this agent.
        agent_state_dict (dict): A dictionary that maps sensor names to sensor observation data.
    """

    def __init__(self, client, name="DefaultAgent"):
        self.name = name
        self._client = client
        self.agent_state_dict = dict()
        self.sensors = dict()

        # self._num_control_abstractions = len(self.control_abstractions)

        # self._max_control_abstraction_length = \
        #     max(map(lambda x: reduce(lambda i, j: i * j, x[1].buffer_shape),
        #             self.control_abstractions))

        self._max_control_abstraction_length = 6
        self._action_buffer = \
            self._client.malloc(name, [self._max_control_abstraction_length], np.float32)
        # Teleport flag: 0: do nothing, 1: teleport, 2: rotate, 3: teleport and rotate
        self._teleport_type_buffer = self._client.malloc(name + "_teleport_flag", [1], np.uint8)
        self._teleport_buffer = self._client.malloc(name + "_teleport_command", [12], np.float32)
        self._control_scheme_buffer = self._client.malloc(name + "_control_scheme", [1],
                                                          np.uint8)
        self._ocean_current_velocity = self._client.malloc(name + "_ocean_current_velocity", [3], np.float32)
        self._current_control_scheme = 0
        self.set_control_scheme(0)

    def act(self, action):
        """Sets the command for the agent. Action depends on the agent type and current control
        scheme.

        Args:
            action(:obj:`np.ndarray`): The action to take.
        """
        self.__act__(action)

    def clear_action(self):
        """Sets the action to zeros, effectively removing any previous actions.
        """
        np.copyto(self._action_buffer, np.zeros(self._action_buffer.shape))

    def set_control_scheme(self, index):
        """Sets the control scheme for the agent.

        Args:
            index (:obj:`int`): The control scheme to use. Should be set with an enum from
        """
        # self._current_control_abstraction = index % self._num_control_abstractions
        # self._control_scheme_buffer[0] = self._current_control_abstraction
        self._current_control_scheme = index
        self._control_scheme_buffer[0] = self._current_control_scheme

    def teleport(self, location=None, rotation=None):
        """Teleports the agent to a specific location, with a specific rotation.

        Args:
            location (np.ndarray, optional): An array with three elements specifying the target
                world coordinates ``[x, y, z]`` in meters (see :ref:`coordinate-system`).
                
                If ``None`` (default), keeps the current location.
            rotation (np.ndarray, optional): An array with three elements specifying roll, pitch,
                and yaw in degrees of the agent.
                
                If ``None`` (default), keeps the current rotation.

        """
        val = 0
        if location is not None:
            val += 1
            np.copyto(self._teleport_buffer[0:3], location)
        if rotation is not None:
            np.copyto(self._teleport_buffer[3:6], rotation)
            val += 2
        self._teleport_type_buffer[0] = val

    def set_physics_state(self, location, rotation, velocity, angular_velocity):
        """Sets the location, rotation, velocity and angular velocity of an agent.

        Args:
            location (np.ndarray): New location (``[x, y, z]`` (see :ref:`coordinate-system`))
            rotation (np.ndarray): New rotation (``[roll, pitch, yaw]``, see (see :ref:`rotations`))
            velocity (np.ndarray): New velocity (``[x, y, z]`` (see :ref:`coordinate-system`))
            angular_velocity (np.ndarray): New angular velocity (``[x, y, z]`` in **degrees** 
                (see :ref:`coordinate-system`))

        """
        np.copyto(self._teleport_buffer[0:3], location)
        np.copyto(self._teleport_buffer[3:6], rotation)
        np.copyto(self._teleport_buffer[6:9], velocity)
        np.copyto(self._teleport_buffer[9:12], angular_velocity)
        self._teleport_type_buffer[0] = 15

    def add_sensors(self, sensor_defs):
        """Adds a sensor to a particular agent object and attaches an instance of the sensor to the
        agent in the world.

        Args:
            sensor_defs (:class:`~biguasim.sensors.BiguaSimSensor` or
                         list of :class:`~biguasim.sensors.BiguaSimSensor`):
                Sensors to add to the agent.
        """
        if not isinstance(sensor_defs, list):
            sensor_defs = [sensor_defs]

        for sensor_def in sensor_defs:
            if sensor_def.agent_name == self.name:
                sensor = SensorFactory.build_sensor(self._client, sensor_def)
                self.sensors[sensor_def.sensor_name] = sensor
                self.agent_state_dict[sensor_def.sensor_name] = sensor.sensor_data

                if not sensor_def.existing:
                    command_to_send = AddSensorCommand(sensor_def)
                    self._client.command_center.enqueue_command(command_to_send)

    def remove_sensors(self, sensor_defs):
        """Removes a sensor from a particular agent object and detaches it from the agent in the
        world.

        Args:
            sensor_defs (:class:`~biguasim.sensors.BiguaSimSensor` or
                         list of :class:`~biguasim.sensors.BiguaSimSensor`):
                Sensors to remove from the agent.
        """
        if not isinstance(sensor_defs, list):
            sensor_defs = [sensor_defs]

        for sensor_def in sensor_defs:
            sensor = self.sensors.pop(sensor_def.sensor_name, None)
            self.agent_state_dict.pop(sensor_def.sensor_name, None)
            command_to_send = RemoveSensorCommand(self.name, sensor_def.sensor_name)
            self._client.command_center.enqueue_command(command_to_send)

            if sensor is not None:
                # The engine keeps this block mapped until it handles the command
                # above, so the release waits for the end of that tick.
                self._client.defer_free(sensor.key)

    def has_camera(self):
        """Indicatates whether this agent has a camera or not.

        Returns:
            :obj:`bool`: If the agent has a sensor or not
        """
        for sensor_type in self.sensors.items():
            if sensor_type is RGBCamera:
                return True

        return False

    @property
    def action_space(self):
        """Gets the action space for the current agent and control scheme.

        Returns:
            :class:`~biguasim.spaces.ActionSpace`: The action space for this agent and control
                scheme."""
        return self.control_abstractions[self._current_control_abstraction][1]

    @property
    def control_abstractions(self):
        """A list of all control schemes for the agent. Each list element is a 2-tuple, with the
        first element containing a short description of the control scheme, and the second
        element containing the :obj:`ActionSpace` for the control scheme.

        Returns:
            (:obj:`str`, :class:`~biguasim.spaces.ActionSpace`):
                Each tuple contains a short description and the ActionSpace
        """
        raise NotImplementedError("Child class must implement this function")

    def get_joint_constraints(self, joint_name):
        """Returns the corresponding swing1, swing2 and twist limit values for the
        specified joint. Will return None if the joint does not exist for the agent.
        """
        raise NotImplementedError("Child class must implement this function")

    def get_ocean_current_velocity(self):
        """Returns the current ocean current velocity for the agent."""
        return self._ocean_current_velocity

    def __act__(self, action):
        # Allow for smaller arrays to be provided as input
        if len(self._action_buffer) > len(action):
            action = np.copy(action)
            action.resize(self._action_buffer.shape)

        # The default act function is to copy the data,
        # but if needed it can be overridden
        np.copyto(self._action_buffer, action)

    def __repr__(self):
        return self.name


class BlueBoat(BiguaSimAgent):
    agent_type = "BlueBoat"

    @property
    def control_abstractions(self):
        scheme_accel = "[lin_accel_x, lin_accel_y, lin_accel_z, ang_accel_x, ang_accel_y, ang_accel_x]"
        thrusters = "[r1 thruster, r2 thruster]"
        cmd_vel = "[vx, vy, vz]"
        cmd_vel_yaw = "[vx, vy, vz, yaw_rate]"
        cmd_pos_yaw = "[vx, vy, vz, yaw]"
        
        return [(scheme_accel, ContinuousActionSpace([6], low=[-20] * 3 + [-2] * 3, high=[20] * 3 + [2] * 3)),
                (thrusters, ContinuousActionSpace([2], low=[-295.75], high=[288.08])),
                (cmd_vel, ContinuousActionSpace([3], low=[-10] * 3, high=[10]*3)),
                (cmd_vel_yaw, ContinuousActionSpace([4], low=[-10] * 3 + [-180] , high=[10] * 3 + [180])),
                (cmd_pos_yaw, ContinuousActionSpace([4], low=[-100] * 3 + [-180] , high=[10] * 3 + [180]))]


    def __repr__(self):
        return "BlueBoat " + self.name
    

class BlueROV2(BiguaSimAgent):
    agent_type = 'BlueROV2'

    @property
    def control_abstractions(self):
        scheme_accel = "[lin_accel_x, lin_accel_y, lin_accel_z, ang_accel_x, ang_accel_y, ang_accel_x]"
        thrusters = "[r1 thruster, r2 thruster, r3 thruster, r4 thruster, r5 thruster, r6 thruster]"
        cmd_vel = "[vx, vy, vz]"
        cmd_vel_yaw = "[vx, vy, vz, yaw_rate]"
        cmd_pos_yaw = "[vx, vy, vz, yaw]"
        
        return [(scheme_accel, ContinuousActionSpace([6], low=[-20] * 3 + [-2] * 3, high=[20] * 3 + [2] * 3)),
                (thrusters, ContinuousActionSpace([6], low=[-278.9], high=[278.9])),
                (cmd_vel, ContinuousActionSpace([3], low=[-10] * 3, high=[10]*3)),
                (cmd_vel_yaw, ContinuousActionSpace([4], low=[-10] * 3 + [-180] , high=[10] * 3 + [180])),
                (cmd_pos_yaw, ContinuousActionSpace([4], low=[-100] * 3 + [-180] , high=[10] * 3 + [180]))]



    def __repr__(self):
        return "BlueROV2 " + self.name
    
class BlueROVHeavy(BiguaSimAgent):
    agent_type = 'BlueROVHeavy'

    @property
    def control_abstractions(self):
        scheme_accel = "[lin_accel_x, lin_accel_y, lin_accel_z, ang_accel_x, ang_accel_y, ang_accel_x]"
        thrusters = "[r1 thruster, r2 thruster, r3 thruster, r4 thruster, r5 thruster, r6 thruster, r7 thruster, r8 thruster]"
        cmd_vel = "[vx, vy, vz]"
        cmd_vel_yaw = "[vx, vy, vz, yaw_rate]"
        cmd_pos_yaw = "[vx, vy, vz, yaw]"

        return [(scheme_accel, ContinuousActionSpace([6], low=[-20] * 3 + [-2] * 3, high=[20] * 3 + [2] * 3)),
                (thrusters, ContinuousActionSpace([8], low=[-278.9], high=[278.9])),
                (cmd_vel, ContinuousActionSpace([3], low=[-10] * 3, high=[10]*3)),
                (cmd_vel_yaw, ContinuousActionSpace([4], low=[-10] * 3 + [-180] , high=[10] * 3 + [180])),
                (cmd_pos_yaw, ContinuousActionSpace([4], low=[-100] * 3 + [-180] , high=[10] * 3 + [180]))]



    def __repr__(self):
        return "BlueROVHeavy " + self.name
    
class DjiMatrice(BiguaSimAgent):
    agent_type = 'DjiMatrice'

    @property
    def control_abstractions(self):
        scheme_accel = "[lin_accel_x, lin_accel_y, lin_accel_z, ang_accel_x, ang_accel_y, ang_accel_x]"
        thrusters = "[r1 thruster, r2 thruster, r3 thruster, r4 thruster]"
        cmd_vel = "[vx, vy, vz]"
        cmd_vel_yaw = "[vx, vy, vz, yaw_rate]"
        cmd_pos_yaw = "[vx, vy, vz, yaw]"
        
        return [(scheme_accel, ContinuousActionSpace([6], low=[-20] * 3 + [-2] * 3, high=[20] * 3 + [2] * 3)),
                (thrusters, ContinuousActionSpace([4], low=[0], high=[592.4])),
                (cmd_vel, ContinuousActionSpace([3], low=[-10] * 3, high=[10]*3)),
                (cmd_vel_yaw, ContinuousActionSpace([4], low=[-10] * 3 + [-180] , high=[10] * 3 + [180])),
                (cmd_pos_yaw, ContinuousActionSpace([4], low=[-100] * 3 + [-180] , high=[10] * 3 + [180]))]

    def __repr__(self):
        return "DjiMatrice " + self.name


class TorpedoAUV(BiguaSimAgent):
    agent_type = 'TorpedoAUV'

    @property
    def control_abstractions(self):
        scheme_accel = "[lin_accel_x, lin_accel_y, lin_accel_z, ang_accel_x, ang_accel_y, ang_accel_x]"
        scheme_rudders_sterns_motor_speed = "[rudder_top, rudder_bottom, stern_left, stern_right, rpm]"
        scheme_depth_heading_rpm_surge = "[depth, heading, rpm, surge]"
        
        
        return [(scheme_accel, ContinuousActionSpace([6], low=[-20] * 3 + [-2] * 3, high=[20] * 3 + [2] * 3)),
                (scheme_rudders_sterns_motor_speed, ContinuousActionSpace([5], low=[-90] * 4 + [-1525], high=[90] * 4 + [1525])),
                (scheme_depth_heading_rpm_surge, ContinuousActionSpace([4], low=[0, -180, -1525, -10], high=[100, 180, 1525, 10]))]


    def __repr__(self):
        return "TorpedoAUV " + self.name

#implementation of the competition drone 
class HolybroX500(BiguaSimAgent):
    agent_type = 'HolybroX500'

    @property
    def control_abstractions(self):
        scheme_accel = "[lin_accel_x, lin_accel_y, lin_accel_z, ang_accel_x, ang_accel_y, ang_accel_x]"
        thrusters = "[r1 thruster, r2 thruster, r3 thruster, r4 thruster]"
        cmd_vel = "[vx, vy, vz]"
        cmd_vel_yaw = "[vx, vy, vz, yaw_rate]"
        cmd_pos_yaw = "[vx, vy, vz, yaw]"
        
        return [(scheme_accel, ContinuousActionSpace([6], low=[-20] * 3 + [-2] * 3, high=[20] * 3 + [2] * 3)),
                (thrusters, ContinuousActionSpace([4], low=[0], high=[592.4])),
                (cmd_vel, ContinuousActionSpace([3], low=[-10] * 3, high=[10]*3)),
                (cmd_vel_yaw, ContinuousActionSpace([4], low=[-10] * 3 + [-180] , high=[10] * 3 + [180])),
                (cmd_pos_yaw, ContinuousActionSpace([4], low=[-100] * 3 + [-180] , high=[10] * 3 + [180]))]

    def __repr__(self):
        return "HolybroX500 " + self.name
    

#implementation of the competition drone: quad build of the Holybro Kopis X8
#Cinelifter 5" (Caged). The airframe is an X8 coaxial octo, but only the four
#TOP rotors are modelled -- the bottom layer is left off, so this is a plain
#QuadCopterX like the X500. The mass/inertia below still describe the complete
#physical airframe (all eight motors and the cage are on board).
class KopisX8(BiguaSimAgent):
    agent_type = 'KopisX8'

    @property
    def control_abstractions(self):
        scheme_accel = "[lin_accel_x, lin_accel_y, lin_accel_z, ang_accel_x, ang_accel_y, ang_accel_x]"
        thrusters = "[r1 thruster, r2 thruster, r3 thruster, r4 thruster]"
        cmd_vel = "[vx, vy, vz]"
        cmd_vel_yaw = "[vx, vy, vz, yaw_rate]"
        cmd_pos_yaw = "[vx, vy, vz, yaw]"

        return [(scheme_accel, ContinuousActionSpace([6], low=[-20] * 3 + [-2] * 3, high=[20] * 3 + [2] * 3)),
                (thrusters, ContinuousActionSpace([4], low=[0], high=[2950.0])),
                (cmd_vel, ContinuousActionSpace([3], low=[-10] * 3, high=[10]*3)),
                (cmd_vel_yaw, ContinuousActionSpace([4], low=[-10] * 3 + [-180] , high=[10] * 3 + [180])),
                (cmd_pos_yaw, ContinuousActionSpace([4], low=[-100] * 3 + [-180] , high=[10] * 3 + [180]))]

    def __repr__(self):
        return "KopisX8 " + self.name
    
######################################################################################################################################################
    

class AgentDefinition:
    """Represents information needed to initialize agent.

    Args:
        agent_name (:obj:`str`): The name of the agent to control.
        agent_type (:obj:`str` or type): The type of BiguaSimAgent to control, string or class
            reference.
        sensors (:class:`~biguasim.sensors.SensorDefinition` or class type (if no duplicate sensors)): A list of
            BiguaSimSensors to read from this agent.
        starting_loc (:obj:`list` of :obj:`float`): Starting ``[x, y, z]`` location for agent 
            (see :ref:`coordinate-system`)
        starting_rot (:obj:`list` of :obj:`float`): Starting ``[roll, pitch, yaw]`` rotation for agent 
            (see :ref:`rotations`)
        existing (:obj:`bool`): If the agent exists in the world or not (deprecated)
    """

    _type_keys = {
        "BlueBoat": BlueBoat,
        "BlueROV2": BlueROV2,
        "BlueROVHeavy" : BlueROVHeavy,
        "DjiMatrice" : DjiMatrice,
        "TorpedoAUV": TorpedoAUV,
        "HolybroX500": HolybroX500,
        "KopisX8": KopisX8,
    }

    def __init__(self, agent_name, agent_type, sensors=None, starting_loc=(0, 0, 0),
                 starting_rot=(0, 0, 0), existing=False, is_main_agent=False):
        self.starting_loc = starting_loc
        self.starting_rot = starting_rot
        self.existing = existing
        self.sensors = sensors or list()
        self.is_main_agent = is_main_agent
        for i, sensor_def in enumerate(self.sensors):
            if not isinstance(sensor_def, SensorDefinition):
                self.sensors[i] = \
                    SensorDefinition(agent_name, agent_type,
                                     sensor_def.sensor_type, sensor_def)
        self.name = agent_name

        if isinstance(agent_type, str):
            self.type = AgentDefinition._type_keys[agent_type]
        else:
            self.type = agent_type


class AgentFactory:
    """Creates an agent object
    """

    @staticmethod
    def build_agent(client, agent_def):
        """Constructs an agent

        Args:
            client (:class:`biguasim.biguasimclient.BiguaSimClient`): BiguaSimClient agent is
                associated with
            agent_def (:class:`AgentDefinition`): Definition of the agent to instantiate

        Returns:

        """
        return agent_def.type(client, agent_def.name)
    


