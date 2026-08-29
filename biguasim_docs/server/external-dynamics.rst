.. _external-dynamics:

=====================================
Vehicles the World Does Not Integrate
=====================================

By default the world integrates every vehicle: a client sends a control command,
the world's dynamics model works out where the vehicle goes, and the world owns
the answer.

An agent spawned with ``externally_driven=True`` inverts that. It is left out of
the world's dynamics entirely; its owner works out where it is and says so with
:class:`~biguasim.server.actions.SetPose`. The world still does collision and
sensor simulation for it, and still decides what everyone else sees.


Why bother
==========

Three things this makes possible, none of which the default mode can do:

**A custom dynamics model.** Someone researching a vehicle model that is not in
:mod:`biguasim.dynamics` can fly it in a shared world without first getting it
merged into the simulator.

**Hardware in the loop.** A real flight controller, or an ArduPilot SITL
instance, closing its loop on the client's machine at whatever rate it likes,
with the world providing the environment around it.

**A real vehicle.** An actual airframe, reporting its actual position, appearing
in the simulated world alongside simulated ones. The world does not need to know
which is which.

It also makes latency moot for whoever is flying it: their vehicle's physics is
already on their machine, so there is nothing to hide.


The two modes are kept apart
============================

An externally driven agent refuses ``SetControl``. A world-driven one refuses
``SetPose``. Both errors say which to use instead.

This is not fussiness. Accepting both would mean two things deciding where a
vehicle is -- the world's integrator and the client's -- with no principled way
to say which is right when they disagree. Better to make the question impossible
to ask.


Collision stays the world's call
================================

A client integrating its own vehicle has no idea where the piers are. It will
happily fly through one.

So the world keeps authority over contact. When a client-driven vehicle hits
something, the world tells its owner where it actually ended up:

.. code-block:: python

   for correction in world.corrections():
       tick, agent, pose = correction["tick"], correction["agent"], correction["pose"]
       # this client chooses to accept it
       my_state.position = pose["position"]

Verified by flying one into the terrain: the client asks to be at z = -80, the
world holds it at -79.4 and says so each tick until the client stops asking.

**What to do with a correction is left to the client**, deliberately. Snapping is
right for a simulated vehicle. Blending is right for something a human is
watching. Ignoring it may genuinely be right for a real airframe, where the
world's opinion about a collision that did not physically happen is simply
wrong. Only the client knows which case it is in, so only the client decides.


Using it
========

.. code-block:: python

   world.spawn_agent(
       "rig", "DjiMatrice",
       location=(0.0, 0.0, 60.0),
       externally_driven=True,
       sensors=[DYNAMICS_SENSOR, COLLISION_SENSOR])

   while flying:
       state = my_own_dynamics.step(...)
       world.set_pose("rig", position=state.position, velocity=state.velocity)
       for correction in world.corrections():
           my_own_dynamics.accept(correction["pose"])

A ``CollisionSensor`` is what makes corrections possible, so an externally driven
agent that wants them needs one.
