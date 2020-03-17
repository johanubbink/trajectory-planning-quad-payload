# Trajectory planning for a quadrotor UAV with suspended payload
This project implements a trajectory planner for a 2D model of a quadrotor UAV, with a suspended payload, through confined spaces. Here is a link to a presentation I gave about the planner.

[![Trajectory planning](https://img.youtube.com/vi/YTAeVl-ViNA/0.jpg)](https://www.youtube.com/watch?v=YTAeVl-ViNA)


## Trajectory Planner
The trajectory planner combines the Astar search algorithm with input shaping actions. Input shaping is an open-loop technique for generating oscillation free responses.

The trajectory planner is implemented in python. Example scenarios is implemented as ipython notebooks, with the planned sequences saved in the folder "Example Trajectories". The output of the trajectory planner needs to be saved as ".mat" file, to be loaded from the simulink model.

### Dependencies
The following python packages are required for the planning:
- heapq
- numpy
- matplotlib

## Simulation model

This is a matlab simulation model for a quadrotor UAV with suspended payload. The simulation model includes:
- Sensor noise 
- Motor transient response
- Internal controllers such as tilt angle controller
- Trajectory regulator to execute a given trajectory
- External wind disturbances

The simulation model is implemented as a simulink model, with the name "quad_payload_model.slx".
To load the needed parameters for simulation, setup_simulation.m file needs to be called. This file takes the planned trajectory from the ".mat" file, and converts it to be used with the simulation model.

### Dependencies
The following matlab and simulink packages are required:
- Simulink
- Aerospace blockset
