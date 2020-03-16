# Trajectory planning for a quadrotor UAV with suspended payload
This project implements a trajectory planner for a 2D model of a quadrotor UAV, with a suspended payload, through confined spaces. To better understand the project, here is a link to a video I made.

[![Alt text](https://img.youtube.com/vi/YTAeVl-ViNA/0.jpg)](https://www.youtube.com/watch?v=YTAeVl-ViNA)



## Trajectory Planner
The trajectory planner combines the Astar search algorithm with input shaping actions. Input shaping is an open-loop technique for generating oscillation free responses.

The trajectory planner is implemented in python. Example scenarios is implemented as ipython notebooks, with the planned sequences saved in the folder "Example Trajectories". The output of the trajectory planner needs to be saved as ".mat" file, to be loaded from the simulink model.

## Simulation model

This is a matlab simulation model for a quadrotor UAV with suspended payload. The simulation model includes:
- Sensor noise 
- Motor transient response
- Internal controllers such as tilt angle controller
- Trajectory regulator to execute a given trajectory

To load, the setup_simulation.m file needs to be called.
