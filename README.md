# Autonomous overtaking maneuver under complex driving conditions

## Abstract
While existing solutions attempt multi-lane overtaking involving simple and
static scenarios, the focus here is on single lane overtaking which require minimal intrusion
on to the adjacent lane in dynamically changing conditions.The method proposed utilizes
a heuristic rule-based strategy to select optimal maneuvers and then uses a combination of
safe and reachable sets to iteratively generate intermediate reference targets based on the
desired maneuver. A nonlinear model predictive controller then plans dynamically feasible
trajectories to these intermediate reference targets that avoid collisions. The proposed
method was implemented and tested under 7 different scenarios that cover many complex
lane-keeping and overtaking scenarios using the CARLA simulation engine with ROS
(Robotic Operating System) framework for inter-component communication with model
predictive controller developed using MATLAB. In every tested scenario, the proposed
planning and control paradigm was able to select the best course of action (maneuver)
and execute the same without collisions with other nearby vehicles.

## Proposed Architecture
 ![Alt text](images/ProposedPipeline.svg?raw=true "Proposed Architecture") 

## Project Structure
```bash
├───Functions     // All MATLAB functions that are used for planning and control scheme 
├───refs    // All referenced Vehicle dynamics models
├───Scenario_definitions  // All definition files that describe different overtaking scenarios, postions of other actors, lane configurations etc.
├───Scripts    // Folder containing the scripts that initialse parameters and necessary structures in the base workspace.
    ├───init_mpc.m  // Contains parameters pertaining to MPC formulation. 
    └───SimulationParameters.m   // Vehicle model and other related parameters
├───System   // Containing Main model files.
    └───Overtaking_Integrated.slx  // Main Simulink model file
    ├───AOTDataDictionary.sldd     // Data dictionary
└───Work
```   



## Results
 ![Alt text](results/Abort_TopView.svg?raw=true "Title") 


