# Trajectory Planning Using RRT and pOGMs

This repository provides MATLAB code for trajectory planning using the Rapidly-exploring Random Tree (RRT) algorithm and probabilistic Occupancy Grid Maps (pOGMs).
![RRT Algorithm](https://github.com/user-attachments/assets/5c77caa2-ebb3-47f0-abae-539c81f03ec6)

## Features

- RRT algorithm for safe trajectory planning.
- Integration with probabilistic occupancy grid maps for obstacle avoidance.

## Requirements

- MATLAB (Tested with R2023b)
- No additional toolboxes required

## 📂 Repository Structure

- `data` – Folder to contain the predicted Occupancy Grid Maps (pOGMs) data. 
- `utils/` – Contains the utility files to execute the RRT algorithm. 
- `RRT.m` - Script to plan a safe trajectory for the ego vehicle based on the RRT algorithm using the pOGM as input.
- `RRTAccProfile.m` - Script to plan the safe trajectory for the ego vehicle based on the RRT agorithm with different longitudinal acceleration profiles using the pOGM as input.  
## Getting Started

1. **Run the main script:**
    - In MATLAB, run:
      ```matlab
      run('RRT.m')
      ```
    - To check the trajectories with different acceleration profiles, run:
      ```matlab
      run('RRTAccProfile.m')
      ```
