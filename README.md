# Active SLAM Utility Function Exploiting Path Entropy

In this implementation, we present the work described in our paper cited below. We propose a utility function for frontier goal selection that exploits the occupancy grid map by utilizing path entropy and favors unknown map locations for maximum area coverage while maintaining low localization and mapping uncertainties. We quantify the efficiency of our method using various graph connectivity matrices and map efficiency indexes for environment exploration tasks.

**M. F. Ahmed, V. Frémont, and I. Fantoni, "Active SLAM Utility Function Exploiting Path Entropy," 2023 IEEE International Conference on Service Operations and Logistics, and Informatics (SOLI), Singapore, 2023, pp. 1-7, doi: 10.1109/SOLI60636.2023.10425063.**

Kindly cite this paper if you use this implementation.

## Dependencies
- ROS Noetic
- Ubuntu 20.04
- Open Karto
- ROSbot 2
- Kobuki plugins
- `sudo apt-get install libsuitesparse-dev`
## ROS Specific Dependencies
1. `sudo apt-get install ros-noetic-grid-map`
2. `sudo apt-get install ros-noetic-move-base`
