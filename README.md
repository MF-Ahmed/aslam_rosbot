# Active SLAM Utility Function Exploiting Path Entropy

In this implementation, we present the work described in our paper cited below. We propose a utility function for frontier goal selection that exploits the occupancy grid map by utilizing path entropy and favors unknown map locations for maximum area coverage while maintaining low localization and mapping uncertainties. We quantify the efficiency of our method using various graph connectivity matrices and map efficiency indexes for environment exploration tasks.

**M. F. Ahmed, V. Frémont, and I. Fantoni, "Active SLAM Utility Function Exploiting Path Entropy," 2023 IEEE International Conference on Service Operations and Logistics, and Informatics (SOLI), Singapore, 2023, pp. 1-7, doi: 10.1109/SOLI60636.2023.10425063.**

Kindly cite this paper if you use this implementation.

## Dependencies
- **ROS Noetic**: Follow the installation instructions [here](http://wiki.ros.org/noetic/Installation).
- **Ubuntu 20.04**: This project is designed for Ubuntu 20.04. Ensure you are using this version of Ubuntu.
- **g2o**: Install from the official repository at [g2o GitHub](https://github.com/RainerKuemmerle/g2o).
- **Open Karto**: Install Open Karto following its official instructions.
- **ROSbot 2**: Ensure the hardware and firmware are set up as per the ROSbot 2 documentation.
- **Kobuki Plugins**: Install necessary plugins for compatibility with the Kobuki base.
- **Libraries**: 
  ```bash
  sudo apt-get install libsuitesparse-dev
  ```
## ROS Specific Dependencies
 ```bash
    sudo apt-get install ros-noetic-grid-map
    sudo apt-get install ros-noetic-move-base
    sudo apt-get install ros-noetic-robot-localization
    sudo apt-get install ros-noetic-costmap-converter
    sudo apt-get install ros-noetic-open-karto
    sudo apt-get install ros-noetic-kobuki-core
