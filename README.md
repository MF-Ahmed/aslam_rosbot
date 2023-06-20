Thesis WS
============

Collaboration in map built by multiple ROSbot

This branch contains the `ROS1 noetic` simulation and the link to the `ROS2 foxy` workspace where there is also the map merging node. <br>
**Make sure to have them installed** <br><br>
To have the correct code working follow these steps

## `ROS1` workspace initialization
In the root folder:
```bash
cd ~/thesis_ws/src/
git clone https://github.com/mmatteo-hub/thesis_ws/edit/karto_slam.git
cd ~/thesis_ws/
catkin_make
```
## `ROS2` workspace initialization
The code for the other node is [here](https://github.com/mmatteo-hub/map_merging_node). Follow also the instructions for the ros bridge in the dedicated folder.<br>
Once finished return to the root folder, then
```bash
cd ~/ros2_ws/src/
git clone https://github.com/mmatteo-hub/ros2_ws.git
cd ~/ros2_ws/
colcon build
```

### Change parameters
Before running, remeber to change few parameters., otherwisethe code will throw errors:
* path value in [constants.py](/src/graph_d_exploration/scripts/constants.py) file
* path value in `mapping_karto_g20.yaml` file for each of the robots:
    * [robot_0 file](/src/graph_d_exploration/param/robot_0/mapping_karto_g2o.yaml)
    * [robot_1 file](/src/graph_d_exploration/param/robot_1/mapping_karto_g2o.yaml)
    * [robot_2file ](/src/graph_d_exploration/param/robot_2/mapping_karto_g2o.yaml)
* path value in [run.py]() file, in particular:
    * `ros1_path_setup_ws`: the path of the `devel` folder of the `ROS1` workspace
    * `ros2_path_setup_ws`: the path of the `install` folder of the `ROS2` workspace


## Program start
*Suggestion*: do not source `ROS-*` automatically from the `.bashrc` file <br><br>
To start the program you will need 7 different shells<br><br>

```bash
# Shell A
source ros-noetic
roscore
```

```bash
# Shell B
source ros-noetic
source ros-noetic-ws
roslaunch graph_d_exploration file.launch
```

```bash
# Shell C
source ros2-foxy
source ros2-foxy-ws
ros2 run ros1_bridge dynamic_bridge 
```

```bash
# Shell D
source ros2-foxy
source ros2-foxy-ws
ros2 launch merge_map merge_map_launch.py 
```

```bash
# Shell E
source ros-noetic
source ros-noetic-ws
roslaunch graph_d_exploration servers.launch
```

```bash
# Shell F
source ros-noetic
source ros-noetic-ws
roslaunch graph_d_exploration multiple.launch
```

### Plotting Node
In order to have plots, there is a dedicated node that has to be run separately from. <br>
To run it type:
```bash
# Shell G
source ros-noetic
source ros-noetic-ws
rosrun plotting plot.py
```

The node has to be run once the simulation starts since it needs the topic to subscribe to in order to record data. <br>
Once the data recorded are sufficient, or when the simulation stops remember to stop the node manually, with `Ctrl+C` from keyboard, since the figure save procedure is performed automatically after that input.

## Program start (faster)
To run the program automatically, just run the following command

```bash
# Shell A
python3 run.py
```

This will start all the nodes for the simulation: It will open 6 shells, the ones from A to F. For the plotting node then run it separately as [above](#plotting-node)

## Setup and start flow for the correct program execution
```bash
# Shell A
python3 run.py
```
Once done this, the program will output the following lines:
```bash
Simulation starting ...


Press to start the ROS bridge, the map merger node and the servers [Enter]: 
```
The program will start the Rviz simulation and the Gazebo model (run in background). <br>
It is suggested to wait for the simulation to correctly start before proceding.

```bash
# Shell B
rosservice call /gazebo_2Dmap_plugin/generate_map
```
This tool will map automatically the entire environment the robots are deployed into. This will produce a map published on a dedicated topic which will be used for the plotting part. <br>
The code of this tool can be found at the following [link](https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin). <br>
Wait until the tool has finished the mapping, it will end its activity and the shell it is running into becomes available again. <br>
Once done, return to the `Shell A` and press `Enter`. The shell will look like
```bash
# Shell A
python3 run.py
```
Once done this, the program will output the following lines:
```bash
Simulation starting ...


Press to start the ROS bridge, the map merger node and the servers [Enter]: 

ROS bridge starting ...

Map merge starting ...

Servers starting ...


Press Enter to start the controller nodes: [Enter]
```

The controller node is the last one to be run, so before that it is needed to start the plotting node (it is not mandatory for the execution of the program but it will output some graphs used as visualization)
```bash
# Shell C
rosrun plotting plot.py
```
Once the node is running, press `Enter` on the `Shell A` and the program will start correctly.

### Stopping the execution
To stop the execution use the `Shell A` which will close all the terminals opened by pressing `Enter` as described in the shell; for the plotting node it has to be manually stopped. This will produce an output `.png` which will be saved in the [`figures`](/src/plotting/) directory of the package (if it does not exist it will be created by the node). The [`config`](/src/plotting/config/number_figures.txt) file is automatically updated by the node.

### Requirements
Be sure to have `pyhthon3` command installed and `wmctrl` which can be got by the `sudo apt-get install wmctrl` command.