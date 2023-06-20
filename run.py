import os
import time
import subprocess
import signal

# Set the paths to the ROS1 and ROS2 installations
ros1_path_setup_distro = "/opt/ros/noetic"
ros1_path_setup_ws = "/home/usr/data/catkin_ws/devel"
ros2_path_setup_distro = "/opt/ros/foxy"
ros2_path_setup_ws = "/home/usr/data/ros2_ws/install"

# Define ROS1 commands
ros1_roscore_cmd = "roscore"
ros1_simulation_cmd = "roslaunch maslam_rosbot file.launch"
ros1_karto_cmd = "roslaunch maslam_rosbot multiple.launch"
ros1_server_plugin = "rosservice call /gazebo_2Dmap_plugin/generate_map"
ros1_centralised = "roslaunch maslam_rosbot servers.launch"

# Define ROS2 commands
ros2_bridge_cmd = "ros2 run ros1_bridge dynamic_bridge"
ros2_map_merge_cmd = "ros2 launch merge_map merge_map_launch.py num_robots:=2"

# Function to run each terminal
def launch_terminal(cmd, title=None, ros1=True):
    title_option = f'--title="{title}"' if title else ''
    if ros1:
        setup_distro_path = ros1_path_setup_distro
        setup_ws_path = ros1_path_setup_ws
    else:
        setup_distro_path = ros2_path_setup_distro
        setup_ws_path = ros2_path_setup_ws

    os.system(
        f"gnome-terminal {title_option} --window -- zsh -c 'source {setup_distro_path}/setup.zsh; source {setup_ws_path}/setup.zsh; {cmd}; exec zsh'")


def main():
    # Launch terminals and store the process objects
    launch_terminal(ros1_roscore_cmd, "ROSCORE", ros1=True)
    time.sleep(2)

    launch_terminal(ros1_simulation_cmd, "ROS1 SIMULATION", ros1=True)
    print("Simulation starting ...")
    
    # Wait for user input to kill all processes
    print("")
    print("")
    input("Press to start the ROS bridge, the map merger node and the servers [Enter]: ")

    launch_terminal(ros2_bridge_cmd, "ROS2 BRIDGE", ros1=False)
    print("")
    print("ROS bridge starting ...")

    launch_terminal(ros1_server_plugin, "ROS2 BRIDGE", ros1=True)
    print("")
    print("ROS plugin for Gazebo working ...")

    launch_terminal(ros2_map_merge_cmd, "ROS2 MAP MERGE", ros1=False)
    print("")
    print("Map merge starting ...")
    launch_terminal(ros1_centralised, "ROS1 CENTRALISED SERVER", ros1=True)
    print("")
    print("Servers starting ...")
    
    print("")
    print("")
    # Wait for user input to kill all processes
    input("Start the controller nodes [Enter]: ")

    launch_terminal(ros1_karto_cmd, "ROS1 CONTROLLER", ros1=True)
    print("Servers starting ...")

    print("")
    print("")
    # Wait for user input to kill all processes
    input("Kill all processes [Enter]: ")

    # Close all terminals
    # Kill all processes containing "gnome-terminal" in the name
    os.system("pkill -f gnome-terminal")

if __name__ == "__main__":
    main()
