import os
import time
import subprocess
import signal

# Set the paths to the ROS1 and ROS2 installations
ros1_path_setup_distro = "/opt/ros/noetic"
ros1_path_setup_ws = "/home/usr/data/catkin_ws/devel"

# Define ROS1 commands
ros1_roscore_cmd = "roscore"
ros1_simulation_cmd = "roslaunch aslam_rosbot house.launch  "
ros1_karto_cmd = "roslaunch aslam_rosbot graph_dopt.launch    "
d_opti_plot = "rosrun aslam_rosbot d_opti_plot.py"


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

    launch_terminal(ros1_simulation_cmd, "RVIZ simulation", ros1=True)
    print("RVIZ Starting and Karto SLAM...")
    
    # Wait for user input to kill all processes
    print("")
    print("")
    time.sleep(5)   
    launch_terminal(ros1_karto_cmd, "ROS1 CONTROLLER", ros1=True)
    print("Graph d_optimality Starting...")
    print("")
    print("")
    time.sleep(5)
    launch_terminal(d_opti_plot, "d_opti_plot", ros1=True)
    print("Plot Started ...")
    print("")
    print("")   
        
    # Wait for user input to kill all processes
    input("Kill all processes [Enter]: ")

    # Close all terminals
    # Kill all processes containing "gnome-terminal" in the name
    os.system("pkill -f gnome-terminal")

if __name__ == "__main__":
    main()
