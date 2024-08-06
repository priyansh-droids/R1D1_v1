# R1D1_v1
usage:
First install ros2 control for your ros version from [here](https://control.ros.org/rolling/doc/getting_started/getting_started.html) <br />
Remember to use the same workspace in which this package is placed for building ros2 control. <br />
Also install twist mux for your ROS distro : `sudo apt install ros-$ROS_DISTRO-twist-mux` <br />
colcon build everything, <br />
follow the steps: <br />
1. start Gazebo <br />
   `ros2 launch r1d1_description gazebo.launch.py` <br />
2. start Rviz <br />
   `ros2 launch r1d1_description display.launch.py` <br />
3. start slam toolbox <br />
   `ros2 launch slam_toolbox online_async_launch.py params_file:=./src/r1d1_description/config/mapper_params_online_async.yaml use_sim_time:=true` <br />
4. start nav2 <br />
   `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true` <br />
5. start twist mux <br />
   `ros2 run twist_mux twist_mux --ros-args --params-file ./src/r1d1_description/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped` <br />
6. start teleop twist keyboard with this <br />
   `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_kboard` <br />

you can add the '/map' topic to visualize the generated map. 2D goal pose also works.
