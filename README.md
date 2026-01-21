# dynamics_control

Quick instructions

1) Quick run (no ROS package install)
   - Ensure dependencies are installed (rclpy, numpy, matplotlib, scipy)
   - Run directly:
     python3 dynamics_control.py

   Note: this runs the node as a standalone Python process. It will try to connect to CoppeliaSim via simROS2 if available.

2) Recommended: build as ROS2 Python package and run via ros2
   - From workspace root (this folder), create a colcon workspace:
     mkdir -p ~/colcon_ws/src
     cp -r /home/anasalsalool/ros_final ~/colcon_ws/src/dynamics_control
   - Build:
     cd ~/colcon_ws
     colcon build --packages-select dynamics_control
   - Source:
     source install/setup.bash
   - Run node:
     ros2 run dynamics_control unified_dynamics_node
   - Or run the launch:
     ros2 launch dynamics_control launch_dynamics.py

     # Check if executable exists
    find install -name "*unified_dynamics*" -type f

    # Run directly
    ./install/dynamics_control/bin/unified_dynamics_node


3) CoppeliaSim (simROS2)
   - Start CoppeliaSim with simROS2 plugin enabled and load the Lua script `child_script.lua`.
   - Ensure ROS_DOMAIN_ID matches between ROS2 and CoppeliaSim.

Dependencies:
- ROS 2 (foxy/galactic/rolling)
- Python packages: numpy, scipy, matplotlib, rclpy
