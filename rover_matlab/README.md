# MATLAB Utilities for ros2_rover

## Prerequisites
- MATLAB with ROS Toolbox installed.
- ROS 2 environment sourced (e.g., `source /opt/ros/humble/setup.bash` and `source ~/ros2_ws/install/setup.bash`).
- MATLAB configured to use the same ROS 2 domain ID as the rover stack.

## Setup
1. Start the ROS 2 stack that publishes the required topics (`/local_costmap/costmap`).
2. Launch MATLAB and run:
   ```matlab
   
   ros2('domainID', <domain_id>);
   ros2('workspace', '/home/pranav/ros2_ws');
   ```
3. Ensure the `/matlab_costmap_reader` node name is available on the network.
4. Run `setup.m` in MATLAB to automatically set the ROS domain ID and register helper nodes.

## Scripts
- `setup.m`: Configures MATLAB to use the rover's ROS 2 domain, instantiates helper nodes, and prints node/topic listings for quick diagnostics.
- `costmap.m`: Subscribes to `/local_costmap/costmap`, reshapes incoming occupancy data into a 2D matrix, and prints the grid for analysis.
- `scanIMU.m`: Subscribes to `/imu` using BEST_EFFORT QoS, maintains a rolling gyro bias estimate, and live-plots zero-mean angular velocity in degrees per second.

## Usage Notes
- Run `setup.m` once per MATLAB session (or after changing ROS_DOMAIN_ID) before executing the other scripts.
- Use `costmap.m` for snapshots of the latest costmap data; rerun as needed to refresh the output.
- Stop `scanIMU.m` with `Ctrl+C` after the desired capture duration.
