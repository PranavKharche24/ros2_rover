% Create ROS2 node
node = ros2node("/matlab_costmap_reader");

% Subscribe to the local costmap topic
costSub = ros2subscriber(node, "/local_costmap/costmap", ...
                         "nav_msgs/OccupancyGrid", ...
                         Reliability="reliable");

% Receive one costmap message
msg = receive(costSub, 1);   % waits up to 1 second

% Convert costmap data into 2D matrix
w = msg.info.width;
h = msg.info.height;

costmapData = reshape(msg.data, [w, h])';   % reshape + transpose

% Print the costmap matrix
disp(costmapData);
