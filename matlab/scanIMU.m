%% Create ROS2 Node and IMU Subscriber (BEST_EFFORT QoS)
node = ros2node("/matlab_imu");

imuSub = ros2subscriber(node, "/imu", "sensor_msgs/Imu",Reliability="besteffort", Durability="volatile");

%% Setup figure for real-time plotting
figure;
title("IMU Gyroscope (Angular Velocity) - Live Plot");
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)");

hold on;
grid on;

degFactor = 180 / pi;          % rad/s -> deg/s
biasWindow = 200;              % samples for rolling bias
gxZeroData = [];
gyZeroData = [];
gzZeroData = [];

legend("Gyro X (deg/s zero-mean)", "Gyro Y (deg/s zero-mean)", "Gyro Z (deg/s zero-mean)");

% Initialize arrays for storing data
timeData = [];
gxData = [];
gyData = [];
gzData = [];

disp("Streaming IMU gyro data... Press Ctrl+C to stop.")

startTime = tic;

%% Continuous Loop
while true
    % Receive IMU message (non-blocking)
    imuMsg = receive(imuSub, 0.1);   % 0.1 sec timeout
    if isempty(imuMsg)
        drawnow;
        continue;
    end
    % Extract gyroscope data (convert to deg/s)
    gx = imuMsg.angular_velocity.x * degFactor;
    gy = imuMsg.angular_velocity.y * degFactor;
    gz = imuMsg.angular_velocity.z * degFactor;

    % Compute elapsed time
    t = toc(startTime);

    % Store values
    timeData(end+1) = t;
    gxData(end+1) = gx;
    gyData(end+1) = gy;
    gzData(end+1) = gz;

    idxStart = max(1, numel(gxData) - biasWindow + 1);
    biasX = mean(gxData(idxStart:end));
    biasY = mean(gyData(idxStart:end));
    biasZ = mean(gzData(idxStart:end));

    gxZeroData(end+1) = gx - biasX;
    gyZeroData(end+1) = gy - biasY;
    gzZeroData(end+1) = gz - biasZ;

    % Live plot (gyro Z or all three — choose your line)
    plot(timeData, gxZeroData, 'r', 'LineWidth', 1.5);
    plot(timeData, gyZeroData, 'g', 'LineWidth', 1.5);
    plot(timeData, gzZeroData, 'b', 'LineWidth', 1.5);

    drawnow;
end
