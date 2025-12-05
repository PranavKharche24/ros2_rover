node = ros2node("/matlab_imu");

imuSub = ros2subscriber(node, "/imu", "sensor_msgs/Imu",Reliability="besteffort", Durability="volatile");

figure;
tiledlayout(3, 1);
axX = nexttile; title(axX, "Gyro X (Normalized)"); ylabel(axX, "Angular Velocity (rad/s)"); grid(axX, "on");
axY = nexttile; title(axY, "Gyro Y (Normalized)"); ylabel(axY, "Angular Velocity (rad/s)"); grid(axY, "on");
axZ = nexttile; title(axZ, "Gyro Z (Normalized)"); ylabel(axZ, "Angular Velocity (rad/s)"); xlabel(axZ, "Time (s)"); grid(axZ, "on");

timeData = []; gxData = []; gyData = []; gzData = [];
normGxData = []; normGyData = []; normGzData = [];
biasGyro = [NaN NaN NaN];
biasAlpha = 0.02;
lastGyro = [NaN NaN NaN];
startTime = tic;

disp("Streaming IMU gyro data... Press Ctrl+C to stop.")

while true
    % Try receiving the message safely
    try
        imuMsg = receive(imuSub, 0.1);   % try for 0.1 sec

    catch
        % If no message arrives, skip this loop iteration
        disp("No IMU data...");
        pause(0.05);
        continue;
    end

    % Extract gyro values
    gx = imuMsg.angular_velocity.x;
    gy = imuMsg.angular_velocity.y;
    gz = imuMsg.angular_velocity.z;

    currentGyro = [gx, gy, gz];
    if all(abs(currentGyro - lastGyro) < 1e-9)
        disp("Duplicate IMU gyro sample detected, skipping plot update.");
        pause(0.05);
        continue;
    end
    if any(isnan(biasGyro))
        biasGyro = currentGyro;
    else
        biasGyro = (1 - biasAlpha) .* biasGyro + biasAlpha .* currentGyro;
    end
    normGyro = currentGyro - biasGyro;
    lastGyro = currentGyro;

    % Time
    t = toc(startTime);

    % Append data
    timeData(end+1) = t;
    gxData(end+1) = gx;
    gyData(end+1) = gy;
    gzData(end+1) = gz;
    normGxData(end+1) = normGyro(1);
    normGyData(end+1) = normGyro(2);
    normGzData(end+1) = normGyro(3);

    cla(axX); plot(axX, timeData, normGxData, 'r', 'LineWidth', 1.2);
    cla(axY); plot(axY, timeData, normGyData, 'g', 'LineWidth', 1.2);
    cla(axZ); plot(axZ, timeData, normGzData, 'b', 'LineWidth', 1.2);

    fprintf("Gyro raw [X Y Z] = [%.3f %.3f %.3f] | norm [%.3f %.3f %.3f]\n", gx, gy, gz, normGyro);
    drawnow limitrate;
end
