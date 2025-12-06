node = ros2node("/matlab_lidar");

lidarSub = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan",Reliability="besteffort", Durability="volatile");

figure("Name", "Live LIDAR Point Cloud", "NumberTitle", "off");
ax = axes; axis(ax, "equal");
xlabel(ax, "X (m)"); ylabel(ax, "Y (m)");
title(ax, "2D LIDAR Scan in Cartesian Coordinates");
grid(ax, "on");

plotHandle = plot(ax, 0, 0, ".", "MarkerSize", 6);
xlim(ax, [-10 10]); ylim(ax, [-10 10]);

warnThrottle = 1.0;
loopClock = tic;
lastWarnTime = -inf;
noDataThrottle = 2.0;
lastNoDataWarn = -inf;
lastStampSec = -1;
lastStampNano = -1;

disp("Streaming LIDAR data... Press Ctrl+C to stop.")

while ishandle(ax)
    try
        scanMsg = receive(lidarSub, 0.2);
    catch
        elapsed = toc(loopClock);
        if elapsed - lastNoDataWarn >= noDataThrottle
            disp("No LIDAR data...");
            lastNoDataWarn = elapsed;
        end
        pause(0.05);
        continue;
    end

    stampSec = double(scanMsg.header.stamp.sec);
    stampNano = double(scanMsg.header.stamp.nanosec);
    if stampSec == lastStampSec && stampNano == lastStampNano
        continue;
    end
    lastStampSec = stampSec;
    lastStampNano = stampNano;

    ranges = double(scanMsg.ranges);
    angles = scanMsg.angle_min + (0:numel(ranges)-1).' * scanMsg.angle_increment;

    invalidMask = ~isfinite(ranges) | (ranges <= scanMsg.range_min);
    invalidCount = sum(invalidMask);
    if isfinite(scanMsg.range_max)
        ranges(invalidMask) = max(scanMsg.range_min + 1e-3, scanMsg.range_max - 1e-3);
    else
        ranges(invalidMask) = scanMsg.range_min + 1.0;
    end

    validIdx = (ranges > scanMsg.range_min) & (~isfinite(scanMsg.range_max) | (ranges <= scanMsg.range_max));
    if ~any(validIdx)
        elapsed = toc(loopClock);
        if elapsed - lastWarnTime >= warnThrottle
            disp("All LIDAR ranges invalid, skipping frame.");
            lastWarnTime = elapsed;
        end
        pause(0.05);
        continue;
    end

    ranges = ranges(validIdx);
    angles = angles(validIdx);

    x = ranges .* cos(angles);
    y = ranges .* sin(angles);

    sampleCount = numel(x);
    sampleIdx = min(5, sampleCount);
    minRange = min(ranges);
    maxRange = max(ranges);
    fprintf("Scan %u.%09u | points: %d | invalid replaced: %d | range min/max (m): %.2f / %.2f | sample XY (m): [%.2f %.2f]\n", ...
        stampSec, stampNano, sampleCount, invalidCount, minRange, maxRange, x(sampleIdx), y(sampleIdx));

    set(plotHandle, "XData", x, "YData", y);
    drawnow limitrate;
end
