function plotTrajectory(data)
    % 输入:
    % data - 一个 N x 4 的数组，包含纬度、经度、高度和时间戳
    % data(:,1) - 纬度 (degrees)
    % data(:,2) - 经度 (degrees)
    % data(:,3) - 高度 (meters)
    % data(:,4) - 时间戳 (seconds)

    % 检查输入数据的有效性
    if size(data, 2) ~= 4
        error('输入数据必须是一个 N x 4 的数组');
    end

    % 提取数据
    latitudes = data(:, 1);
    longitudes = data(:, 2);

    % 初始位置
    initial_lat = latitudes(1);
    initial_lon = longitudes(1);

    % 转换经纬度为地理距离（单位：米），假设球面地球
    % 地球半径
    R = 6371000; % 米

    % 计算纬度和经度的变化
    lat_diff = deg2rad(latitudes - initial_lat);
    lon_diff = deg2rad(longitudes - initial_lon);

    % 纬度和经度的差异
    delta_lat = lat_diff;
    delta_lon = lon_diff;

    % 计算距离
    % 计算地球表面两点之间的距离（即平面距离）
    x = R * delta_lon .* cos(deg2rad(initial_lat)); % 东向距离
    y = R * delta_lat; % 北向距离

    % 绘制二维轨迹图
    figure;
    plot(x, y, '-o', 'LineWidth', 2);
    xlabel('East (m)');
    ylabel('North (m)');
    title('2D Trajectory View (Top View)');
    grid on;
end
