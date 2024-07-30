function averageRMSE = calculateAverageRMSE(dataWithErrors, trueIMUData, R)
    % 数据对齐基于最后一列的时间戳
    trueTimes = trueIMUData(:, end);  % 真实数据的时间戳

    % 初始化RMSE数组
    rmseValues = zeros(R, 6); % 6列，对应陀螺仪和加速度计的X、Y、Z轴

    % 遍历每一组数据，计算RMSE
    for i = 1:R
        % 提取第i组带误差的数据
        errorIMUData = dataWithErrors(:, 1:6, i);
        errorTimesI = dataWithErrors(:, end, i);  % 提取当前组的时间戳

        % 数据对齐
        alignedTrueIMUData = interp1(trueTimes, trueIMUData(:, 1:6), errorTimesI, 'linear', 'extrap');

        % 计算每个轴的RMSE并存储
        for j = 1:6
            squaredError = (errorIMUData(:, j) - alignedTrueIMUData(:, j)).^2;
            rmseValues(i, j) = sqrt(mean(squaredError));
        end
    end

    % 计算平均RMSE
    averageRMSE = mean(rmseValues, 1); % 对每个轴计算平均RMSE
end

