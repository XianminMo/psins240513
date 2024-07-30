function averageAEE = calculateAverageAEE(dataWithErrors, trueIMUData, R)
    % 数据对齐基于最后一列的时间戳
    trueTimes = trueIMUData(:, end);  % 真实数据的时间戳
    aeeValues = zeros(R, 6); % 6列，对应陀螺仪和加速度计的X、Y、Z轴

    % 遍历每一组数据，计算AEE
    for i = 1:R
        % 提取第i组带误差的数据
        errorIMUData = dataWithErrors(:, 1:6, i);
        errorTimes = dataWithErrors(:, end, i);  % 提取当前组的时间戳

        % 数据对齐
        alignedTrueIMUData = interp1(trueTimes, trueIMUData(:, 1:6), errorTimes, 'linear', 'extrap');

        % 计算每个轴的AEE并存储
        for j = 1:6
            absoluteError = abs(errorIMUData(:, j) - alignedTrueIMUData(:, j));
            aeeValues(i, j) = mean(absoluteError);
        end
    end

    % 计算平均AEE
    averageAEE = mean(aeeValues, 1); % 对每个轴计算平均AEE
end
