function [rmse_attitude, rmse_velocity, rmse_position, aee_attitude, aee_velocity, aee_position] = computeErrors(true_data, exp_data)
    % 时间戳
    time_true = true_data(:, end);   % 真实数据时间戳
    time_exp = exp_data(:, end);     % 实验数据时间戳
    
    % 整体插值
    interpolated_data = interp1(time_exp, exp_data(:, 1:9), time_true, 'linear', 'extrap');
    
    % 计算每个部分的RMSE
    rmse_attitude = sqrt(mean((true_data(:, 1:3) - interpolated_data(:, 1:3)).^2, 'all'));
    rmse_velocity = sqrt(mean((true_data(:, 4:6) - interpolated_data(:, 4:6)).^2, 'all'));
    rmse_position = sqrt(mean((true_data(:, 7:9) - interpolated_data(:, 7:9)).^2, 'all'));
    
    % 计算每个部分的AEE
    aee_attitude = mean(abs(true_data(:, 1:3) - interpolated_data(:, 1:3)), 'all');
    aee_velocity = mean(abs(true_data(:, 4:6) - interpolated_data(:, 4:6)), 'all');
    aee_position = mean(abs(true_data(:, 7:9) - interpolated_data(:, 7:9)), 'all');
end

