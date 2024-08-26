filename = '3.gdf';

% 打开文件
fileID = fopen(filename, 'r');

% 初始化存储变量
IMU_data = [];
attitude_vector = [];
velocity_vector = [];
position_vector = [];
time_stamp = [];

% 读取数据
prev_gps_week = -1;
prev_gps_seconds = -1;
gps_data_valid = false;

while ~feof(fileID)
    line = fgetl(fileID);
    
    if startsWith(line, '$GPFPD')
        % 解析GPS数据行
        line_cleaned = regexprep(line, '^\$GPFPD,|,\d*\*.*$', '');
        
        % 使用 strsplit 手动分割字符串
        data_cells = strsplit(line_cleaned, ',');

        % 将单元格数组中的字符串转换为数值
        data1 = cellfun(@str2double, data_cells);

        gps_week = data1(1);
        gps_seconds = data1(2);
        attitude = [data1(3), data1(4), data1(5)]; % 方位角yaw、俯仰角pitch、横滚角roll 姿态
        velocity = [data1(9), data1(10), data1(11)]; % 东、北、天 速度
        position = [data1(6), data1(7), data1(8)];% 经度、纬度、高度
              
        % 更新上一帧的 GPS 数据
        prev_gps_week = gps_week;
        prev_gps_seconds = gps_seconds;
        gps_data_valid = true;
        
    elseif startsWith(line, '$GTIMU') && gps_data_valid
        % 解析IMU数据行
        line_cleaned = regexprep(line, '^\$GTIMU,|,\d*\*.*$', '');
        
        % 使用 strsplit 手动分割字符串
        data_cells = strsplit(line_cleaned, ',');

        % 将单元格数组中的字符串转换为数值
        data2 = cellfun(@str2double, data_cells);

        imu_week = data2(1);
        imu_seconds = data2(2);
        
        % 判断IMU行和前一行的GPS行是否属于同一帧
        if imu_week == prev_gps_week && imu_seconds == prev_gps_seconds
            gyro = [data2(3), data2(4), data2(5)]; % 三轴陀螺仪输出
            accel = [data2(6), data2(7), data2(8)]; % 三轴加速度计输出
            
            % 计算时间戳
            if isempty(time_stamp)
                start_time = imu_seconds; % 记录开始时间
            end
            timestamp = imu_seconds - start_time;
            
            % 累加数据
            IMU_data = [IMU_data; gyro, accel, timestamp];
            attitude_vector = [attitude_vector; attitude, timestamp];
            velocity_vector = [velocity_vector; velocity, timestamp];
            position_vector = [position_vector; position, timestamp];
            time_stamp = [time_stamp; timestamp];
        end
        
        % 重置GPS数据有效性
        gps_data_valid = false;
    end
end

% 关闭文件
fclose(fileID);

% 保存IMU数据
save('./data_GPSIMU/data.mat', 'IMU_data', 'attitude_vector', 'velocity_vector', 'position_vector');
IMU_table = array2table(IMU_data, 'VariableNames', {'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Timestamp'});
attitude_table = array2table(attitude_vector, 'VariableNames', {'Yaw', 'Pitch', 'Raw', 'Timestamp'});
velocity_table = array2table(velocity_vector, 'VariableNames', {'Velocity_E', 'Velocity_N', 'Velocity_U', 'Timestamp'});
position_table = array2table(position_vector, 'VariableNames', {'Longitude', 'Latitude', 'Altitude', 'Timestamp'});

% 将 IMU 数据表格保存为 IMU_data.csv
writetable(IMU_table, './data_GPSIMU/IMU_data.csv');

% 将姿态数据表格保存为 velocity_data.csv
writetable(attitude_table, './data_GPSIMU/attitude_data.csv');

% 将速度数据表格保存为 velocity_data.csv
writetable(velocity_table, './data_GPSIMU/velocity_data.csv');

% 将位置数据表格保存为 position_data.csv
writetable(position_table, './data_GPSIMU/position_data.csv');        

% plot(position_table.Timestamp, position_table.Longitude);
% % 输出结果
% disp('IMU Data:');
% disp(IMU_table);
% disp('Velocity Vector:');
% disp(velocity_table);
% disp('Position Data:');
% disp(position_vector);