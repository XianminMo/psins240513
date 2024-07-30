function mock_imu = compute_mock_measurement(avp)
    % Inputs：包含姿态、速度、位置和时间戳的矩阵
    % Outputs：拟IMU测量，即反解加速度和角速度
    % avp是一个Nx10矩阵:
    % avp = [att,vn,pos,t] xyz三轴

    % 初始化输出参数
    N = size(avp, 1);
    % 差分会导致少一行数据点
    accelerations_b = zeros(N-1, 3); % b系下的加速度
    angular_velocities = zeros(N-1, 3); % 角速度
    timestamps = avp(2:end, 10); % 时间戳，排除第一个
  
    
    % 遍历数据，计算加速度和角速度
    for i = 2:N
        % 时间间隔
        deltaTime = avp(i, 10) - avp(i-1, 10);
        
        % 估算地球模型参数
        pos_mid = (avp(i-1, 7:9) + avp(i, 7:9)) / 2; % 取中间点位置计算地球参数
        eth_mid = earth(pos_mid, (avp(i, 4:6) + avp(i-1, 4:6)) / 2); % 使用中间点的速度和位置
        
        % 使用a2mat函数获取相邻时间点的DCM
        DCM_prev = a2mat(avp(i-1, 1:3)); % Cnb,即n系到b系的旋转矩阵或者b系到n系的坐标变换矩阵
        DCM_curr = a2mat(avp(i, 1:3));

        % 位置差分得到速度
        deltaP = (avp(i, 7:9) - avp(i-1, 7:9)) / deltaTime;
        % 速度差分得到导航坐标系下的加速度
        acceleration_n = (2 / deltaTime) * (deltaP - avp(i-1, 4:6));
        % 将加速度从导航坐标系转换到机体坐标系
        accelerations_b(i-1, :) = (DCM_curr' * acceleration_n')' - (DCM_curr' * eth_mid.gcc)';

        % 计算姿态变化矩阵并提取角速度分量
        delta_dcm = DCM_prev' * DCM_curr - eye(3); 
        angular_velocities(i-1, 1) = delta_dcm(2,1) / deltaTime; % ωx
        angular_velocities(i-1, 2) = delta_dcm(1,3) / deltaTime; % ωy
        angular_velocities(i-1, 3) = delta_dcm(3,2) / deltaTime; % ωz
    end
    
    mock_imu = [angular_velocities accelerations_b timestamps];
end
