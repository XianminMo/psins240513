%% data loading
glvs
% 以下是一组无误差的真实飞行惯导数据的载入
% load flightimuavpr.mat;    % 一组无误差的真实飞行惯导数据
% ts = 0.005;    % imu采样频率
% insplot(avpr)    % 物体运动真值（采用INS/DGNSS组合平滑后，可作为参考真值，三轴姿态，速度，位置以及时间戳）
% imu = imu(200/ts:1800/ts,:);    % 取前半小时的飞行轨迹，去掉前200秒的静止
% imuplot(imu) ;   % imu数据（三轴角增量和三轴速度增量以及时间戳）
% avpr = avpr(200/0.01:1800/0.01,:);
% avp0 = avpr(1,:)';    % avp initial value

% % 以下是用组合导航设备采集的数据载入avpr
load data_GPSIMU\data.mat
ts = 0.1; % 时间间隔
imu = IMU_data;
imu(:, 1:3) = deg2rad(imu(:,1:3));

% 将角速度和比力转换为增量
imu(:, 1:3) = imu(:, 1:3) * ts; % 角增量，单位：rad
imu(:, 4:6) = imu(:, 4:6) * glv.g0 * ts; % 速度增量，单位：m/s
% imuplot(imu);

att_true = attitude_vector(:, [2, 3, 1]);
yaw = att_true(:, 3);
for i = 1 : length(yaw)
    if yaw(i) <= 180 && yaw(i) >= 0
        yaw(i) = -yaw(i);
    else
        yaw(i) = 360 - yaw(i);
    end
end % yaw: 北偏东为正 0-360 -> 北偏西为正 -180-180
att_true(:, 3) = yaw;
att_true = deg2rad(att_true(:, :)); % [pitch, roll, yaw] in rad(pi)

vel_true = velocity_vector(:, [1, 2, 3]); % [E, N, U] in m/s, 去掉最后一列时间戳

pos_true = position_vector(:, [1, 2, 3, 4]); 
pos_true(:, 1:2) = pos_true(:, 1:2) * glv.deg; % [latitude, longitude, altitude] in rad, rad, m/s

avpr = [att_true, vel_true, pos_true];
avp0 = avpr(1,:)';

% href = position_vector(:, end-1:end);

% 以下是用小导航设备采集的数据载入avpr
load ./data_smallIMU/data.mat
ts = 0.1;
imu(:, 1:6) = imu(:, 1:6) * ts; % 处理为增量形式
href = position_vector(:, end-1:end);
% avp0 = [-0.00541052068118242, 0.0171042266695444, -2.56790292845926, -0.0170000000000000, 0.00600000000000000, -0.0180000000000000, 0.597958943767879, 1.89634169258822, 0, 0];
%% geoplot
figure;
latitudes = position_vector(:,1);
longitudes = position_vector(:,2);
geoplot(latitudes, longitudes, '-o', 'LineWidth',2, 'MarkerSize', 2);
hold on;
% 标记起始点和终点
geoplot(latitudes(1), longitudes(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % 起始点
geoplot(latitudes(end), longitudes(end), 'gs', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % 终点
% 添加图形标题
title('Position Trajectory');
% 添加标注文本
text(longitudes(1), latitudes(1), ' Start', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', 'r', 'FontWeight', 'bold');
text(longitudes(end), latitudes(end), ' End', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'Color', 'g', 'FontWeight', 'bold');
% 添加网格
geobasemap('streets'); % 使用街道图层作为地图底图
hold off;
% plotTrajectory(position_vector);
%% error setting
% eb = 0.1;
% db = 180;
% web = 0.03;
% wdb = 30;
% imuerr = imuerrset(eb, db, web, wdb);
% imu0 = imuadderr(imu, imuerr);
% imuplot(imu0);
% davp0 = avperrset([0.5;0.5;5], 0.1, [10;10;10]);
% avp00 = avpadderr(avp0, davp0); 
eb = 0;
db = 0;
web = 0;
wdb = 0;
imuerr = imuerrset(eb, db, web, wdb);
imu0 = imuadderr(imu, imuerr);
% imuplot(imu0);
davp0 = avperrset([0;0;0], 0, [0;0;0]);
avp00 = avpadderr(avp0, davp0); 
%% inertial navigation
nn = 2;    % 子样数，每累积nn个imu数据更新一次姿态
nts = nn * ts;    % 姿态更新时间间隔
ins = insinit(avp00, ts); vn0 = avp00(4:6); pos0 = avp00(7:9);    % 初始化
%% reference height for altitude damping / height simulation
% var = 1; tau = 10; bias = 3;
% t = (avpr(1,10):ts:avpr(end,10))';
% bh = interp1(avpr(:,10), avpr(:,9), t, 'linear');
% bh = bh + bias + markov1(var, tau, ts, length(bh));
% href = [bh, t];
%%
len = length(imu0); avp = zeros(fix(len/nn), 10);    % 计算imu数据总长度，并初始化avp数组
ki = timebar(nn, len, 'Pure inertial navigation processing.');    % 显示图

for k = 1 : nn : len-nn+1    % 每累积nn个imu数据更新一次姿态，即n子样算法
    k1 = k+nn-1;
    wvm = imu0(k:k1, 1:6); t = imu0(k1, end);    % 提出该时段内的imu中角增量和速度增量序列，有nn个值以及时间戳
    ins = insUpdateForOneLoop(ins, wvm, 4);    % 更新一次ins结构体，avp数据存于结构体中，即更新一次姿态，位置，速度，输入为上一时刻的avp信息以及该新输入的imu提供的校准角增量以及速度增量信息
    %%
    ins.eth.dgnt = t;
    ins.avp(9) = href(k1,1);    % 高度修正
    %%
    avp(ki,:) = [ins.avp; t]';    % 将这一时段内计算得到的avp存入avp数组，以及时间序列
    ki = timebar;
end
%% trajectory ploting
plot_trajectory_from_position(avpr, avp, pos0);
% dxyz = pos2dxyz(avp(:,7:9));
% insplot(avp);
% avperr = avpcmpplot(avpr, avp);
% inserrplot(avperr);
% xyz1 = pos2dplot(avp(:, 7:10));
%% compare and caculate error using ground truth
[rmse_attitude, rmse_velocity, rmse_position, aee_attitude, aee_velocity, aee_position] = computeErrors(avpr, avp);
%% compare and calulate error
R = 10; % 实验数据添加噪声次数
numSimulations = 100;

imu_mock = avp2imu(avp);

% 实验数据添加噪声 Z->{Z}
imu_mock_set = generateIMUDataWithErrors(imu_mock, R, eb, db, web, wdb);

% 初始化存储每次模拟的RMSE和AEE结果
allRMSEs = zeros(numSimulations, 6);
allAEEs = zeros(numSimulations, 6);

% Monte-Carlo method
for i = 1:numSimulations
    % 给“真实”IMU数据添加噪声
    imuWithErr = generateIMUDataWithErrors(imu, 1, eb, db, web, wdb); % R=1, 相当于添加一个噪声，不形成一组
     
    % 计算实验数据和“新的真实数据”之间的RMSE和AEE
    simulatedRMSE = calculateAverageRMSE(imu_mock_set, imuWithErr, R);
    simulatedAEE = calculateAverageAEE(imu_mock_set, imuWithErr, R);
    
    % 存储每次模拟的结果
    allRMSEs(i, :) = simulatedRMSE;
    allAEEs(i, :) = simulatedAEE;
end

% 计算所有模拟的平均RMSE和AEE
averageRMSE = mean(allRMSEs, 1);
averageAEE = mean(allAEEs, 1);

% 计算陀螺仪和加速度计三轴平均
% 陀螺仪三轴的RMSE和AEE平均值
gyroRMSE_Average = mean(averageRMSE(1:3));
gyroAEE_Average = mean(averageAEE(1:3));

% 加速度计三轴的RMSE和AEE平均值
accelRMSE_Average = mean(averageRMSE(4:6));
accelAEE_Average = mean(averageAEE(4:6));

% 将性能评估值组合成一对
performanceEvaluation = [gyroRMSE_Average, accelRMSE_Average; gyroAEE_Average, accelAEE_Average];

% 输出性能评估数组
disp('性能评估数组（第一行为RMSE，第二行为AEE，第一列为陀螺仪，第二列为加速度计）：');
disp(performanceEvaluation);







    