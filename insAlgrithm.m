%% data loading
glvs
load flightimuavpr.mat;
ts = 0.005;    % imu采样频率
% insplot(avpr)    % 物体运动真值（采用INS/DGNSS组合平滑后，可作为参考真值，三轴姿态，速度，位置以及时间戳）
% imuplot(imu) ;   % imu数据（三轴角增量和三轴速度增量以及时间戳）
imu = imu(1:1800/ts,:);    % 取前半小时的飞行轨迹
avp0 = avpr(1,:)';
%% error setting
eb = 0.1;
db = 180;
web = 0.03;
wdb = 30;
imuerr = imuerrset(eb, db, web, wdb);
imu0 = imuadderr(imu, imuerr);
% imuplot(imu0);
davp0 = avperrset([0.5;0.5;5], 0.1, [10;10;10]);
avp00 = avpadderr(avp0, davp0); 
%% inertial navigation
nn = 4;    % 子样数，每累积nn个imu数据更新一次姿态
nts = nn * ts;    % 姿态更新时间间隔
ins = insinit(avp00, ts); vn0 = avp00(4:6); pos0 = avp00(7:9);    % 初始化

len = length(imu0); avp = zeros(fix(len/nn), 10);    % 计算imu数据总长度，并初始化avp数组
ki = timebar(nn, len, 'Pure inertial navigation processing.');    % 显示图

for k = 1 : nn : len-nn+1    % 每累积nn个imu数据更新一次姿态，即n子样算法
    k1 = k+nn-1;
    wvm = imu0(k:k1, 1:6); t = imu0(k1, end);    % 提出该时段内的imu中角增量和速度增量序列，有nn个值以及时间戳
    ins = insUpdateForOneLoop(ins, wvm, 0);    % 更新一次ins结构体，avp数据存于结构体中，即更新一次姿态，位置，速度，输入为上一时刻的avp信息以及该新输入的imu提供的校准角增量以及速度增量信息
    avp(ki,:) = [ins.avp; t]';    % 将这一时段内计算得到的avp存入avp数组，以及时间序列
    ki = timebar;
end






    