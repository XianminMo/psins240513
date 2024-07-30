%% data loading
glvs
filename = 'data_0.csv';
dataArray = readmatrix(filename);
data = [dataArray(:, 7:9), dataArray(:, 4:6)];
timeStamps = (0:size(data,1)-1)' * 0.2;
data = [data, timeStamps];
imu = data;
ts = 0.2;    % imu采样频率
% imuplot(imu) ;   % imu数据（三轴角增量和三轴速度增量以及时间戳）
avp0 = [0; 0; 0; 0; 0; 0; 0; 0; 0];    % avp initial value
%% error setting
% eb = 0.1;
% db = 180;
% web = 0.03;
% wdb = 30;
% imuerr = imuerrset(eb, db, web, wdb);
% imu0 = imuadderr(imu, imuerr);
% % imuplot(imu0);
% davp0 = avperrset([0.5;0.5;5], 0.1, [10;10;10]);
% avp00 = avpadderr(avp0, davp0); 
%% inertial navigation
nn = 2;    % 子样数，每累积nn个imu数据更新一次姿态
nts = nn * ts;    % 姿态更新时间间隔
ins = insinit(avp0, ts); vn0 = avp0(4:6); pos0 = avp0(7:9);    % 初始化
%% reference height for altitude damping / height simulation
% var = 1; tau = 10; bias = 3;
% t = (avpr(1,10):ts:avpr(end,10))';
% bh = interp1(avpr(:,10), avpr(:,9), t, 'linear');
% bh = bh + bias + markov1(var, tau, ts, length(bh));
% href = [bh, t];
%%

len = length(imu); avp = zeros(fix(len/nn), 10);    % 计算imu数据总长度，并初始化avp数组
ki = timebar(nn, len, 'Pure inertial navigation processing.');    % 显示图

for k = 1 : nn : len-nn+1    % 每累积nn个imu数据更新一次姿态，即n子样算法
    k1 = k+nn-1;
    wvm = imu(k:k1, 1:6); t = imu(k1, end);    % 提出该时段内的imu中角增量和速度增量序列，有nn个值以及时间戳
    ins = insUpdateForOneLoop(ins, wvm, 0);    % 更新一次ins结构体，avp数据存于结构体中，即更新一次姿态，位置，速度，输入为上一时刻的avp信息以及该新输入的imu提供的校准角增量以及速度增量信息
    %%
    % ins.eth.dgnt=t;
    % ins.pos(3) = href(k1,1);    % 高度修正
    %%
    avp(ki,:) = [ins.avp; t]';    % 将这一时段内计算得到的avp存入avp数组，以及时间序列
    ki = timebar;
end
%% trajectory ploting
% plot_trajectory_from_position(avpr, avp);
insplot(avp);
% avperr = avpcmpplot(avpr, avp);
% inserrplot(avperr);