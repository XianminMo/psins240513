clear;
glvs
[nn, ts, nts] = nnts(4, 0.01);  % subsamples & sampling interval
afa = 40*glv.deg;              % half-apex angle
f = 2;  w = 2*pi*f;             % frequency
T = 10;                          % simulation time
% [wm, qr] = conesimu(afa, f, ts, T);    % 仿真圆锥运动, wm为角增量（imu数据），qr为姿态四元数（真值）
pcoef = [ -7.8125e-01     6.25e-01     2.8125e-01     2.5       0
              -3.1250e-02    -5.50e-01     4.5000e-01    -5.0e-01   0
               1.8750e-01    -6.75e-01     2.25          -2.55      0 ];
[wm, qr] = highmansimu(pcoef, ts, T, 1);
coef = wm2wtcoef(ts, nn);  % 根据nn个角增量采样可以构造一个N-1次的多项式角速度去拟合真实的角速度，迭代求解等效旋转矢量微分方程精确数值解，该项为转换矩阵系数
% 这里可以考虑加上初始的姿态误差
len = length(wm);
q1i = qr(1,:)'; q2i = q1i;  % 初始化姿态四元数
% q3i = q1i; q4i = q1i; q5i = q1i;
ki = 1;   
q1 = zeros(fix(len/nn), 4); q2 = q1; 
% q3 = q1; q4 = q1; q5 = q1;
%% 利用角增量去解算姿态的变化
for k=1:nn:len-nn+1
    k1 = k+nn-1;
	wmi = wm(k:k1, :);	q0 = qr(k1+1,:)';
	phim = cnscl(wmi, 0);  q1i = qmul(q1i,rv2q(phim)); % optimal method
    q2i = qmul(q2i, rv2q(btzpicard(wmi'*coef, nts))); % accurate numerical solution with picard method
    % q3i = qmul(q3i, m2qua(dcmtaylor(wmi'*coef, nts))); % accurate numerical solution with taylor method
    % q4i = qmul(q4i, rv2q(btzrk4(wmi, nts)));  % Bortz Runge-Kutta
    % q5i = qrk4(q5i, wmi, nts);  % quaternion Runge-Kutta
    q1(ki, :) = q1i'; q2(ki, :) = q2i'; 
    % q3(ki, :) = q3i'; q4(ki, :) = q4i'; q5(ki, :) = q5i'; 
    ki = ki+1;
end

%% 将解算得到的姿态四元数反演推得拟角增量-mock gyro increment
wm1_mock = invertToMockAngularIncrement(q1);
wm2_mock = invertToMockAngularIncrement(q2);
% wm3_mock = invertToMockAngularIncrement(q3);
% wm4_mock = invertToMockAngularIncrement(q4);
% wm5_mock = invertToMockAngularIncrement(q5);

%% 参考角增量处理，聚合至拟角增量同维度
wm_true_aggregated = aggregate_real_angular_increment(wm, nn); % nn为降采样频率
wm1_mock = angularIncrementProcess(wm1_mock, wm_true_aggregated, nn);
wm2_mock = angularIncrementProcess(wm2_mock, wm_true_aggregated, nn);
% wm3_mock = angularIncrementProcess(wm3_mock, wm_true_aggregated, nn);
% wm4_mock = angularIncrementProcess(wm4_mock, wm_true_aggregated, nn);
% wm5_mock = angularIncrementProcess(wm5_mock, wm_true_aggregated, nn);

%% error caculate and plot
[RMSE1, AEE1] = computeError(wm1_mock, wm_true_aggregated);
[RMSE2, AEE2] = computeError(wm2_mock, wm_true_aggregated);
% [RMSE3, AEE3] = computeError(wm3_mock, wm_true_aggregated);
% [RMSE4, AEE4] = computeError(wm4_mock, wm_true_aggregated);
% [RMSE5, AEE5] = computeError(wm5_mock, wm_true_aggregated);

algorithms = {'coning method', 'picard method'};
RMSE = [RMSE1, RMSE2]; 
AEE = [AEE1, AEE2]; 

figure;

% 设置颜色
colors = lines(2);

% 绘制RMSE对比图
b1 = bar(RMSE, 'FaceColor', 'flat');
b1.CData = colors; % 应用颜色
title('Comparison of RMSE for Different Algorithms', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Algorithms', 'FontSize', 12);
ylabel('RMSE', 'FontSize', 12);
set(gca, 'XTickLabel', algorithms, 'FontSize', 10);
grid on;
legend({'RMSE'}, 'Location', 'northeast');

% 自动调整y轴范围
margin = 0.1; % 边距比例
ymin = min(RMSE) - margin * range(RMSE);
ymax = max(RMSE) + margin * range(RMSE);
ylim([ymin, ymax]);

% 在柱状图上显示数值
for i = 1:length(RMSE)
    text(i, RMSE(i), sprintf('%.5e', RMSE(i)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8, 'Color', 'k');
end

% 调整图形窗口大小和布局
set(gcf, 'Position', [100, 100, 600, 400]); % 调整窗口大小

figure;

% 绘制AEE对比图
b2 = bar(AEE, 'FaceColor', 'flat');
b2.CData = colors; % 应用颜色
title('Comparison of AEE for Different Algorithms', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Algorithms', 'FontSize', 12);
ylabel('AEE', 'FontSize', 12);
set(gca, 'XTickLabel', algorithms, 'FontSize', 10);
grid on;
legend({'AEE'}, 'Location', 'northeast');

% 自动调整y轴范围
ymin = min(AEE) - margin * range(AEE);
ymax = max(AEE) + margin * range(AEE);
ylim([ymin, ymax]);

% 在柱状图上显示数值
for i = 1:length(AEE)
    text(i, AEE(i), sprintf('%.5e', AEE(i)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8, 'Color', 'k');
end

% 调整图形窗口大小和布局
set(gcf, 'Position', [800, 100, 600, 400]); % 调整窗口大小

%% helper method
function wm_aggregated = aggregate_real_angular_increment(wm_true, factor)
    % 输入: wm_true - 真实角增量序列，大小为 [original_length, 3]
    %      factor - 聚合因子，例如4
    % 输出: wm_aggregated - 聚合后的角增量序列，大小为 [N, 3]

    % 获取真实角增量序列长度
    original_length = size(wm_true, 1);
    
    % 计算聚合后的序列长度，忽略最后不完整的一组
    N = floor(original_length / factor);
    
    % 初始化聚合后的角增量序列
    wm_aggregated = zeros(N, 3);
    
    % 将真实角增量每四行加起来
    for i = 1:N
        start_idx = (i - 1) * factor + 1;
        end_idx = i * factor;
        wm_aggregated(i, :) = sum(wm_true(start_idx:end_idx, :), 1);
    end
end

function [RMSE, AEE] = computeError(wm_mock, wm_true)
    % wm_mock:[N, 4]，最后一列为时间戳，需要去掉
    RMSE = sqrt(mean((wm_mock(:,1:3) - wm_true(:,1:3)).^2, 'all'));
    AEE = mean(abs(wm_mock(:,1:3) - wm_true(:,1:3)), 'all');
end

function wm_mock = invertToMockAngularIncrement(q)
    N = size(q, 1);
    wm_mock = zeros(N-1, 3); % 初始化角增量矩阵
    wmi_mock = [0; 0; 0]';
    for i = 2:N
        % 当前四元数
        q1 = q(i, :)';
        q0 = q(i-1, :)';
        
        % 计算相对旋转四元数并求出等效旋转矢量
        dq = qmul(qinv(q0), q1);
        phim = q2rv(dq);  
    
        % 由等效旋转矢量反解出角增量
        wmi_mock = (eye(3)+askew(wmi_mock/12))^-1*phim; % using previous subsample: phim = wm + 1/12*cross(wm_1,wm)
        wm_mock(i-1, :) = wmi_mock';
    end
end

function wm_mock = angularIncrementProcess(wm_mock, wm_true_aggregated, nn)
    if nn ~= 3 || nn ~= 5 % 数据对齐
        wm_mock = [wm_true_aggregated(1,:); wm_mock];
    end
end

