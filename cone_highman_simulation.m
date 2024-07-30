glvs
[nn, ts, nts] = nnts(4, 0.01);  % subsamples & sampling interval
afa = 60*glv.deg;              % half-apex angle
f = 2;  w = 2*pi*f;             % frequency
T = 10;                          % simulation time
[wm, qr] = conesimu(afa, f, ts, T);    % 仿真圆锥运动, wm为角增量（imu数据），qr为姿态四元数（真值）
% [wm, qr] = highmansimu(randn(3,5), ts, T, 1);
coef = wm2wtcoef(ts, nn);  % 根据nn个角增量采样可以构造一个N-1次的多项式角速度去拟合真实的角速度，迭代求解等效旋转矢量微分方程精确数值解，该项为转换矩阵系数
% 这里可以考虑加上初始的姿态误差
len = length(wm);
q1i = qr(1,:)'; q2i = q1i; q3i = q1i; q4i = q1i; q5i = q1i; ki = 1;    % 初始化姿态四元数
q1 = zeros(fix(len/nn), 4); q2 = q1; q3 = q1; q4 = q1; q5 = q1;
%% 利用角增量去解算姿态的变化
for k=1:nn:len-nn+1
    k1 = k+nn-1;
	wmi = wm(k:k1, :);	q0 = qr(k1+1,:)';
	phim = cnscl(wmi, 0);  q1i = qmul(q1i,rv2q(phim)); % optimal method
    q2i = qmul(q2i, rv2q(btzpicard(wmi'*coef, nts))); % accurate numerical solution with picard method
    q3i = qmul(q3i, m2qua(dcmtaylor(wmi'*coef, nts))); % accurate numerical solution with taylor method
    q4i = qmul(q4i, rv2q(btzrk4(wmi, nts)));  % Bortz Runge-Kutta
    q5i = qrk4(q5i, wmi, nts);  % quaternion Runge-Kutta
    q1(ki, :) = q1i'; q2(ki, :) = q2i'; q3(ki, :) = q3i'; q4(ki, :) = q4i'; q5(ki, :) = q5i'; ki = ki+1;
end

%% 将解算得到的姿态四元数反演推得拟角增量-mock gyro increment
N = size(q1, 1);
wm_mock = zeros(N-1, 3); % 初始化角增量矩阵
ki = 1;
wmi_mock = [0; 0; 0]';
for i = 2:N
    % 当前四元数
    q = q3(i, :)';
    q_pre = q3(i-1, :)';
    
    % 计算相对旋转四元数并求出等效旋转矢量
    dq = qmul(qinv(q_pre), q);
    phim = q2rv(dq);  
    % 由等效旋转矢量反解出角增量
    wmi_mock = (glv.I33+askew(wmi_mock/12))^-1*phim; % using previous subsample: phim = wm + 1/12*cross(wm_1,wm)
    wm_mock(i-1, :) = wmi_mock';
end
%% 参考角增量处理，聚合至拟角增量同维度
wm_true_aggregated = aggregate_real_angular_increment(wm, nn);
wm_mock = [wm_true_aggregated(1,:); wm_mock];
% 添加时间戳
t = (nts:nts:T)';
wm_mock = [wm_mock, t];
wm_true_aggregated = [wm_true_aggregated, t];

%% error caculate





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


