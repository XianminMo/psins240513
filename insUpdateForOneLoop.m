function ins = insUpdateForOneLoop(ins, imu, f)
% SINS Updating Alogrithm including attitude, velocity and position
% updating for one loop.
%
% Prototype: ins = insupdate(ins, imu)
% Inputs: ins - SINS structure array created by function 'insinit'
%         imu - gyro & acc incremental sample(s), maybe wvm which is the
%         gyro & acc incremental sample in one loop
%         f - 0 for optimal coning compensation methond
%             1 for accurate numerical solution based on picard
%             2 for accurate numerical solution based on taylor
%             3 for quaternion Runge-Kutta
%             4 for Bortz Runge-Kutta
% Output: ins - SINS structure array after updating

global glv
nn = size(imu, 1);    % 这里的imu实际上是外面的大循环中的一步，是指提取了nn个imu数据（角增量和速度增量），在外面是变量wvm，第一个维度即nn的大小
nts = nn*ins.ts;  nts2 = nts/2;  ins.nts = nts;    % 计算姿态更新时间间隔nts，以及nts2为中间时间间隔
wm = imu(:, 1:3); vm = imu(:, 4:6);    % nn个角增量和速度增量
coef = wm2wtcoef(ins.ts, nn);  % 根据nn个角增量采样可以构造一个N-1次的多项式角速度去拟合真实的角速度，迭代求解等效旋转矢量微分方程精确数值解，该项为转换矩阵系数

%% rotation & sculling compensation for velocity increment
dvbm = [0; 0; 0];
vmm = sum(vm, 1); wmm = sum(wm, 1);
scullm = scullpolyn(wm, vm);    % using polynomial fitting sculling compensation method
rotm = 1.0/2*cros(wmm, vmm);
dvbm = (vmm+(rotm+scullm)*glv.csCompensate)';    % velocity increment after rotation & sculling compensation
%% different attitude updating methods
if f == 0
    cm = glv.cs(nn-1,1:nn-1)*wm(1:nn-1,:);
    dphim = cros(cm,wm(nn,:));
    phim = (wmm+dphim*glv.csCompensate)';    % 等效旋转矢量，表示姿态的更新，以rotation vector表示
    dq = rv2q(phim);    % 将rv转换成四元数表示
elseif f == 1
    phim = btzpicard(wm'*coef, nts);
    dq = rv2q(phim);
elseif f == 2
    dq = qtaylor(wm'*coef, nts);
    phim = q2rv(dq);
elseif f == 3
    q1 = qrk4(ins.qnb, wm, nts);
    dq = qmul(qinv(ins.qnb), q1);
    phim = q2rv(dq);
elseif f == 4
    phim = btzrk4(wm, nts);
    dq = rv2q(phim);
end
%% earth & angular rate updating
vn01 = ins.vn+ins.an*nts2; pos01 = ins.pos+ins.Mpv*vn01*nts2;      % extrapolation at t1/2 速度，位置外推
ins.eth = ethupdate(ins.eth, pos01, vn01);
ins.wib = phim/nts; ins.fb = dvbm/nts;  % same as trjsimu
ins.web = ins.wib - ins.Cnb'*ins.eth.wnie;
ins.wnb = ins.wib - (ins.Cnb*rv2m(phim/2))'*ins.eth.wnin;
%% (1)velocity updating
ins.fn = qmulv(ins.qnb, ins.fb);    % 比力由b系转换到n系
% ins.an = qmulv(rv2q(-ins.eth.wnin*nts2),ins.fn) + ins.eth.gcc;
ins.an = rotv(-ins.eth.wnin*nts2, ins.fn) + ins.eth.gcc;  ins.anbar = 0.9*ins.anbar + 0.1*ins.an;    % 加速度计算
vn1 = ins.vn + ins.an*nts;    % 速度更新
%% (2)position updating
ins.Mpv(4)=1/ins.eth.RMh; ins.Mpv(2)=1/ins.eth.clRNh;
ins.Mpvvn = ins.Mpv*(ins.vn+vn1)/2;
ins.pos = ins.pos + ins.Mpvvn*nts;    % 位置更新
ins.vn = vn1;     % 速度更新
ins.an0 = ins.an;    % 加速度更新
%% (3)attitude updating
ins.qnb = qupdt2(ins.qnb, phim, ins.eth.wnin*nts);
[ins.qnb, ins.att, ins.Cnb] = attsyn(ins.qnb);
ins.avp = [ins.att; ins.vn; ins.pos];













