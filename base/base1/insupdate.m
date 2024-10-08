function ins = insupdate(ins, imu)
% SINS Updating Alogrithm including attitude, velocity and position
% updating.
%
% Prototype: ins = insupdate(ins, imu)
% Inputs: ins - SINS structure array created by function 'insinit'
%         imu - gyro & acc incremental sample(s)
% Output: ins - SINS structure array after updating
%
% See also  insinit, cnscl, earth, inspure, trjsimu, imuadderr, avpadderr, q2att,
%           inslever, alignvn, aligni0, etm, kffk, kfupdate, insplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/03/2008, 12/01/2013, 18/03/2014, 09/09/2014
    nn = size(imu,1);    % 这里的imu实际上是外面的大循环中的一步，是指提取了nn个imu数据（角增量和速度增量），在外面是变量wvm，第一个维度即nn的大小
    nts = nn*ins.ts;  nts2 = nts/2;  ins.nts = nts;    % 计算姿态更新时间间隔nts，以及nts2为中间时间间隔 
    [phim, dvbm] = cnscl(imu,0);    % coning    & sculling compensation, 圆锥/划桨误差补偿
    phim = ins.Kg*phim-ins.eb*nts; dvbm = ins.Ka*dvbm-ins.db*nts;  % calibration
    %% earth & angular rate updating 
    vn01 = ins.vn+ins.an*nts2; pos01 = ins.pos+ins.Mpv*vn01*nts2;      % extrapolation at t1/2 速度，位置外推
    if ins.openloop==0, ins.eth = ethupdate(ins.eth, pos01, vn01);
    elseif ins.openloop==1, ins.eth = ethupdate(ins.eth, ins.pos0, ins.vn0); end
    ins.wib = phim/nts; ins.fb = dvbm/nts;  % same as trjsimu
    ins.web = ins.wib - ins.Cnb'*ins.eth.wnie;
%     ins.wnb = ins.wib - ins.Cnb'*ins.eth.wnin;
    ins.wnb = ins.wib - (ins.Cnb*rv2m(phim/2))'*ins.eth.wnin;  % 2014-11-30
    %% (1)velocity updating
    ins.fn = qmulv(ins.qnb, ins.fb);    % 比力由b系转换到n系
%     ins.an = qmulv(rv2q(-ins.eth.wnin*nts2),ins.fn) + ins.eth.gcc;
    ins.an = rotv(-ins.eth.wnin*nts2, ins.fn) + ins.eth.gcc;  ins.anbar = 0.9*ins.anbar + 0.1*ins.an;
    vn1 = ins.vn + ins.an*nts;    % 速度更新
    %% (2)position updating
%     ins.Mpv = [0, 1/ins.eth.RMh, 0; 1/ins.eth.clRNh, 0, 0; 0, 0, 1];
    ins.Mpv(4)=1/ins.eth.RMh; ins.Mpv(2)=1/ins.eth.clRNh;
%     ins.Mpvvn = ins.Mpv*((ins.vn+vn1)/2+(ins.an-ins.an0)*nts^2/3);  % 2014-11-30
    ins.Mpvvn = ins.Mpv*(ins.vn+vn1)/2;
    ins.pos = ins.pos + ins.Mpvvn*nts;  
    ins.vn = vn1;     % 速度更新
    ins.an0 = ins.an;
    %% (3)attitude updating
    ins.Cnb0 = ins.Cnb;
%     ins.qnb = qupdt(ins.qnb, ins.wnb*nts);  % lower accuracy than the next line
    ins.qnb = qupdt2(ins.qnb, phim, ins.eth.wnin*nts);
    [ins.qnb, ins.att, ins.Cnb] = attsyn(ins.qnb);
    ins.avp = [ins.att; ins.vn; ins.pos];

