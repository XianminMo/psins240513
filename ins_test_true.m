glvs
glvs
filename = 'data_0.csv';
dataArray = readmatrix(filename);
data = [dataArray(:, 7:9), dataArray(:, 4:6)];
timeStamps = (0:size(data,1)-1)' * 0.2;
data = [data, timeStamps];
imu = data;
avp0 = [0; 0; 0; 0; 0; 0; 0; 0; 0];    % avp initial value
%% pure inertial navigation & error plot
avp = inspure(imu, avp0, 'H', 1);
insplot(avp);
% plot_trajectory_from_position(trj.avp, avp)
% avp = inspure(imu, avp00, 'f', 1);
% avperr = avpcmpplot(trj.avp, avp);
