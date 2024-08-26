load 20-04-48-474\Matlab\data.mat
glvs
pos = data00000000(:, 17:19); % [h, longitude, latitude] in m, °, °
vel = data00000000(:, 20); % in km/h
gyro = data00000000(:, 6:8); % in °/s
acc = data00000000(:, 3:5); % in g
t = data00000000(:, 2);
t = t - t(1, 1);

pos = [pos(:, [3, 2, 1]), t];
position_vector = [pos, t];
pos(:, 1:2) = pos(:, 1:2) * glv.deg; % [latitude, longitude, h] in rad, rad, m

vel = [vel / 3.6, t]; % in m/s

gyro = gyro * glv.deg; % in rad/s
acc = acc * glv.g0; % in m/s2

imu = [gyro, acc, t];

save('./data_smallImu/data.mat', 'pos', 'position_vector', 'vel', 'imu');