%% System Parameters
Jx = 0.114700;  %
Jy = 0.057600;  %
Jz = 0.171200;  %
g = 9.806650;   % m/s^2
m = 1.56;       % kg
J_vec = [Jx; Jy; Jz]; 
sys = [g; m; J_vec]; 

%% Simulation Time Parameters
sampling_rate = 10;         % Hertz
dt = 1/sampling_rate;       % Timestep
ti = 0;                     % RKM
tf = 50;                    % Final time
tspan = ti:dt:tf;           % Time span
tsteps = length(tspan);     % Total time steps
prop_it = 10;               % Propagation iterations
h = dt/prop_it;             % EKF timesteps
tsim = 0;                   % Simulation Time with true states

%% Lidar Sensor Specs/LimitsS (Based on Velobite Solid State LiDAR)
alpha_lim = deg2rad([-45; 45]);     % Azimuthal Limit (deg)
beta_lim = deg2rad([-35; 35]);      % Elevation Limit (deg)
delta_lim = 100;                    % Range Limit (m)

%% Noise Parameters
sn = 1; % Set Noise On/off

% LiDAR Sensor Measurement Error parameters (Based on Velodyne ULTRA PuckTM)
sigma_alpha = deg2rad(0.33);        % Azimuth measurement error Std. Dev. (rad)
sigma_beta = deg2rad(0.3);          % Elevation measurement error Std. Dev. (rad)
sigma_delta = 3e-2;                 % Range measurement error Std. Dev. (m)

% IMU Sensor Noises (Based on MPU-9250 IMU)
acc_ND = 300;                               % Noise Density Accelerometer (ug/rtHz)
gyro_ND = 0.01;                             % Noise Density Gyro (dps/rtHz)
sigma_az = acc_ND * 10^-6 * g *...
    sqrt(sampling_rate);                    % Std. Dev. Accelerometer (m/s^2)
sigma_omega = deg2rad(gyro_ND * ...
         sqrt(sampling_rate));              % Std. Dev. Gyro (rad/sec)

% Covariance Matrices
sigma_imu = sn * [sigma_az; ...
            sigma_omega * ones(3, 1)];  % Estimation Error Std. Dev. IMU
R_imu = diag(sigma_imu.^2);             % Estimation Error Covariance Matrix IMU               

sigma_zeta = sn * [sigma_alpha; ...
            sigma_beta; sigma_delta];   % Measurement Error Std. Dev. LiDAR
R_zeta = diag(sigma_zeta.^2);             % Measurement Error Covariance Matrix