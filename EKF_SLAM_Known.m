%% Clearing Workspace
clc; 
clear; 
close all; 

%% Include Folders
addpath("extras\");

%% State Functions
load("State_functions");

%% Simulation Parameters
sim_parameters; 

%% Desired Trajectory
disp("Available Trajectories: 8, (c)ircle, (s)piral")
destraj = input("Enter the Trajectory: ", "s"); 
despos0 = [0; 0; 0; pi/2]; % Trajectory offset
ydes = trajectory_funs(destraj,despos0); 

%% Random Landmark Pose Generation (Capital L signifies all Landmarks)
% n_L = 40;                           % Number of Landmarks
% alpha_L_lim = deg2rad([-180; 180]); % Azimuth zone limits for landmarks
% beta_L_lim = deg2rad([-40; 40]);    % Elevation zone limits for landmarks
% delta_L_lim = [8; 15];              % Radius zone limits for landmarks
% 
% % Inertial Coordinates (N) of Landmarks (Cartesian (L) and Polar (Lp))
% [n_L, Lids, N_L, N_Lp] = landmarks_polar(n_L, alpha_L_lim, beta_L_lim, delta_L_lim); 
load("lmk_data");

%% Initial Conditions, Tuning Phat0 and Q
a = zeros(12, 1);    % Initial True Quad State
xhat = zeros(9, 1);  % Assuming Initial State at Origin and Stationary
% Phat = diag([1 * ones(3, 1); 1 * ones(3, 1); 0.02 * ones(3, 1)]);
% q = [0.01 * ones(3, 1); 0.01 * ones(3, 1); 0.02 * ones(3, 1)]; 
Phat = diag([0.01 * ones(3, 1); 0.01 * ones(3, 1); 0.02 * ones(3, 1)]);
q = [0.01 * ones(3, 1); 0.01 * ones(3, 1); 0.02 * ones(3, 1)]; 
% Phat = diag([0.001 * ones(3, 1); 0.1 * ones(3, 1); 0 * ones(3, 1)]);
% q = [0.02 * ones(2, 1); 0.05; 0.01 * ones(3, 1); 0.2 * ones(3, 1)]; 
Q = diag(q.^2);                 % Process Error Covariance Matrix

%% Storage Variables
n_a = 12;       % Quad States
n_s = 9;        % Quad Estimates
a_his = NaN(n_a, tsteps); 
a_d_his = NaN(n_a, tsteps); 
ahat_his = NaN(n_a, tsteps);
u_his = NaN(4, tsteps); 
lambda_his = NaN(4, tsteps); 
err_s_his = NaN(n_s, tsteps); 
sigma3_s_his = NaN(n_s, tsteps); 
Lo_his = struct("O",[],"y_o",[],"n_o",[],"N_L",[]); 

for i = 1:tsteps
    ti = tspan(i)
    tcomp = tspan(1:i); % Completed time steps
    tic;

    %% True State Segregation
    rho = a(1:3); 
    nu = a(4:6); 
    Lambda = a(7:9); 
    omega = a(10:12); 
    psi = Lambda(3); 

    %% IMU Sensor Estimates After Noise Filtering
    eta = normrnd(zeros(4,1), sigma_imu);   % Estimation Error 
    if i == 1
        az_hat = - g + eta(1);          % Vertical Accelerometer Estimate
        omega_hat = omega + eta(2:4);   % Gyro Estimate
    else
        az_hat = - (u(1) / m) + eta(1);   % Vertical Accelerometer Estimate
        omega_hat = omega + eta(2:4);   % Gyro Estimate
    end
    lambda = [az_hat; omega_hat];          % Measurements serving as control for SLAM

    %% LiDAR Observation: Measurements at i
    % LiDAR Observations
    C_NB = C_NB_fun(Lambda);
    B_L = C_NB' * (N_L - rho);              % Cartesian in Body Frame
    Lo = observed_lmks(Lids, B_L, ...
        alpha_lim, beta_lim, delta_lim) ;   % Observed Landmarks
    Lo.N_L = N_L(:,Lo.O);                 % Observed Landmarks Inertial
    
    % LiDAR Measurements with Error
    v = normrnd(zeros(3, Lo.n_o), repmat(sigma_zeta,1, Lo.n_o)); 
    Lo.y_o = Lo.y_o + v; 
    
    %% EKF-SLAM (Begins after i > 1)
    if i > 1
        % Prediction: (xhat+, Phat+)_i-1 --> (xhat-, Phat-)_i
        [xhat, Phat, Fi] = predict_known(tspan(i-1), dt, prop_it,...
            sys, xhat, Phat, lambda, f_s, F, G_q, R_imu, Q, Pdot_ss); 

        % Update: (xhat-, Phat-)_i --> (xhat+, Phat+)_i
        [xhat, Phat, Hi] = update_known(xhat, Phat,...
            Lo, h_o, H_s, R_zeta); 
        
        % Observability
%         Obs_mat = obsv(Fi, Hi);
%         rnk = rank(Obs_mat);
%         disp([rnk, size(xhat,1), size(Obs_mat, 2)])
    end

    %% Estimated State Segregation
    rho_hat = xhat(1:3); 
    nu_hat = xhat(4:6); 
    Lambda_hat = xhat(7:9); 
    psi_hat = Lambda_hat(3); 

    %% Trajectory Control / Control Signal
    ahat = [rho_hat; nu_hat; Lambda_hat; omega_hat];
    [u, a_d] = Trajectory_Control(ti, sys, ahat, ydes, A, B, C_NB_fun); 

    %% Check Time of EKF-SLAM
    dt_es = toc;
    if dt_es >= dt && i > 10
        break;
    end

    %% Storing True States History
    a_his(:, i) = a; 
    a_d_his(:, i) = a_d; 
    ahat_his(:, i) = ahat;
    u_his(:, i) = u; 
    lambda_his(:, i) = lambda;
    err_s_his(:,i) = ahat_his(1:9, i) - a_his(1:9, i);
    sigma3_s_his(:,i) = 3 .* sqrt(diag(Phat));
    Lo_his(i) = Lo;

    %% Motion: Simulation for True Values at i + 1
    [~, a_ode] = ode45(f_a, ti:h:ti+dt, a, [], u, sys); 
    a = a_ode(end,:)'; 
    a(7) = pi2pi(a(7), a_his(7,i)); % wrapping phi to pi
    a(8) = pi2pi(a(8), a_his(8,i)); % wrapping theta to pi
    a(9) = pi2pi(a(9), a_his(9,i)); % wrapping psi to pi
    
    %% Check if errors within bound
    if any(isnan(err_s_his(:,i)),"all") || mean(abs(err_s_his(1:3,i))) >= 2
%         break;
    end

end

%% Completed Time Range Snippet abd Saving Paths
tcompx = tcomp(tcomp >= 0 & tcomp <= ti); 
fig_path = "results/Known_" + destraj;
vid_path = "videos/Known_" + destraj;
varstr(1) = struct("name", "_pos", "val", 1:3);
varstr(2) = struct("name", "_vel", "val", 4:6);
varstr(3) = struct("name", "_att", "val", 7:9);
varstr(4) = struct("name", "_angr", "val", 10:12);
set(0, 'DefaultLineLineWidth', 0.8);
vis = "on";

%% Estimation Plots and Vids
% Plotting_known;