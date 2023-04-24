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
disp("Available Trajectories: 8, (c)ircle, (s)piral, (st)raight")
destraj = input("Enter the Trajectory: ", "s"); 
despos0 = [0; 0; 0; pi/2]; % Trajectory offset
ydes = trajectory_funs(destraj,despos0); 

%% Initiation of # Landmarks and Time History
max_nL = 500;
t_n_m = zeros(1, max_nL);
t_n_u = zeros(1, max_nL);
t_n_k = zeros(1, max_nL);

%% Monte Carlo
for n_L = 40%repmat(5:5:200,[1,4])                % Number of Landmarks
% disp(n_L);
% %% Random Landmark Pose Generation (Capital L signifies all Landmarks)
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
% Phat = diag([0.01 * ones(3, 1); 0.01 * ones(3, 1); 0.02 * ones(3, 1)]);
% q = [0.01 * ones(3, 1); 0.01 * ones(3, 1); 0.02 * ones(3, 1)]; 
Phat = diag([0.001 * ones(3, 1); 0.1 * ones(3, 1); 0 * ones(3, 1)]);
q = [0.2 * ones(2, 1); 0.05; 0.01 * ones(3, 1); 0.2 * ones(3, 1)]; 
Q = diag(q.^2);                 % Process Error Covariance Matrix

%% Storage Variables
n_a = 12;       % Quad States
n_s = 9;        % Quad Estimates
ss = 1:n_s;     % Quad Estimates Position
a_his = NaN(n_a, tsteps); 
a_d_his = NaN(n_a, tsteps); 
ahat_his = NaN(n_a, tsteps);
u_his = NaN(4, tsteps); 
lambda_his = NaN(4, tsteps); 
err_s_his = NaN(n_s, tsteps); 
sigma3_s_his = NaN(n_s, tsteps); 
err_m_his = NaN(3, n_L, tsteps); 
sigma3_m_his = NaN(3, n_L, tsteps); 
err_m_mean = NaN(3, tsteps); 
sigma3_m_mean = NaN(3, tsteps); 

Lhat_loc = NaN(3,n_L); % Location of Lids in xhat
mhat_Lids = [];
Lo_his = struct("O", [], "y_o", [], "n_o", [], "N_L", [], ...
                "kappa", [], "ik", [], "n_k", [], "y_k", [], "Lhat_loc_k", [], ...
                "upsilon", [], "iu", [], "n_u", [], "y_u", []); 

mhat_his = struct("N_L", [], "Lids", []);
n_m_his = NaN(1, tsteps); 
n_u_his = NaN(1, tsteps); 

for i = 1:tsteps
    ti = tspan(i);
    tcomp = tspan(1:i); % Completed time steps

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
    Lo.N_L = N_L(:,Lo.O);                 % Observed Landmarks True Inertial Position
    
    % LiDAR Measurements with Error
    v = normrnd(zeros(3, Lo.n_o), repmat(sigma_zeta,1, Lo.n_o)); 
    Lo.y_o = Lo.y_o + v; 
    
    %% Categorize Landmarks
    M = find(~isnan(Lhat_loc(1,:)));   % Mapped Lmks L_ids
    n_m = numel(M);                    % No. of Mapped Lmks

    [Lo.kappa, Lo.ik] = intersect(Lo.O, M); % Mapped Observed L_ids
    Lo.n_k = numel(Lo.kappa);               % No. of Mapped and Observed Lmks
    Lo.y_k = Lo.y_o(:, Lo.ik);              % Mapped Observed Lmks Observation
    Lo.Lhat_loc_k = Lhat_loc(:, Lo.kappa);  % Position of Mapped Observed Lmks in mhat

    [Lo.upsilon, Lo.iu] = setdiff(Lo.O, Lo.kappa); % Unmapped Observed Lids
    Lo.n_u = numel(Lo.upsilon);                    % No. of Unmapped Observed Lmks
    Lo.y_u = Lo.y_o(:, Lo.iu);                     % Unmapped Observed Lmks Observation

    %% EKF-SLAM (Begins after i > 1)
    if i > 1
        % Prediction: (xhat+, Phat+)_i-1 --> (xhat-, Phat-)_i
        t_pred_s = tic;
        [xhat, Phat, Fi] = predict(tspan(i-1), dt, prop_it,...
            sys, xhat, Phat, lambda, f_s, F, G_q, R_imu, Q, Pdot_ss, Pdot_sm); 
        t_pred = toc(t_pred_s);

        % Update: (xhat-, Phat-)_i --> (xhat+, Phat+)_i
        t_up_s = tic;
        [xhat, Phat, Hi] = update(xhat, Phat,...
            Lo, h_o, H_s, H_l, R_zeta); 
        t_up = toc(t_up_s);
        
        % Observability
%         HC = zeros(size(Hi, 2));
%         HC(ss,ss) = Fi;
%         Obs_mat = obsv(HC, Hi);
%         rnk = rank(Obs_mat);
%         disp([rnk, size(xhat,1), size(Obs_mat, 2), Lo.n_k])
    elseif i == 1
        n_x = size(xhat,1);
        n_m = (n_x - n_s) / 3;  % No. of Lmks Estimated
        mm = (n_s + 1) : n_x;   % Lmk Estimates Positions
        t_pred = 0;
        t_up = 0;
    end

    % Registration
    shat = xhat(ss);
    Phat_ss = Phat(ss, ss);
    t_reg_s = tic;

    for j = 1:Lo.n_u
        Phat_sm = Phat(ss, mm);
        y_l = Lo.y_u(:, j);
        ju = Lo.upsilon(j);
        lhat = gk(shat, y_l);
        xhat = [xhat; lhat];    % Total Mapspace Augmentation
        
        Gs = G_s(shat, y_l);
        Gl = G_l(shat, y_l);
        Phat_ll = Gs * Phat_ss * Gs' + Gl * R_zeta * Gl';
        Phat_lx = Gs * [Phat_ss, Phat_sm];
        Phat_xl = Phat_lx';
        Phat = [Phat,    Phat_xl;
                Phat_lx, Phat_ll];  % Covariance Augmentation
        
        n_m = n_m + 1;
        loc = (3 * n_m - 2) : (3 * n_m); % Location of lhat in mhat
        Lhat_loc(:, ju) = loc + n_s;     % Location of lhat in xhat
        mhat_Lids = [mhat_Lids; ju];     % Lids Sequence in mhat
        
        n_x = size(xhat,1);
        mm = (n_s + 1) : n_x;               % Lmk Estimates Positions
    end
    t_reg = toc(t_reg_s);

    %% Estimated State Segregation
    rho_hat = xhat(1:3); 
    nu_hat = xhat(4:6); 
    Lambda_hat = xhat(7:9); 
    psi_hat = Lambda_hat(3); 

    %% Trajectory Control / Control Signal
    ahat = [rho_hat; nu_hat; Lambda_hat; omega_hat];
    [u, a_d] = Trajectory_Control(ti, sys, ahat, ydes, A, B, C_NB_fun); 

    %% Check Time of EKF-SLAM
    t_es = t_pred + t_up + t_reg;
    if t_es >= dt && i > 10
        break;
    end

    %% Storing True States History
    a_his(:, i) = a; 
    a_d_his(:, i) = a_d; 
    ahat_his(:, i) = ahat;
    u_his(:, i) = u; 
    lambda_his(:, i) = lambda;
    err_s_his(:,i) = ahat_his(1:9, i) - a_his(1:9, i);
    sigma3_s_his(:,i) = 3 .* sqrt(diag(Phat_ss));
    
    mhat_reshaped = reshape(xhat(mm), [3, n_m]);
    mhat_his(i).N_L = mhat_reshaped;   % Estimates Lmks Sequentially
    mhat_his(i).Lids = mhat_Lids;
    Phat_mm = Phat(mm, mm);
    err_m_his(:, 1:n_m, i) = mhat_reshaped - N_L(:, mhat_Lids);
    err_m_mean(:, i) = mean(err_m_his(:, 1:n_m, i), 2);
    sigma3_m_his(:,1:n_m, i) = reshape(3 .* sqrt(diag(Phat_mm)),...
                                        [3, n_m]);
    sigma3_m_mean(:, i) = mean(sigma3_m_his(:,1:n_m, i), 2);
    Lo_his(i) = Lo;
    n_m_his(i) = n_m;
    n_u_his(i) = Lo.n_u;
    if n_m > 0 && Lo.n_u > 0 && Lo.n_k > 0 && n_L > 10
        t_n_m(n_m) = mean([t_pred, t_n_m(n_m)], 'omitnan');
        t_n_k(Lo.n_k) = mean([t_up, t_n_k(Lo.n_k)], 'omitnan');
        t_n_u(Lo.n_u) = mean([t_reg, t_n_u(Lo.n_u)], 'omitnan');
    end

    %% Motion: Simulation for True Values at i + 1
    [~, a_ode] = ode45(f_a, ti:h:ti+dt, a, [], u, sys); 
    a = a_ode(end,:)'; 
    a(7) = pi2pi(a(7), a_his(7, i)); % wrapping phi to pi
    a(8) = pi2pi(a(8), a_his(8, i)); % wrapping theta to pi
    a(9) = pi2pi(a(9), a_his(9, i)); % wrapping psi to pi
    
    %% Check if errors within bound
    if any(isnan(err_s_his(:,i)),"all") || mean(abs(err_s_his(1:3,i))) >= 2
%         break;
    end

end
end

%% Completed Time Range Snippet abd Saving Paths
tcompx = tcomp(tcomp >= 0 & tcomp <= ti); 
fig_path = "results/Unknown_" + destraj;
vid_path = "videos/Unknown_" + destraj;
varstr(1) = struct("name", "_pos", "val", 1:3);
varstr(2) = struct("name", "_vel", "val", 4:6);
varstr(3) = struct("name", "_att", "val", 7:9);
varstr(4) = struct("name", "_angr", "val", 10:12);
set(0, 'DefaultLineLineWidth', 0.8);
vis = "on";

%% Estimation Plots and Vids
% Plotting_unknown;

%% Time Plots
% Time_Plots
