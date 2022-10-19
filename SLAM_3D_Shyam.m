%% Clearing Workspace
clc;
clear;
close all;

%% System Parameters
global Jx Jy Jz g m

Jx = 0.114700;%
Jy = 0.057600;%
Jz = 0.171200;%
g = 9.806650;%m/s^2
m = 1.56;%kg

%% Trajectory
trajectory_funs;

%% Lidar Jacobian;
global xhat_dot F_k H_x H_k H_L Q R
% Lidar_jac; % gives h(x) and H(xhat) matrix
% EKF_functions; % gives f(x) i.e xhat_dot and F(xhat,U_ekf) matrix
load('Lidar_jac','H_x','G_x','H_k','H_L','G_k','G_L');
load('EKF_functions','xhat_dot','F_k');

%% Simulation Parameters
dt = 1/10;                 % Timestep
tn = 0;                     % RKM
tf = 40;                    % Final time
tspan = tn:dt:tf;           % Time span
tsteps = length(tspan);     % Total time steps
prop_it = 10;
h = dt/prop_it;

%% Initial Conditions (Range Kutta_requirements)
ytraj0 = ytraj_fun(0);
ydottraj0 = ydottraj_fun(0);
[pn0,pe0,pd0,psi0] = deal(ytraj0(1),ytraj0(2),ytraj0(3),ytraj0(4));
[u0,v0,w0,psidot0] = deal(ydottraj0(1),ydottraj0(2),ydottraj0(3),ydottraj0(4));
[phi0,theta0] =deal(0,0);
[phidot0,thetadot0] =deal(0,0);
turnrate_tf0 = turnrate_BI(phi0,theta0);
Attrate0 = turnrate_tf0 \ [phidot0;thetadot0;psidot0];
states_IC = [pn0;pe0;pd0;u0;v0;w0;phi0;theta0;psi0;Attrate0];  % Initial State
states = states_IC;

%% Random Landmark Pose Generation
n_L = 100;                        % Number of Landmarks
r_L_lim = [8;20];                 % Radius zone limits for landmarks
az_L_lim = deg2rad([-180;180]);   % Azimuth zone limits for landmarks
elev_L_lim = deg2rad([-20;20]);   % Elevation zone limits for landmarks

% Polar Coordinates of Landmarks
[L_ids,L_in_polar] = landmarks_polar(n_L,r_L_lim,az_L_lim,elev_L_lim);

% Cartesian Coordinates of Landmarks
L_in = sph2cart_shyam(L_in_polar); %[pn,pe,pd] form

% Structure for Sensed Landmark information
L_sense = struct();

%% Lidar Sensor Specs/Limits
r_lim = 20;                     % Range Limit
az_lim = deg2rad([-45;45]);     % Azimuthal Limit
elev_lim = deg2rad([-30;30]);   % Elevation Limit

%% Noise Parameters
set_noise = 1;
% Lidar Sensor Measurement Error parameters
r1 = 0.01*set_noise;                     % Azimuth measurement error std dev (m)
r2 = 0.01*set_noise;                     % Elevation measurement error std dev (rad)
r3 = 0.1*set_noise;                      % Range measurement error std dev (rad)

% IMU and Accelerometer Sensor Noises
az_noise = 0.1*set_noise;
gyro_noise = 0.01*set_noise;
att_ekf_noise = 0.01*set_noise;

%% Covariance Matrices
q = set_noise .* [0.1;0.1;0.1;0.1;0.1;0.1;0.01] * 2;
Q = diag(q.^2);              % Process Error Covariance Matrix
r_ob = [r1;r2;r3];
R = diag(r_ob.^2);           % Measurement Error Covariance Matrix

%% EKF Initialization
% xhat = [states_IC(1:6);states(9)];          % Initial States for EKF same at states_IC
xhat = [6;0;5;0;0;0;pi/2];                     % Initial States for EKF far from states_IC
Phat = diag([10;10;10;10;10;10;5]);      % Initial State Error Covariance Matrix

%% Some Storage Variables
states_his = NaN(12,tsteps);
traj_his = NaN(7,tsteps);
Atttraj_his = NaN(3,tsteps);
U_ekf = NaN(6,tsteps);
r_pose = NaN(6,tsteps);
xhat_his = NaN(7,tsteps);
err_his = NaN(7,tsteps);
sigma3_his = NaN(7,tsteps);

%% Simulation True
i = 1;
states_his(:,i) = states;

for i = 1:tsteps
    tn = tspan(i);
    
    %% Measurement and Control Input Creation
    % Accelerometer Measurements
    if i==1
        az = - g + normrnd(0,az_noise);
    else
        az = -(Thrust/m) + normrnd(0,az_noise);
    end

    % Gyro Sensor Measurements
    pz = states(10,1) + normrnd(0,gyro_noise);
    qz = states(11,1) + normrnd(0,gyro_noise);
    rz = states(12,1) + normrnd(0,gyro_noise);

    % Attitude Sensor Measurements
    phiz = states(7,1) + normrnd(0,att_ekf_noise);
    thetaz = states(8,1) + normrnd(0,att_ekf_noise);
    
    % Lidar Sensor Measurements
    r_pose(:,i) = [states(1:3);states(7:9)];  
    L_in = L_in(:); % Making sure L_in is a vector
    [L_sense(i).L_id,L_sense(i).L_b_polar] = sense_lmk_3D(L_ids,L_in,...
        r_pose(:,i),r_lim,az_lim,elev_lim);       % L_sense Landmarks Information Polar
    L_in = reshape(L_in,[3,n_L]);
    L_sense(i).L_in = L_in(:,L_sense(i).L_id);    % L_sense Landmarks Inertial
    
    v_r1 = normrnd(0,r1);          % Measurement Error in Azimuth
    v_r2 = normrnd(0,r2);          % Measurement Error in Elevation
    v_r3 = normrnd(0,r3);          % Measurement Error in Range
    v_L = [v_r1;v_r2;v_r3];
    L_sense(i).L_b_polar_err = L_sense(i).L_b_polar + v_L;   % Lidar Readings with measurement noise
    L_sense(i).L_in = L_in(:,L_sense(i).L_id);
    
    U_ekf(:,i) = [az;pz;qz;rz;phiz;thetaz];
    
    %% Trajectory Controller
    [ur,xr,traj] = Trajectory(tspan(i));% Generating Reference Trajectory ([pn,pe,pd,psi])
    Kt = Trajectory_LQR_K();
%     [Thrust,phi_c,theta_c,r_c] = Trajectory_controller([xhat(1,1);xhat(2,1);xhat(3,1);
%         xhat(4,1);xhat(5,1);xhat(6,1);phiz;thetaz;xhat(7,1);pz;qz;rz],xr,ur,Kt);%EKF
    [Thrust,phi_c,theta_c,r_c] = Trajectory_controller(states,xr,ur,Kt);%Integration
    
    %% Attitude Controller
%     att_states = [phiz-phi_c,pz,thetaz-theta_c,qz,rz-r_c];%EKF
    att_states = [states(7,1)-phi_c,states(10,1),states(8,1)-theta_c,states(11,1),states(12,1)-r_c]; %Integration
    Ka = Attitude_LQR_K();
    taus = -Ka * att_states';
    
    %% Storing Trajectory History
    traj_his(:,i) = xr;
    Atttraj_his(:,i) = [phi_c;theta_c;r_c];
    
    %% Control Input for True Trajectory
    U = [taus;Thrust];
    
    %% Integrating states dot
    Dynamics_true = @(t,x,u) Dynamics(t,x,u);
    for j = 1:prop_it
        [tn,states]=RK4_CDEKF(Dynamics_true,tn,states,U,h);
    end
    
    %% Storing True States History
    if i<tsteps
        states_his(:,i+1) = states;
    end
    
end

%% EKF
i = 1;
xhat_his(:,i) = xhat;
err_his(:,i) = xhat_his(:,i) - states_his([1,2,3,4,5,6,9],i);
sigma3_his(:,i) = 3 .* sqrt(diag(Phat(:,:)));

for i = 1:tsteps
    tn = tspan(i);
    
    %% EKF
    [xhat,Phat] = CDEKF_Shyam(tn,xhat,Phat,U_ekf(:,i),L_sense(i),dt,prop_it);
    
    %% Storing Estimated States History
    if i<tsteps
        xhat_his(:,i+1) = xhat;
        err_his(:,i+1) = xhat_his(:,i) - states_his([1,2,3,4,5,6,9],i+1);
        sigma3_his(:,i+1) = 3 .* sqrt(diag(Phat(:,:)));
    end

end

%% Error Evaluation
root_mean_r_x = sqrt(sum(err_his(1,:).^2) / tsteps);
root_mean_r_y = sqrt(sum(err_his(2,:).^2) / tsteps);
root_mean_r_z = sqrt(sum(err_his(3,:).^2) / tsteps);
root_mean_r = mean([root_mean_r_x,root_mean_r_y,root_mean_r_z]);

err_disp_r = sprintf('Root Mean Square Error for position of UAV with Known Landmarks = %.2f %%', root_mean_r*100);
disp(err_disp_r)

%% Animation
figure('WindowState','maximized');

lmks = scatter3(L_in(2,:),L_in(1,:),-L_in(3,:),'k.'); % True Landmarks
hold on;
xlabel('North')
ylabel('East')
zlabel('Up')
axis('equal')
grid on;
xlim([-15,15])
ylim([-15,15])
zlim([-15,15])
rotate3d on
if size(L_in,2) <= 30
    text(L_in(2,:),L_in(1,:),-L_in(3,:),string(L_ids));
end

for i = 1:tsteps
    % Quad 1 Estimated
    quad1 = Draw_Quad(U_ekf(5,i),-U_ekf(6,i),pi/2-xhat_his(7,i),xhat_his(2,i),xhat_his(1,i),-xhat_his(3,i),0.3);
    quad1_traj = plot3(xhat_his(2,1:i),xhat_his(1,1:i),-(xhat_his(3,1:i)),'-','DisplayName','Estimated');
    quad1_traj.Color = 'red'; % Estimated Trajectory

    % Quad 2 True
    quad2 = Draw_Quad(states_his(7,i),-states_his(8,i),pi/2-states_his(9,i),states_his(2,i),states_his(1,i),-states_his(3,i),2); 
    quad2_traj = plot3(states_his(2,1:i),states_his(1,1:i),-(states_his(3,1:i)),'-','DisplayName','True');
    quad2_traj.Color = 'black'; % True Trajectory
    
    % Sensed Landmarks
    sensed_lmks = scatter3(L_sense(i).L_in(2,:),L_sense(i).L_in(1,:),-L_sense(i).L_in(3,:),50,'ro');
    
    % FOV Generation
    [X,Y,Z] = fov_3D_rec(az_lim(2),elev_lim(2),r_lim);
    M = makehgtform('translate',[states_his([2,1],i);-states_his(3,i)],'zrotate',pi/2-states_his(9,i),'yrotate',-states_his(8,i),'xrotate',states_his(7,i));
    fov = surf(X,Y,Z,'Parent',hgtransform('Matrix',M),'LineStyle','none','FaceAlpha',0.3);
    
    % Creating animation
    drawnow limitrate;
    handles_del = [quad1;quad1_traj;quad2;quad2_traj;sensed_lmks;fov];
    if i<tsteps
        delete(handles_del);
    end
    
end

%% Control Tracking Plots (True vs Commanded)
% figure('WindowState','maximized');
% subplot(6,2,1)
% plot(tspan,states_his(1,:),tspan,traj_his(1,:))
% xlabel('Time (sec)')
% ylabel('pn')
% subplot(6,2,3)
% plot(tspan,states_his(2,:),tspan,traj_his(2,:))
% xlabel('Time (sec)')
% ylabel('pe')
% subplot(6,2,5)
% plot(tspan,-states_his(3,:),tspan,-traj_his(3,:))
% xlabel('Time (sec)')
% ylabel('h')
% subplot(6,2,7)
% plot(tspan,states_his(4,:),tspan,traj_his(4,:))
% xlabel('Time (sec)')
% ylabel('u')
% subplot(6,2,9)
% plot(tspan,states_his(5,:),tspan,traj_his(5,:))
% xlabel('Time (sec)')
% ylabel('v')
% subplot(6,2,11)
% plot(tspan,-states_his(6,:),tspan,-traj_his(6,:))
% xlabel('Time (sec)')
% ylabel('w')
% subplot(6,2,2)
% plot(tspan,states_his(7,:),tspan,Atttraj_his(1,:))
% xlabel('Time (sec)')
% ylabel('phi')
% subplot(6,2,4)
% plot(tspan,states_his(8,:),tspan,Atttraj_his(2,:))
% xlabel('Time (sec)')
% ylabel('theta')
% subplot(6,2,6)
% plot(tspan,states_his(9,:),tspan,traj_his(7,:))
% xlabel('Time (sec)')
% ylabel('psi')
% subplot(6,2,8)
% plot(tspan,states_his(10,:))
% xlabel('Time (sec)')
% ylabel('p')
% subplot(6,2,10)
% plot(tspan,states_his(11,:))
% xlabel('Time (sec)')
% ylabel('q')
% subplot(6,2,12)
% plot(tspan,states_his(12,:),tspan,Atttraj_his(3,:))
% xlabel('Time (sec)')
% ylabel('r')
% legend('True','Commanded')

%% Estimation Tracking Plots (Estimated Vs True)
figure('WindowState','maximized');
subplot(4,2,1)
plot(tspan,states_his(1,:),tspan,xhat_his(1,:))
xlabel('Time (sec)')
ylabel('pn (meter)')
legend('True','Estimated')
subplot(4,2,2)
plot(tspan,states_his(2,:),tspan,xhat_his(2,:))
xlabel('Time (sec)')
ylabel('pe (meter)')
subplot(4,2,3)
plot(tspan,-states_his(3,:),tspan,-xhat_his(3,:))
xlabel('Time (sec)')
ylabel('h (meter)')
subplot(4,2,4)
plot(tspan,states_his(4,:),tspan,xhat_his(4,:))
xlabel('Time (sec)')
ylabel('u (m/s)')
subplot(4,2,5)
plot(tspan,states_his(5,:),tspan,xhat_his(5,:))
xlabel('Time (sec)')
ylabel('v (m/s)')
subplot(4,2,6)
plot(tspan,-states_his(6,:),tspan,-xhat_his(6,:))
xlabel('Time (sec)')
ylabel('w (m/s)')
subplot(4,2,7)
plot(tspan,states_his(9,:),tspan,xhat_his(7,:))
xlabel('Time (sec)')
ylabel('psi (rad)')
suptitle('Estimated Vs True States')

%% Estimation Error Plots
figure('WindowState','maximized');
subplot(4,2,1)
plot(tspan,err_his(1,:),tspan,sigma3_his(1,:),tspan,-sigma3_his(1,:))
xlabel('Time (sec)')
ylabel('e_{pn} (meter)')
legend('State Error','3-sigma','- 3-sigma')
subplot(4,2,2)
plot(tspan,err_his(2,:),tspan,sigma3_his(2,:),tspan,-sigma3_his(2,:))
xlabel('Time (sec)')
ylabel('e_{pe} (meter)')
subplot(4,2,3)
plot(tspan,err_his(3,:),tspan,sigma3_his(3,:),tspan,-sigma3_his(3,:))
xlabel('Time (sec)')
ylabel('e_h (meter)')
subplot(4,2,4)
plot(tspan,err_his(4,:),tspan,sigma3_his(4,:),tspan,-sigma3_his(4,:))
xlabel('Time (sec)')
ylabel('e_u (m/s)')
subplot(4,2,5)
plot(tspan,err_his(5,:),tspan,sigma3_his(5,:),tspan,-sigma3_his(5,:))
xlabel('Time (sec)')
ylabel('e_v (m/s)')
subplot(4,2,6)
plot(tspan,err_his(6,:),tspan,sigma3_his(6,:),tspan,-sigma3_his(6,:))
xlabel('Time (sec)')
ylabel('e_w (m/s)')
subplot(4,2,7)
plot(tspan,err_his(7,:),tspan,sigma3_his(7,:),tspan,-sigma3_his(7,:))
xlabel('Time (sec)')
ylabel('e_{\psi}  (rad)')
suptitle('Estimation Error and 3-Sigma Boundaries for UAV');
