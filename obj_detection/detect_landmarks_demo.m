%% Initializatoin
clear;
close all;
clc;

%% Random Landmark Pose Generation
n_L = 100;                          % Number of Landmarks
r_L_lim = [7;11];                    % Radius zone limits for landmarks
th_L_lim = deg2rad([-180;180]);     % Azimuth zone limits for landmarks
phi_L_lim = deg2rad([-180;180]);    % Elevation zone limits for landmarks

% Polar Coordinates of Landmarks
[L_ids,L_in_polar] = landmarks_polar(n_L,r_L_lim,th_L_lim,phi_L_lim);

% Cartesian Coordinates of Landmarks
L_in = sph2cart_shyam(L_in_polar);

%% Robot Pose
r_pose = [0;0;0;pi/3;pi/4;-pi/3];     %[x;y;z;psi;theta;phi]

%% Sensor Specs Limits
r_lim = 12;                      % Range Limit
th_lim = deg2rad([-45;45]);     % Azimuthal Limit
phi_lim = deg2rad([-30;30]);    % Elevation Limit

%% Detecting Landmarks in FOV and Plot
L_in = L_in(:); % Making sure L_in is a vector
[L_id_sensed,L_b_polar_sensed] = sense_lmk_3D(L_ids,L_in,r_pose,r_lim,th_lim,phi_lim);
L_in = reshape(L_in,[3,n_L]);
L_in_sensed = L_in(:,L_id_sensed);

%% Plot
fig = figure();
scatter3(r_pose(2),r_pose(1),-r_pose(3),50,'+')
hold on
scatter3(L_in(2,:),L_in(1,:),-L_in(3,:),'.')
scatter3(L_in_sensed(2,:),L_in_sensed(1,:),-L_in_sensed(3,:),50,'bo')
if n_L<=20
    text(L_in_sensed(2,:),L_in_sensed(1,:),-L_in_sensed(3,:),string(L_id_sensed))
end
legend('Robot','Landmarks','Sensed Landmarks')

% [X,Y,Z]=fov_3D_sq(sqrt(2*(r_lim*tan(th_lim(2)))^2),r_lim);
[X,Y,Z]=fov_3D_rec(th_lim(2),phi_lim(2),r_lim);
M=makehgtform('translate',[r_pose(2),r_pose(1),-r_pose(3)],'zrotate',pi/2-r_pose(6),'yrotate',-r_pose(5),'xrotate',r_pose(4));
h=surf(X,Y,Z,'Parent',hgtransform('Matrix',M),'LineStyle','none','FaceAlpha',0.6);
% view(rad2deg([r_pose(6)+pi/2,-r_pose(5)]))
% view(rad2deg([r_pose(6)+pi/2,r_pose(5)-pi/6]))
axis('equal')
grid on
max_ax = 15;
xlabel('x'); ylabel('y'); zlabel('z');
xlim([-max_ax,max_ax])
ylim([-max_ax,max_ax])
zlim([-max_ax,max_ax])