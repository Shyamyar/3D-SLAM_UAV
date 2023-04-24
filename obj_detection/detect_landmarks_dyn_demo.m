%% Initializatoin
clear;
close all;
clc;

%% Simulation Parameters
tn = 10;
t = 0:0.05:tn;
steps = length(t);

%% Random Landmark Pose Generation
n_L = 1000;                         % Number of Landmarks
r_L_lim = [7;11];                   % Radius zone limits for landmarks
th_L_lim = deg2rad([-180;180]);     % Azimuth zone limits for landmarks
phi_L_lim = deg2rad([-50;50]);      % Elevation zone limits for landmarks

% Polar Coordinates of Landmarks ([Az,El,r])
[L_ids,L_in_polar] = landmarks_polar(n_L,r_L_lim,th_L_lim,phi_L_lim);

% Cartesian Coordinates of Landmarks ([x,y,z])
L_in = sph2cart_shyam(L_in_polar);

%% Sensor Specs Limits
r_lim = 9;                      % Range Limit
th_lim = deg2rad([-45;45]);     % Azimuthal Limit
phi_lim = deg2rad([-30;30]);    % Elevation Limit

%% Robot Pose
radius = 5;
x = radius * cos(2*pi*(t/tn));
y = radius * sin(2*pi*(t/tn));
z = radius * sin(4*pi*(t/tn));
roll =  0 * pi * (t/tn);
pitch = 4 * pi * (t/tn);
yaw = 2 * pi * (t/tn);
r_pose = NaN(6,steps);

%% Structure for Sensed Landmark information
sensed = struct();

%% Simulation
for j = 1:steps
    %% Getting robot pose at t
    r_pose(:,j) = [x(j);y(j);z(j);roll(j);pitch(j);yaw(j)];     %[x;y;z;psi;theta;phi]

    %% Detecting Landmarks in FOV at t
    L_in = L_in(:); % Making sure L_in is a vector
    [sensed(j).L_id,sensed(j).L_b_polar] = sense_lmk_3D(L_ids,L_in,r_pose(:,j),r_lim,th_lim,phi_lim);
    L_in = reshape(L_in,[3,n_L]);
    sensed(j).L_in = L_in(:,sensed(j).L_id);
    
end

%% Plot

% Initiating Video Creation
Video_Creation = false;
if Video_Creation
    myVideo = VideoWriter('Landmark_Detection.avi');
    mov_cnt = 1;
    open(myVideo)
end

% Initiating Plot
figure('WindowState','maximized');
lmks = scatter3(L_in(2,:),L_in(1,:),-L_in(3,:),'r.'); % Landmarks
hold on;
path = animatedline('Color','k'); % Path of robot

for j = 1:steps
    % Plot changing robot and sensed landmarks
    robot = scatter3(r_pose(2,j),r_pose(1,j),-r_pose(3,j),50,'g+');
    sense_lmks = scatter3(sensed(j).L_in(2,:),sensed(j).L_in(1,:),-sensed(j).L_in(3,:),50,'bo');
    
    % Plot robot path
    addpoints(path,r_pose(2,j),r_pose(1,j),-r_pose(3,j));
    
    % Plot changing FOV
    [X,Y,Z]=fov_3D_rec(th_lim(2),phi_lim(2),r_lim);
    M=makehgtform('translate',[r_pose(2,j),r_pose(1,j),-r_pose(3,j)],'zrotate',pi/2-r_pose(6,j),'yrotate',-r_pose(5,j),'xrotate',r_pose(4,j));
    h=surf(X,Y,Z,'Parent',hgtransform('Matrix',M),'LineStyle','none','FaceAlpha',0.6);
    
    % Setting view to Front view of FOV
%     view(rad2deg([r_pose(6,j)+pi/2,r_pose(5,j)-100]))
%     view([180*(t(j)/tn)-37.5,180*(t(j)/tn)+30])
    
    % Plot details/parameters
    axis('equal')
    grid on
    max_ax = 15;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim([-max_ax,max_ax])
    ylim([-max_ax,max_ax])
    zlim([-max_ax,max_ax])
    
    % Animation rate
    drawnow limitrate;
    
    % Movie making
    if Video_Creation
        MM(mov_cnt)=getframe;
        frame=getframe(gcf);
        writeVideo(myVideo,frame);
        mov_cnt=mov_cnt+1;
    end
    
    % Handles update
    if j < steps
        delete(robot);
        delete(sense_lmks);
        delete(h);
    end
end

if Video_Creation
    close(myVideo);
end
% movie(MM)