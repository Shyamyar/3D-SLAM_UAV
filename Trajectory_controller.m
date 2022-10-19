function [F,phi_c,theta_c,r_c]=Trajectory_controller(states,xr,ur,Kt)
global m

% Variables to use
phi = states(7,1);
theta = states(8,1);
psi = states(9,1);
q = states(11,1);

% Getting Velrate in Inertial Frame
RB2N = C(3,psi) * C(2,theta) * C(1,phi); %DCM to convert from body to inertial
Velrate_in = RB2N * states(4:6);

% Calculating Error
traj_states = [states(1:3);Velrate_in;psi];
dx = traj_states-xr;

% Error Inputs for Calculating Control Inputs
du = -Kt * dx;
uc = du + ur;

% Thrust
up = [uc(1,1);uc(2,1);uc(3,1)];
F = m* sqrt(up'*up);

% Roll and Pitch Commands
R_psi = [cos(psi),sin(psi),0;-sin(psi),cos(psi),0;0,0,1];
z = -R_psi * up * m/F;
phi_c = asin(-z(2));
theta_c = atan2(z(1),z(3));

% Yaw rate Commands
r_c = (uc(4,1) * cos(theta) - q * sin(phi)) / cos(phi);

end