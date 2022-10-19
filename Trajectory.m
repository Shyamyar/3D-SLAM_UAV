function [ur,xr,ytraj]=Trajectory(t)
global g

load('trajectory_funs.mat');
ytraj = ytraj_fun(t);
ydottraj = ydottraj_fun(t);
yddottraj = yddottraj_fun(t);

% ytraj=[pn,pe,pd,psi];
% ydottraj=[-a*w2*sin(w2*t),b*w1*cos(w1*t),-0.2,2*pi/T];
% yddottraj=[-a*w2*w2*cos(w2*t),-b*w1*w1*sin(w1*t),0,0];

ur=[yddottraj(1);yddottraj(2);yddottraj(3)-g;yddottraj(4)];
xr=[ytraj(1);ytraj(2);ytraj(3);ydottraj(1);ydottraj(2);ydottraj(3);ytraj(4)];

end