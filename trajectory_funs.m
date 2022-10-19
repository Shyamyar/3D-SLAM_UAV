%% Creating Trajectory Functions
syms tn

a = 5;
b = 5;
c = 0;
n = 0*tn;
T = 20;
w1 = 2*pi/T;
w2 = w1;
w3 = w1;
pn = a * cos(w2*tn);
pe = b * sin(w1*tn);
pd = n + c*sin(w3*tn);
psi = (w1*tn) + pi/2;

ytraj_fun = matlabFunction([pn;pe;pd;psi],'Vars',{tn});
ydottraj_fun = matlabFunction(diff(ytraj_fun,tn),'Vars',{tn});
yddottraj_fun = matlabFunction(diff(ydottraj_fun,tn),'Vars',{tn});

save('trajectory_funs.mat','ytraj_fun','ydottraj_fun','yddottraj_fun');