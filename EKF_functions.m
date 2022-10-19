%% Creating functions for EKF Prediction
syms t pn pe pd u v w phi theta psi p q r az gr

xhat = [pn;pe;pd;u;v;w;psi];
U = [az;p;q;r;phi;theta];
RB2N = C(3,psi) * C(2,theta) * C(1,phi); %DCM to convert from body to inertial
pos_dot = RB2N * [u;v;w];
coriolis_term = [(r*v-q*w);(p*w-r*u);(q*u-p*v)];
Thrustvec = [0;0;az];
Gravityvec = transpose(RB2N) * [0;0;gr]; % In Vehicle Frame
Velrate = coriolis_term + Thrustvec + Gravityvec;
psi_dot = q * sin(phi)/cos(theta) + r * cos(phi)/cos(theta);

xhat_dot_fun = [pos_dot;Velrate;psi_dot];
xhat_dot = matlabFunction(xhat_dot_fun,'Vars',{t,xhat,U,gr});

Jac_xhat = jacobian(xhat_dot_fun,xhat);
F_k = matlabFunction(Jac_xhat,'Vars',{t,xhat,U,gr});

save('EKF_functions','xhat_dot','F_k');
    