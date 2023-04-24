function [u_f, a_d] = Trajectory_Control(ti, sys, a, ydes, A, B, C_NB_fun)
    
%% State Segregation
rho = a(1:3);
nu = a(4:6);
Lambda = a(7:9);
omega = a(10:12);
psi = Lambda(3);
rho_dot = C_NB_fun(Lambda) * nu;
r = [rho; rho_dot; psi];
g = sys(1);
m = sys(2);

%% Differential Flatness
rho_d = ydes.rho_d(ti);
rho_dot_d = ydes.rho_dot_d(ti);
rho_ddot_d = ydes.rho_ddot_d(ti);
psi_d = ydes.psi_d(ti);
psi_dot_d = ydes.psi_dot_d(ti);
r_d = [rho_d; rho_dot_d; psi_d];
u_d = [rho_ddot_d - [0; 0; g]; psi_dot_d];

%% Trajectory LQR Controller
u = Trajectory_Controller(r_d, u_d, r, A, B);

%% Inverse Mapping
[f_T, Lambda_c, omega_c] = Inverse_Mapping(m, u, r_d);

%% Attitude Controller
tau = Attitude_Controller(sys, Lambda_c, omega_c, Lambda, omega);

%% Final Output
u_f = [f_T; tau];
a_d = [rho_d;
       C_NB_fun(Lambda_c)' * rho_dot_d;
       Lambda_c;
       omega_c];