function [r_t, u_t] = Differential_Flatness(ti, g, des, despos0)

%% Values from Desired Trajectory Functions
rho_t = des.rho_d(ti, despos0);
rho_dot_t = des.rho_dot_d(ti);
rho_ddot_t = des.rho_ddot_d(ti);
psi_t = des.psi_d(ti);
psi_dot_t = des.psi_dot_d(ti);
r_t = [rho_t; rho_dot_t; psi_t];
u_t = [rho_ddot_t - [0; 0; g]; psi_dot_t];