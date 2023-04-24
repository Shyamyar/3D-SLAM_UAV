function [f_T, Lambda_c, omega_c] = Inverse_Mapping(m, u_r, r_t)
    
%% Commaned Thrust
u_p = u_r(1:3);
f_T = m * sqrt(u_p' * u_p);

%% Commanded Angles
psi_t = r_t(7);
C_3 = C(3,psi_t);
z = - (m / f_T) * C_3' * u_p;
phi_c = asin(-z(2));
theta_c = atan2(z(1), z(3));
Lambda_c = [phi_c; theta_c; psi_t];

%% Commanded Angular rates
u_psi = u_r(4);
p_c = 0;
q_c = 0;
r_c = u_psi * cos(theta_c) / cos(phi_c);
omega_c = [p_c; q_c; r_c];