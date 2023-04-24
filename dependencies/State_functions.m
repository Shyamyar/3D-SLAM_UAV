%% Creating functions for EKF Prediction
syms t m Jx Jy Jz gk 
syms pn pe pd u v w phi theta psi p q r 
syms az f_T tau_phi tau_theta tau_psi
syms pn_dot pe_dot pd_dot u_p1 u_p2 u_p3 u_psi
syms pnL peL pdL
syms alpha beta delta 

% System Parameters
J_vec= [Jx; Jy; Jz]; 
J = [Jx,  0,   0; 
      0, Jy,   0; 
      0,  0,  Jz]; 
sys = [gk; m; J_vec]; 

% Primary States 
rho = [pn; pe; pd]; 
nu = [u; v; w]; 
Lambda = [phi; theta; psi]; 
omega = [p; q; r]; 
rho_dot_lqr = [pn_dot; pe_dot; pd_dot]; 
l = [pnL; peL; pdL]; 
y_l = [alpha; beta; delta]; 

% Torques, Forces
tau = [tau_phi; tau_theta; tau_psi]; 
N_f_g = [0; 0; m * gk]; 
B_f_T = [0; 0; -f_T]; 
chi = [0; 0; az]; 

% Dynamics State Function
a = [rho; nu; Lambda; omega]; 
u_c = [f_T; tau]; 

% EKF State Function
s = a(1:9); 
u_s = [az; p; q; r]; 

% State Rate Change Expressions w/o Pertubation
C_NB = C(3, psi) * C(2, theta) * C(1, phi); %DCM to convert from body to inertial
rho_dot = C_NB * nu; 
nu_dot = - cross(omega, nu) + (1/m) * (transpose(C_NB) * N_f_g + B_f_T); 
nu_dotz = - cross(omega, nu) + (1/m) * transpose(C_NB) * N_f_g + chi; 
D = D_matrix(phi, theta); 
Lambda_dot = D * omega; 
omega_dot =  J \ (tau - cross(omega, J * omega));

% DFB-LQR State Function
r_r = [rho; rho_dot_lqr; psi]; 
u_p = [u_p1; u_p2; u_p3]; 
u_r = [u_p; u_psi];
rho_ddot = u_p + (1/m) * N_f_g; 

% State Function Dynamics
a_dot = simplify([rho_dot; nu_dot; Lambda_dot; omega_dot]); 
f_a = matlabFunction(a_dot, 'Vars', {t, a, u_c, sys}); 

% State Function DFB-LQR
r_r_dot = simplify([rho_dot_lqr; rho_ddot; u_psi]); 
f_r = matlabFunction(r_r_dot, 'Vars', {t, r_r, u_r, sys}); 

% State Function EKF
s_dot = simplify([rho_dot; nu_dotz; Lambda_dot]); 
f_s = matlabFunction(s_dot, 'Vars', {t, s, u_s, sys}); 

% Jacobian A, B
Jac_r_r = jacobian(r_r_dot, r_r); 
A = double(Jac_r_r); 
Jac_u_r = jacobian(r_r_dot, u_r); 
B = double(Jac_u_r); 
Jac_g = jacobian(r_r_dot, gk); 
b = double(Jac_g); 

% Jacobian F
Jac_s = jacobian(s_dot, s); 
F = matlabFunction(Jac_s, 'Vars', {t, s, u_s, sys}); 

% Jacobian F
Jac_u_s = jacobian(s_dot, u_s); 
G_q = matlabFunction(Jac_u_s, 'Vars', {t, s, u_s, sys}); 

% G_w (Process Error: G_w * w * G_w')
Jac_w = jacobian(s_dot, a); 
G_w = matlabFunction(Jac_w, 'Vars', {t, a, sys}); 
G_w = G_w(t, a, sys);
G_w(G_w ~= 0) = 1;
G_w = double(G_w);

% Observation Model
c = transpose(C_NB) * (l - rho); 
h_o_sym = cart2sph_shyam(c); 
h_o = matlabFunction(h_o_sym, 'Vars', {s, l}); 

% Inverse Observation Model
var_rho = sph2cart_shyam(y_l); 
g_sym = C_NB * var_rho + rho; 
gk = matlabFunction(g_sym, 'Vars', {s, y_l}); 

% Jacobian H
H_s_sym = jacobian(h_o_sym, s); 
H_l_sym = jacobian(h_o_sym, l);
H_s = matlabFunction(H_s_sym, 'Vars', {s, l}); 
H_l = matlabFunction(H_l_sym, 'Vars', {s, l}); 
H = matlabFunction([H_s_sym, H_l_sym], 'Vars', {s, l}); 

% Jacobian H
G_s_fun = jacobian(g_sym, s); 
G_l_fun = jacobian(g_sym, y_l); 
G_s = matlabFunction(G_s_fun, 'Vars', {s, y_l}); 
G_l = matlabFunction(G_l_fun, 'Vars', {s, y_l}); 

% C_NB as function
C_NB_fun = matlabFunction(C_NB, 'Vars', {Lambda}); 

% Pdot Propagation function
Pdot_ss = @(t, P, F, Q) F * P + P * transpose(F) + Q; % Covariance matrix (P_ss)
Pdot_sm = @(t, P, F) F * P; % Covariance matrix (P_sm)

save("Extras\State_functions", "f_a", "f_s", "F", "A", "B",...
    "h_o", "gk", "H_s", "H_l", "H", "Pdot_ss", "Pdot_sm", ...
    "G_q", "G_s", "G_l", "C_NB_fun"); 