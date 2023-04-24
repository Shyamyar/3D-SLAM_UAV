function tau = Attitude_Controller(sys, Lambda_c, omega_c, Lambda, omega)
    
%% States
phi = Lambda(1);
theta = Lambda(2);
psi = Lambda(3);
p = omega(1);
q = omega(2);
r = omega(3);
Jx = sys(3);
Jy = sys(4);

%% Commanded States
phi_c = Lambda_c(1);
theta_c = Lambda_c(2);
psi_c = Lambda_c(3);
p_c = omega_c(1);
q_c = omega_c(2);
r_c = omega_c(3);

%% Tau Control
lqr_true = false;
if lqr_true
    %% LQR Controller
    Ka = Attitude_LQR_K(sys, Lambda);
    e_a = [phi - phi_c; p - p_c;...
           theta - theta_c; q - q_c;...
           psi - psi_c; r - r_c];
    tau = - Ka * e_a;
else
    %% PD Gains
    zeta_roll = 0.707;
    a_phi3 = 1 / Jx;
    phi_max = 30 * (pi/180);
    tau_phi_max = 2;
    wn_phi = sqrt(a_phi3 * tau_phi_max * sqrt(1 - zeta_roll^2) / phi_max);
    Kp_phi = wn_phi^2 / a_phi3;
    Kd_phi = 1.1 * (2 * zeta_roll * wn_phi) / a_phi3;
    
    zeta_pitch = 0.707;
    a_theta3 = 1 / Jy;
    theta_max = 30 * (pi/180);
    tau_theta_max = 2;
    wn_pitch = sqrt(a_theta3 * tau_theta_max * sqrt(1 - zeta_pitch^2) / theta_max);
    Kp_theta = wn_pitch^2 / a_theta3;
    Kd_theta = 1.1 * (2 * zeta_pitch * wn_pitch) / a_theta3;
    
    zeta_yaw = 0.707;
    a_phi3 = 1 / Jy;
    phi_max = 30 * (pi/180);
    tau_phi_max = 1;
    wn_yaw = sqrt(a_phi3 * tau_phi_max * sqrt(1 - zeta_yaw^2) / phi_max);
    Kp_psi = wn_yaw^2 / a_phi3;
    Kd_psi = 1.1 * (2 * zeta_yaw * wn_yaw) / a_phi3;
    
    %% PD Controller
    tau_phi = pd_control(phi, phi_c, p , p_c, Kp_phi, Kd_phi);
    tau_theta = pd_control(theta, theta_c, q , q_c, Kp_theta, Kd_theta);
    tau_psi = pd_control(psi, psi_c, r , r_c, Kp_psi, Kd_psi);
    
    tau = [tau_phi; tau_theta; tau_psi];
end