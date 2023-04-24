function u = Trajectory_Controller(r_d, u_d, r, A, B)

%% Solving Algebric Ricatti Equation
K = Trajectory_LQR_K(A, B);

%% Control input
r_tilde = r - r_d;
u_tilde = - K * r_tilde;
u = u_d + u_tilde;