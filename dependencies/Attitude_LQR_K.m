function K = Attitude_LQR_K(sys, Lambda)

%% States
Jx = sys(3);
Jy = sys(4);
Jz = sys(5);
phi = Lambda(1);
theta = Lambda(2);

%% Linearized Matrices
A = [   0, 1, 0, phi * theta, 0,  theta;
        0, 0, 0,           0, 0,      0;
        0, 0, 0,           1, 0, -theta;
        0, 0, 0,           0, 0,      0;
        0, 0, 0,         phi, 0,      1;
        0, 0, 0,           0, 0,      0];

B = [  0,      0,    0;
    1/Jx,      0,    0;
       0,      0,    0;
       0,   1/Jy,    0;
       0,      0,    0;
       0,      0, 1/Jz];

%% Q Tuning for Error
q1 = 0.37;
q2 = 2.8;
q3 = 0.32;
q4 = 2.5;
q5 = 0.35;
q6 = 2.5;
Q = [   1/q1^2, 0, 0, 0, 0, 0;
        0, 1/q2^2, 0, 0, 0, 0
        0, 0, 1/q3^2, 0, 0, 0;
        0, 0, 0, 1/q4^2, 0, 0;
        0, 0, 0, 0, 1/q5^2, 0;
        0, 0, 0, 0, 0, 1/q6^2];

%% R Tuning for Control
r1 = 1;
r2 = 1;
r3 = 0.3;
R = [   1/r1^2, 0, 0;
        0, 1/r2^2, 0;
        0, 0, 1/r3^2];

[X, K, L] = icare(A, B, Q, R, [], [], []);