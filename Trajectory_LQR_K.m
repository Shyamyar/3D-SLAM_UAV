function K=Trajectory_LQR_K()
A = [0,0,0,1,0,0,0;
    0,0,0,0,1,0,0;
    0,0,0,0,0,1,0;
    0,0,0,0,0,0,0;
    0,0,0,0,0,0,0;
    0,0,0,0,0,0,0;
    0,0,0,0,0,0,0];

B = [0,0,0,0;
    0,0,0,0;
    0,0,0,0;
    1,0,0,0;
    0,1,0,0;
    0,0,1,0;
    0,0,0,1];

q1 = 0.03;
q2 = 0.03;
q3 = 0.03;
q4 = 0.03;
q5 = 0.03;
q6 = 1;
q7 = 0.01;

Q = [1/q1^2,0,0,0,0,0,0;
    0,1/q2^2,0,0,0,0,0;
    0,0,1/q3^2,0,0,0,0;
    0,0,0,1/q4^2,0,0,0;
    0,0,0,0,1/q5^2,0,0;
    0,0,0,0,0,1/q6^2,0;
    0,0,0,0,0,0,1/q7^2];

r1=0.1;
r2=0.1;
r3=0.1;
r4=0.1;
R = [1/r1^2,0,0,0;
    0,1/r2^2,0,0;
    0,0,1/r3^2,0;
    0,0,0,1/r4^2];
[X,K,L] = icare(A,B,Q,R,[],[],[]);

end