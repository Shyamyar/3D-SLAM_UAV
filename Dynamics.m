function statesdot=Dynamics(t,states,U)
global Jx Jy Jz g m

[Tphi,Ttheta,Tpsi,F] = deal(U(1),U(2),U(3),U(4));
pn = states(1,1);
pe = states(2,1);
pd = states(3,1);
u = states(4,1);
v = states(5,1);
w = states(6,1);
phi = states(7,1);
theta = states(8,1);
psi = states(9,1);
p = states(10,1);
q = states(11,1);
r = states(12,1);

% States 1,2,3 [pn_dot,pe_dot,pd_dot]
RB2N = C(3,psi) * C(2,theta) * C(1,phi); %DCM to convert from body to inertial
pos_dot = RB2N * [u;v;w];

% States 4,5,6 [u_dot,v_dot,w_dot]
coriolis_term = [(r*v-q*w);(p*w-r*u);(q*u-p*v)];
Thrustvec = [0;0;-F/m];
Gravityvec = transpose(RB2N) * [0;0;g]; % In Vehicle Frame
Velrate = coriolis_term + Thrustvec + Gravityvec;

% States 7,8,9 [phi_dot,theta_dot,psi_dot]
turnrate_tf = turnrate_BI(phi,theta);
GyroVec = [p;q;r];
Attrate = turnrate_tf * GyroVec;

% States 10,11,12 [p_dot,q_dot,r_dot]
Vec2 = [Tphi/Jx;Ttheta/Jy;Tpsi/Jz];
Vec3 = [q*r*(Jy-Jz)/Jx;p*r*(Jz-Jx)/Jy;q*p*(Jx-Jy)/Jz];
Gyrorate = Vec2 + Vec3;

statesdot = [pos_dot;Velrate;Attrate;Gyrorate];
end