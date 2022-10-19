function [xhat,Phat] = CDEKF_Shyam(tn,xhat,Phat,U_ekf,L_sense,dt,prop_it)
global xhat_dot F_k H_x H_k H_L Q R
global g
[az,pz,qz,rz,phiz,thetaz] = deal(U_ekf(1),U_ekf(2),U_ekf(3),U_ekf(4),U_ekf(5),U_ekf(6));
h = dt/prop_it;
n = size(xhat,1);

%% For Covariance Propagation
Pdot = @(t,P,F) F * P + P * transpose(F) + Q;     % Propagation for covariance matrix

%% Update
Ym = L_sense.L_b_polar_err;
L_in = L_sense.L_in;

% Sort Measured Landmarks s.t. nearing landmark updates latest
[range_sort,I] = sort(Ym(3,:),'descend');
% Ym = Ym(:,I);
% L_in = L_sense.L_in(:,I);

for k = 1:length(I)
    oth = [L_in(:,k);phiz;thetaz];
    Hk = H_k(xhat,oth); % From Lidar_Jac
    Hx = H_x(xhat,oth); % From Lidar_Jac
    E = (R + (Hk * Phat * Hk'));
    K = Phat * Hk' / E;
    Z = Ym(:,k) - Hx;
    xhat = xhat + K * Z;
    Phat = (eye(n) - K * Hk) * Phat;
end

%% Propagation
tpropx = tn;
tpropP = tn;
for i = 1:prop_it
    [tpropx,xhat] = RK4_xdot(xhat_dot,tpropx,xhat,U_ekf,g,h);
    F  = F_k(tpropP,xhat,U_ekf,g);
    [tpropP,Phat] = RK4_CDEKF(Pdot,tpropP,Phat,F,h);
end

end