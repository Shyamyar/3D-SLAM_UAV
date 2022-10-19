function [xhat,Phat] = CDEKF_tot(tn,xhat,Phat,U_ekf,L_sense,dt,prop_it)
global xhat_dot F_k H_x H_k H_L Q R landmarks_t r
global g
[az,pz,qz,rz,phiz,thetaz] = deal(U_ekf(1),U_ekf(2),U_ekf(3),U_ekf(4),U_ekf(5),U_ekf(6));
h = dt/prop_it;

%% For Covariance Propagation
Pdot = @(t,P,F) F * P + P * transpose(F) + Q;     % Propagation for covariance matrix

%% Update
nil_t = find(landmarks_t(1,:)~=0); % total INITIALIZED LANDMARKS
nil_s = intersect(L_sense.L_id,nil_t); % total INITIALIZED LANDMARKS in L_sense landmarks
loc_nil_s = find(ismember(L_sense.L_id,nil_t));
Ym = L_sense.L_b_polar_err(:,loc_nil_s);% Measurements of L_sense initialized measurements
m = landmarks_t(landmarks_t~=0)'; % all pointers to landmarks
rm = [r , m]; 

% Sort Measured Landmarks s.t. nearing landmark updates latest
% [range_sort,I] = sort(Ym(3,:),'descend');
% Ym = Ym(:,I);
% L_in = L_sense.L_in(:,I);

count = 1;
for k = nil_s
    l = landmarks_t(:, k)';
    L_in = xhat(l);
%     L_in = L_sense.L_in(:,count);
    oth = [L_in;phiz;thetaz];
    Hk = H_k(xhat(r),oth); % From Lidar_Jac
    HL = H_L(xhat(r),oth);
    H = [Hk,HL];
    rl=[r,l];
    E = R + (H * Phat(rl,rl) * H');
    K = Phat(rm,rl) * H' / E;
    Hx = H_x(xhat(r),oth); % From Lidar_Jac
    Z = Ym(:,count) - Hx;
    xhat(rm) = xhat(rm) + K * Z;
    Phat(rm,rm) = Phat(rm,rm) - K * E * K';
    count = count + 1;
end

%% Propagation
tpropx = tn;
tpropP = tn;
for i = 1:prop_it
    [tpropx,xhat(r)] = RK4_xdot(xhat_dot,tpropx,xhat(r),U_ekf,g,h);
    F  = F_k(tpropP,xhat(r),U_ekf,g);
    [tpropP,Phat(r,r)] = RK4_CDEKF(Pdot,tpropP,Phat(r,r),F,h);
end

end