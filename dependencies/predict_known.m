function [xhat, Phat, Fi] = predict_known(ti, dt, prop_it, sys, xhat, ...
    Phat, u_s, f_s, F, G_q, R_imu, Q, Pdot_ss)

h = dt / prop_it;
shat = xhat(1:9);

%% Propagation / Prediction xhat
Fi  = F(ti, shat, u_s, sys);
% Gq = G_q(ti, shat, u_s, sys);
% Q = Gq * R_imu * Gq' + Q;
[~, shat_ode] = ode45(f_s, ti:h:ti+dt, shat, [], u_s, sys);
xhat(1:9) = shat_ode(end, :)';

%% Propagation / Prediction Phat
for i = 1:prop_it
    [ti, Phat] = RK4_Phat_ss(Pdot_ss, ti, Phat, Fi, Q, h);
end