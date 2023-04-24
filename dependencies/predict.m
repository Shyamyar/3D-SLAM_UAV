function [xhat, Phat, Fi] = predict(ti, dt, prop_it, sys, xhat, Phat, ...
    u_s, f_s, F, G_q, R_imu, Q, Pdot_ss, Pdot_sm)

h = dt / prop_it;
n_x = size(xhat,1);
n_s = 9;                % No. of Quad Estimates Positions
ss = 1:n_s;             % Quad Estimates Positions
mm = (n_s + 1) : n_x;   % Lmk Estimates Positions
shat = xhat(ss);
Phat_ss = Phat(ss, ss);
Phat_sm = Phat(ss, mm);
Phat_mm = Phat(mm, mm);

%% Propagation / Prediction xhat
Fi  = F(ti, shat, u_s, sys);
% Gq = G_q(ti, shat, u_s, sys);
% Q = Gq * R_imu * Gq' + Q;
[~, shat_ode] = ode45(f_s, ti:h:ti+dt, shat, [], u_s, sys);
xhat(ss) = shat_ode(end, :)';

%% Propagation / Prediction Phat
ts = ti;
for i = 1:prop_it
    [ts, Phat_ss] = RK4_Phat_ss(Pdot_ss, ts, Phat_ss, Fi, Q, h);
    [ti, Phat_sm] = RK4_Phat_sm(Pdot_sm, ti, Phat_sm, Fi, h);
end
Phat_ms = Phat_sm';
Phat = [Phat_ss, Phat_sm;
        Phat_ms, Phat_mm];