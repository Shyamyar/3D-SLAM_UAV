function [t, y] = RK4_Phat_ss(func, t, y, F, Q, h)

k1 = func(t, y, F, Q);
k2 = func(t + h/2, y + h * k1/2, F, Q);
k3 = func(t + h/2, y + h * k2/2, F, Q);
k4 = func(t, y + h * k3, F, Q);

y = y + h * (k1 + 2 * (k2 + k3) + k4) / 6;
t = t + h;
