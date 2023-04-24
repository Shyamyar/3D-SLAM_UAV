function [t, y] = RK4_Phat_sm(func, t, y, F, h)

k1 = func(t, y, F);
k2 = func(t + h/2, y + h * k1/2, F);
k3 = func(t + h/2, y + h * k2/2, F);
k4 = func(t, y + h * k3, F);

y = y + h * (k1 + 2 * (k2 + k3) + k4) / 6;
t = t + h;
