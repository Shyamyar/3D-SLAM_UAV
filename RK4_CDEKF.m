function [t,y] = RK4_CDEKF(func,t,y,U,h)

k1 = func(t, y, U);
k2 = func(t + h/2, y + h * k1/2, U);
k3 = func(t + h/2, y + h* k2/2, U);
k4 = func(t, y + h*k3, U);

y = y + h * (k1 + 2 * (k2 + k3) + k4) / 6;
t = t + h;
