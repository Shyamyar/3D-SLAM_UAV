function [t,y] = RK4_xdot(func,t,y,u,g,h)

k1 = func(t, y, u, g);
k2 = func(t + h/2, y + h * k1/2, u, g);
k3 = func(t + h/2, y + h* k2/2, u, g);
k4 = func(t, y + h*k3, u, g);

y = y + h * (k1 + 2 * (k2 + k3) + k4) / 6;
t = t + h;
