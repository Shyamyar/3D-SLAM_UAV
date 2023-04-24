function ydes = trajectory_funs(traj, pos0)
%% Creating Trajectory Functions
syms ti

pn0 = pos0(1); 
pe0 = pos0(2); 
pd0 = pos0(3); 
psi0 = pos0(4); 

if traj == "8"
    a = 5; 
    b = 3; 
    c = 1; 
    T = 25; 
    w1 = 2 * pi / T; 
    w2 = 0.5 * w1; 
    w3 = 1 * w1; 
    pn = pn0 + a * cos(w2 * ti); 
    pe = pe0 + b * sin(w1 * ti); 
    pd = pn0 + c * sin(w3 * ti); 
    psi = psi0 + w1 * ti; 

elseif traj == "spiral" || traj == "s"
    a = 5; 
    b = 5; 
    c = 0; 
    n = -0.01 * ti; 
    T = 50; 
    w1 = 2 * pi / T; 
    w2 = 1 * w1; 
    w3 = 1 * w1; 
    pn = pn0 + a * cos(w2 * ti); 
    pe = pe0 + b * sin(w1 * ti); 
    pd = pn0 + n + c * sin(w3 * ti); 
    psi = psi0 + w1 * ti; 

elseif traj == "circle" || traj == "c"
    a = 5; 
    b = 5; 
    c = 0; 
    T = 50; 
    w1 = 2 * pi / T; 
    w2 = w1; 
    w3 = w1; 
    pn = pn0 + a * cos(w2 * ti); 
    pe = pe0 + b * sin(w1 * ti); 
    pd = pd0 + c * sin(w3 * ti); 
    psi = psi0 + w1 * ti; 

elseif traj == "straight" || traj == "st"
    vn = 0; 
    ve = 0.1; 
    vd = 0; 
    T = 50; 
    w1 = 0; 
    pn = pn0 + vn * ti; 
    pe = pe0 + ve * ti; 
    pd = pd0 + vd * ti; 
    psi = psi0 + w1 * ti; 

end

rho_sym = [pn; pe; pd]; 
ydes.rho_d = matlabFunction(rho_sym, 'Vars', {ti}); 
rho_dot_sym = diff(rho_sym, ti); 
ydes.rho_dot_d = matlabFunction(rho_dot_sym, 'Vars', {ti}); 
rho_ddot_sym = diff(rho_dot_sym, ti); 
ydes.rho_ddot_d = matlabFunction(rho_ddot_sym, 'Vars', {ti}); 
psi_sym = psi; 
ydes.psi_d = matlabFunction(psi_sym, 'Vars', {ti}); 
psi_dot_sym = diff(psi_sym, ti); 
ydes.psi_dot_d = matlabFunction(psi_dot_sym, 'Vars', {ti}); 