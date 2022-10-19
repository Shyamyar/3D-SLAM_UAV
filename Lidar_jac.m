syms xL yL zL x y z u v w phi theta psi azm elev range

L_in = [xL; yL; zL];                        % Landmark Position Inertial Cartesian
r_pos = [x; y; z];                          % Robot Position Inertial Cartesian
L_b_polar = [azm;elev;range];               % Landmark Position Body Polar (Measurement)
C_BI = C(3,psi) * C(2,theta) * C(1,phi);    % Sensor Orientation

H_x_sym = cart2sph_shyam(transpose(C_BI) * (L_in - r_pos)); % Observation Model
G_x_sym = C_BI * sph2cart_shyam(L_b_polar) + r_pos;         % Inverse Observation Model

states_ref = [x; y; z; u; v; w; psi];
oth1 = [xL; yL; zL; phi; theta];
states_tot = [x; y; z; u; v; w; phi; theta; psi];
oth2 = [azm; elev; range; phi; theta];

H_x = matlabFunction(H_x_sym,'Vars',{states_ref, oth1});
G_x = matlabFunction(G_x_sym,'Vars',{states_ref, oth2});

H_k = jacobian(H_x_sym,states_ref);
H_k = matlabFunction(H_k,'Vars',{states_ref,oth1});
H_L = jacobian(H_x_sym,L_in);
H_L = matlabFunction(H_L,'Vars',{states_ref,oth1});

G_k = jacobian(G_x_sym,states_ref);
G_k = matlabFunction(G_k,'Vars',{states_ref,oth2});
G_L = jacobian(G_x_sym,L_b_polar);
G_L = matlabFunction(G_L,'Vars',{states_ref,oth2});

save('Lidar_jac','H_x','G_x','H_k','H_L','G_k','G_L');
