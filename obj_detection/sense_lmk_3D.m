function [L_id_return,L_b_polar_return] = sense_lmk_3D(L_ids,L_in,r_pose,r_lim,az_lim,elev_lim)
% The function identifies landmarks in a given sensor zone specified by
% it's spherical coordinate limits
% L_in is Landmark Map vector with 3D coordinates of Landmanrks in inertial
% frame
% r_lim is range limit, th_lim is azimuthal limit, phi_lim is elevation
% limit
% of the range, bearing sensor like Lidar
% L_id stores the ids of Landmarks that are in the sensor zone
% r_pose is the position and orientation of robot in inertial frame
    
n_L = length(L_in)/3;   % Number of Landmarks
L_in = reshape(L_in,[3,n_L]);  % Convert the Lamdmark Vectorspace into Column Mapspace

r_pos = r_pose(1:3);                 % Robot Inertial Position
r_or = r_pose(4:6);                  % Robot Orientation in 321 rotation
[phi,theta,psi] = deal(r_or(1),r_or(2),r_or(3));

C_BI = C(3,psi) * C(2,theta) * C(1,phi);    % 3-2-1 Rotation matrix for Body to Inertial
L_b = C_BI' * (L_in - r_pos);               % Landmark Positions in Body Frame cartesian

check = NaN(3,n_L);
[L_b_polar,elev_xy] = cart2sph_shyam(L_b);
check(1,:) = az_lim(1)<L_b_polar(1,:) & L_b_polar(1,:)<az_lim(2);
check(2,:) = elev_lim(1)<elev_xy & elev_xy<elev_lim(2);
check(3,:) = L_b_polar(3,:)<=r_lim;

L_id_return = L_ids(sum(check==ones(3,1))==3);

L_b_polar_return = L_b_polar(:,L_id_return);