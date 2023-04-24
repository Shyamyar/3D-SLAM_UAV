function [L_ids,L_polar] = landmarks_polar(n_L,r_L_lim,th_L_lim,phi_L_lim)

% Polar Coordinates of Landmarks
L_ids = 1:n_L;
radius_l = r_L_lim(1) + (r_L_lim(2)-r_L_lim(1)).*rand(1,n_L);
az_l = th_L_lim(1) + (th_L_lim(2)-th_L_lim(1)).*rand(1,n_L);
elev_l = phi_L_lim(1) + (phi_L_lim(2)-phi_L_lim(1)).*rand(1,n_L);
L_polar = [az_l;elev_l;radius_l];