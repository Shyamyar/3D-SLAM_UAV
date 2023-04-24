function [n_L, L_ids, N_L, N_Lp] = landmarks_polar(n_L, alpha_L_lim, beta_L_lim, delta_L_lim)

%% Spherical Landmarks in Given Limits
alpha_L = linspace(alpha_L_lim(1), alpha_L_lim(2), n_L);
beta_L = linspace(beta_L_lim(1), beta_L_lim(2), n_L);
delta_L = linspace(delta_L_lim(1), delta_L_lim(2), n_L);

L_ids = 1:n_L;
N_Lp = [alpha_L(randperm(length(alpha_L))); 
        beta_L(randperm(length(beta_L))); 
        delta_L(randperm(length(delta_L)))]; 
N_L = sph2cart_shyam(N_Lp); 

% %% Straight Incremental Line Landmarks
% N_L = [];
% for i = 0:3:40
%     pn_L = normrnd(0, 2, 1, i+1);
%     pe_L = repmat(i, [1, i+1]);
%     pd_L = normrnd(0, 2, 1, i+1);
%     N_L = [N_L, [pn_L; pe_L; pd_L]];
% end
% n_L = size(N_L, 2);
% L_ids = 1:n_L;
% N_Lp = cart2sph_shyam(N_L);
