function Lo = observed_lmks(L_ids, B_L, alpha_lim, beta_lim, delta_lim)
    
n_L = size(B_L, 2);            % Number of Landmarks
[B_Lp, beta_xy] = cart2sph_shyam(B_L);

check = NaN(3, n_L); 
check(1, :) = alpha_lim(1) < B_Lp(1, :) & B_Lp(1, :) < alpha_lim(2);
check(2, :) = beta_lim(1) < beta_xy & beta_xy < beta_lim(2);
check(3, :) = B_Lp(3, :) <= delta_lim;

Lo.O = L_ids(sum(check == ones(3, 1)) == 3); % Observed Landmark Ids
Lo.y_o = B_Lp(:, Lo.O);       % Observed Landmarks LiDAR Measurements
Lo.n_o = size(Lo.O, 2);      % Number of Observed Landmarks