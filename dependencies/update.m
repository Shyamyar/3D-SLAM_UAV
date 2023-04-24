function [xhat, Phat, Hi] = update(xhat, Phat, Lo, h_o, H_s, H_l, R_zeta)

n_x = size(xhat,1);
n_s = 9;                % No. of Quad Estimates Positions
ss = 1:n_s;             % Quad Estimates Positions
mm = (n_s + 1) : n_x;   % Lmk Estimates Positions
sm = [ss, mm];
shat = xhat(ss);        % Quad Estimates

y_k = Lo.y_k(:);            % Vectorize Mapped Observations
n_k = Lo.n_k;               % No. of Mapped Observed Lmks
kappa = Lo.kappa;           % Lids of Mapped Observed Lmks
mk = Lo.Lhat_loc_k(:)';       % Position of Kappa in mhat
smk = [ss, mk];             % Position of Quad States and Kappa
mhat_k = xhat(mk);          % mhat components of kappa
mhat_reshaped_k = reshape(mhat_k, [3, n_k]); % Reshape mhat_k into individual columns of kappa
N_L_k = Lo.N_L(:, Lo.ik);

%% Iterative Update
% if ~isempty(kappa)
%     for k = 1:n_k
%         shat = xhat(1:9);
%         loc = (3 * k - 2) : (3 * k);
%         l = Lids_hat_k(loc)';
%         sl = [ss, l];
%         lhat = mhat_reshaped_k(:, k); % Estimated Mapped Observed Lmks
%         Hs_i = H_s(shat, lhat); 
%         Hl_i = H_l(shat, lhat);
%         Hi = [Hs_i, Hl_i];
%         E = R_zeta + (Hi * Phat(sl, sl) * Hi');
%         K = Phat(sm, sl) * Hi' / E;
%         ho_k = h_o(shat, lhat); 
%         Z = y_k(loc) - ho_k;
%         xhat = xhat + K * Z;
%         Phat = Phat - K * E * K';
%     end
% end

%% Lump Update
Phat_sm_smk = Phat(sm, smk);
Phat_smk_smk = Phat(smk, smk);

Hs_i = NaN(numel(y_k), size(shat,1));
Hl_i = zeros(numel(y_k), numel(y_k));
Hi = [Hs_i, Hl_i];
ho_k = NaN(size(y_k));
R_zeta_k = zeros(numel(y_k), numel(y_k));
if ~isempty(kappa)
    for k = 1:n_k
        loc = (3 * k - 2) : (3 * k);
        lhat = mhat_reshaped_k(:, k); % Estimated Mapped Observed Lmks
%         lhat = N_L_k(:, k);           % True Mapped Observed Lmks
        Hs_i(loc, :) = H_s(shat, lhat); 
        Hl_i(loc, loc) = H_l(shat, lhat);
        ho_k(loc, :) = h_o(shat, lhat); 
        R_zeta_k(loc, loc) = R_zeta;
    end
Hi = [Hs_i, Hl_i];
E = R_zeta_k + (Hi * Phat_smk_smk * Hi');
K = Phat_sm_smk * Hi' / E;
Z = y_k - ho_k;
xhat = xhat + K * Z;
Phat = Phat - K * E * K';
end
