function [xhat, Phat, Hi] = update_known(xhat, Phat, Lo, h_o, H_s, R_zeta)

n_s = 9;            % Quad Estimates Positions
shat = xhat(1:n_s); % Quad Estimates

y_o = Lo.y_o(:); % Vecotorize Measurements
n_o = Lo.n_o;
N_L = Lo.N_L;
Hi = NaN(numel(y_o), size(xhat,1));
ho_k = NaN(size(y_o));
R_zeta_k = zeros(numel(y_o), numel(y_o));

%% Iterative Update
% if ~isempty(N_L)
%     for k = 1:n_o
%         loc = (3 * k - 2) : (3 * k);
%         shat = xhat(1:9);
%         l = N_L(:, k);
%         Hi = H_s(shat, l); 
%         E = R_zeta + (Hi * Phat * Hi');
%         K = Phat * Hi' / E;
%         ho_k = h_o(shat, l); 
%         Z = y_o(loc) - ho_k;
%         xhat = xhat + K * Z;
%         Phat = Phat - K * E * K';
%     end
% end

%% Lump Update
if ~isempty(N_L)
    for k = 1:n_o
        l = N_L(:, k); % Known Lmks
        loc = (3 * k - 2) : (3 * k);
        Hi(loc, :) = H_s(shat, l); 
        ho_k(loc, :) = h_o(shat, l); 
        R_zeta_k(loc, loc) = R_zeta;
    end
E = R_zeta_k + (Hi * Phat * Hi');
K = Phat * Hi' / E;
Z = y_o - ho_k;
xhat = xhat + K * Z;
Phat = Phat - K * E * K';
end