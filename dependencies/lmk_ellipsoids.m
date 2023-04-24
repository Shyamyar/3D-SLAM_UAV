function ell = lmk_ellipsoids(mhat_his, sigma3_m_his)
lsize = size(mhat_his.Lids, 1);
for l = 1:lsize
    elpn = mhat_his.N_L(1,l);
    elpe = mhat_his.N_L(2,l);
    elpd = mhat_his.N_L(3,l);
    elpn_err = sigma3_m_his(1,l)';
    elpe_err = sigma3_m_his(2,l)';
    elpd_err = sigma3_m_his(3,l)';
    [X,Y,Z] = ellipsoid(elpe, elpn, -elpd,...
        elpe_err, elpn_err, -elpd_err);
    ell(l,1) = surf(X, Y, Z , 'FaceAlpha', 0.5, 'EdgeAlpha', 0,...
        'FaceColor', 'k', 'DisplayName', 'Confidence Ellipsoid');
end
