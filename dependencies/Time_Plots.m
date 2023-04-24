%% Time Plots vs # Landmarks
fig_tm = figure();
nlmk = (5:198)';
t_n_mf = fillmissing(t_n_m(nlmk)', 'linear');
tnm_fit = fit(nlmk, t_n_mf, "poly2");
plot(tnm_fit, nlmk, t_n_mf)
grid on
ylabel("Time (sec)")
xlabel("$n_m$", 'Interpreter', 'latex')
xlim([5,200])
legend hide
set(gca, 'fontname', 'times' , "FontSize", 14)
legend("Values", "Trendline")
exportgraphics(fig_tm, fig_path + "_tnm.png", 'Resolution', 300)

fig_tu = figure();
nlmk = (5:65)';
t_n_kf = fillmissing(t_n_k(nlmk)', 'linear');
tnk_fit = fit(nlmk, t_n_kf, "poly2");
plot(tnk_fit, nlmk, t_n_kf)
grid on
ylabel("Time (sec)")
xlabel("$n_{\kappa}$", 'Interpreter', 'latex')
xlim([5,65])
legend hide
set(gca, 'fontname', 'times' , "FontSize", 14)
legend("Values", "Trendline")
exportgraphics(fig_tu, fig_path + "_tnk.png", 'Resolution', 300)

fig_tk = figure();
nlmk = (5:22)';
t_n_uf = fillmissing(t_n_u(nlmk)', 'linear');
tnu_fit = fit(nlmk, t_n_uf, "poly1");
plot(tnu_fit, nlmk, t_n_uf)
grid on
ylabel("Time (sec)")
xlabel("$n_{\upsilon}$", 'Interpreter', 'latex')
xlim([5,22])
legend hide
set(gca, 'fontname', 'times' , "FontSize", 14)
legend("Values", "Trendline")
exportgraphics(fig_tk, fig_path + "_tnu.png", 'Resolution', 300)
