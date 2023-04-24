%% State Plots (True Vs Estimated Vs Reference States)
fig1 = figure('Visible', vis);
ch = 300;
fig1.Position = [680-ch   558-ch   560+ch   420+ch];
% sgtitle('True Vs Estimated Vs Desired States');
ylabel_text = ["$p_{n}$ (m)", "$p_{e}$ (m)", "$p_{d}$ (m)", "$u$ (m/s)",...
    "$v$ (m/s)", "$w$ (m/s)", "$\phi$ (rad)", "$\theta$ (rad)", ...
    "$\psi$ (rad)", "$p$ (rad/s)", "$q$ (rad/s)", "$r$ (rad/s)"];
sp_size = [6 2];

Plots(a_his, tcompx, "b-", ylabel_text, sp_size)
Plots(ahat_his, tcompx, "k--", ylabel_text, sp_size)
Plots(a_d_his, tcompx, "r-.", ylabel_text, sp_size)
leg1 = legend("True", "Estimated", "Desired");
set(leg1, 'FontName', 'times', 'FontSize', 12);
leg1.Location = 'bestoutside';

% exportgraphics(fig1, fig_path + "_states.png", 'Resolution', 300)

%% State Plots Separate (True Vs Estimated Vs Reference States)
for var = 1:4
fig1a = figure('Visible', vis);
% sgtitle('True Vs Estimated Vs Desired States');
ylabel_text = ["$p_{n}$ (m)", "$p_{e}$ (m)", "$p_{d}$ (m)", "$u$ (m/s)",...
    "$v$ (m/s)", "$w$ (m/s)", "$\phi$ (rad)", "$\theta$ (rad)", ...
    "$\psi$ (rad)", "$p$ (rad/s)", "$q$ (rad/s)", "$r$ (rad/s)"];
vars = varstr(var).val; 
sp_size = size(vars');

Plots(a_his(vars, :), tcompx, "b-", ylabel_text(vars), sp_size)
Plots(ahat_his(vars, :), tcompx, "k--", ylabel_text(vars), sp_size)
Plots(a_d_his(vars, :), tcompx, "r-.", ylabel_text(vars), sp_size)
xlabel('Time (sec)')
% leg1a = legend("True", "Estimated", "Desired");
% set(leg1a, 'FontName', 'times', "FontSize", 12);

% exportgraphics(fig1a, fig_path + varstr(var).name + ".png", 'Resolution', 300)
close(fig1a);
end

%% State Plots Combined (True Vs Estimated Vs Reference States)
fig1b = figure('Visible', vis);
ch = 300;
fig1b.Position = [680-ch   558-ch   560+ch   420+ch];
% sgtitle('True Vs Estimated Vs Desired States');
leg_name = ["$p_{n}$", "$p_{e}$", "$h$", "$u$",...
    "$v$", "$w$", "$\phi$", "$\theta$", ...
    "$\psi$", "$p$", "$q$", "$r$"];
units = ["m", "m/s", "rad", "rad/s"];
tag = ["True", "Estimated", "Desired"];
style = ["-", "--" ,"-."];
var_col = [0 0.4470 0.7410;
            0.8500 0.3250 0.0980;
            0.9290 0.6940 0.1250];
sp_size = [2 2];

Plots_com(a_his, tcompx, units, leg_name, var_col, style(1), tag(1), sp_size);
Plots_com(ahat_his, tcompx, units, leg_name, var_col, style(2), tag(2), sp_size);
Plots_com(a_d_his, tcompx, units, leg_name, var_col, style(3), tag(3), sp_size);
ah1 = axes('position', get(gca,'position'), 'visible', 'off');
hold on;
plt(1) = plot(tcompx, a_his(end,:), "k"+style(1),...
    'LineWidth', 1.2, 'Visible', 'off');
plt(2) = plot(tcompx, ahat_his(end,:), "k"+style(2),...
    'LineWidth', 1.2, 'Visible', 'off');
plt(3) = plot(tcompx, a_d_his(end,:), "k"+style(3),...
    'LineWidth', 1.2, 'Visible', 'off');

leg1b = legend(ah1, plt, tag);
set(leg1b,...
    'Position',[0.7 0.4 0.1 0],...
    'FontName', 'times', 'FontSize', 12);

% exportgraphics(fig1b, fig_path + "_states_com.png", 'Resolution', 300)

%% Estimation Error Plots over time for Quadcopter
fig2 = figure('Visible', vis);
ch = 300;
fig2.Position = [680-ch   558-ch   560+ch   420+ch];
% sgtitle('Estimation Error for Quadcopter States');
ylabel_text = ["$e_{pn}$ (m)", "$e_{pe}$ (m)", "$e_{pd}$ (m)", "$e_u$ (m/s)",...
    "$e_v$ (m/s)", "$e_w$ (m/s)", "$e_\phi$ (rad)", "$e_\theta$ (rad)",...
    "$e_\psi$ (rad)"];
sp_size = [3 3];

Plots_error(err_s_his, tcompx, "k-", ylabel_text, sp_size)
Plots_error(-sigma3_s_his, tcompx, "r-.", ylabel_text, sp_size)
Plots_error(sigma3_s_his, tcompx, "r-.", ylabel_text, sp_size)
% leg2 = legend("Estimation Error", "3sigma Boundary");
% set(leg2, 'FontName', 'times', 'FontSize', 12);

% exportgraphics(fig2, fig_path + "_error.png", 'Resolution', 300)

%% Estimation Error Plots Separate over time for Quadcopter
for var = 1:3
fig2a = figure('Visible', vis);
% sgtitle('True Vs Estimated Vs Desired States');
ylabel_text = ["$e_{pn}$ (m)", "$e_{pe}$ (m)", "$e_{pd}$ (m)", "$e_u$ (m/s)",...
    "$e_v$ (m/s)", "$e_w$ (m/s)", "$e_\phi$ (rad)", "$e_\theta$ (rad)",...
    "$e_\psi$ (rad)"];
vars = varstr(var).val; 
sp_size = size(vars');

Plots_error(err_s_his(vars, :), tcompx, "k-", ylabel_text(vars), sp_size)
Plots_error(-sigma3_s_his(vars, :), tcompx, "r-.", ylabel_text(vars), sp_size)
Plots_error(sigma3_s_his(vars, :), tcompx, "r-.", ylabel_text(vars), sp_size)
xlabel('Time (sec)')
% leg2a = legend("EstimationError", "3sigma Boundary");
% set(leg2a, 'FontName', 'times', "FontSize", 12);

% exportgraphics(fig2a, fig_path + varstr(var).name + "_err.png", 'Resolution', 300)
close(fig2a);

rmse = sqrt(mean(err_s_his(vars, 50:end).^2, 2));
avg_rmse = mean(rmse);
rmse_disp = "RMSE (%) for " + ylabel_text(vars)' + " = " + rmse;
avg_rmse_disp = "Avg RMSE (%) for " + varstr(var).name + " = " + avg_rmse;
disp(rmse_disp)
disp(avg_rmse_disp)
end

%% Estimation Error Plots over time for Landmarks
fig3 = figure('Visible', vis);
% sgtitle('Estimation Error for Landmark States');
ylabel_text = ["$e_{pn}$ (m)", "$e_{pe}$ (m)", "$e_{pd}$ (m)"];
sp_size = [3 1];

for k = 1:n_L
    if  k <= n_L
        err_m_k = reshape(err_m_his(:,k,:),[3, tsteps]);
        sigma_m_k = reshape(sigma3_m_his(:,k,:),[3, tsteps]);
    else
        err_m_k = [reshape(err_m_his(:,k,:),[3, tsteps]);
                    err_m_mean];
        sigma_m_k = [reshape(sigma3_m_his(:,k,:),[3, tsteps]); ...
                       sigma3_m_mean];
        ylabel_text = [ylabel_text, "Mean " + ylabel_text];
    end
    Plots_error(err_m_k , tcompx, "k-", ylabel_text, sp_size)
    Plots_error(-sigma_m_k, tcompx, "r-.", ylabel_text,sp_size)
    Plots_error(sigma_m_k, tcompx, "r-.", ylabel_text, sp_size)
end
xlabel('Time (sec)')
% leg3 = legend("Estimation Error", "3sigma Boundary");
% set(leg3, 'FontName', 'times', "FontSize", 12);

% exportgraphics(fig3, fig_path + "_lmk_error.png", 'Resolution', 300)

rmse = sqrt(mean(err_m_his(:, :, :).^2,[2,3],'omitnan'));
avg_rmse = mean(rmse);
rmse_disp = "RMSE (%) for " + ylabel_text(1:3)' + " = " + rmse;
avg_rmse_disp = "Avg RMSE (%) for " + "Landmark Position" + " = " + avg_rmse;
disp(rmse_disp)
disp(avg_rmse_disp)

%% Estimation Error for each Landmark: Animation till the end 
fig3a = figure('Visible', vis);
% sgtitle('Estimation Error for Landmarks at the end of Simulation');
ylabel_text = ["$e_{pn}$ (m)", "$e_{pe}$ (m)", "$e_{pd}$ (m)"];

for i = 1:10:tsteps
    for k = 1:3
        subplot(3, 1, k)
        grid on
        err_m_ti = err_m_his(k, :, i);
        sigma3_m_ti = sigma3_m_his(k, :, i);
        e_bar(k) = errorbar(mhat_his(i).Lids, err_m_ti(~isnan(err_m_ti)), ...
            sigma3_m_ti(~isnan(sigma3_m_ti)), 'ko');
        hold on
        mean_bar(k) = plot(mhat_his(i).Lids, repmat(err_m_mean(k, i), ...
            [1, length(mhat_his(i).Lids)]), 'r');
        ylabel(ylabel_text(k), "Interpreter", "latex")
        set(gca, 'FontName', 'times', 'FontSize', 12)
    end
    drawnow limitrate
    if i < tsteps
        delete([e_bar, mean_bar])
    end
end
xlabel("Landmark IDs")

leg4a = legend("Estimation Error", "Mean Estimation Error");
set(leg4a, 'Position', [0.20 0.94 0.64 0.052],...
    'Orientation', 'horizontal',...
    'FontName', 'times', "FontSize", 12);

% exportgraphics(fig3a, fig_path + "_end_error.png", 'Resolution', 300)

%% Animation when Error
fig4 = figure('Visible', vis);
ch = 200;
fig4.Position = [680-ch   558-ch   560+ch   420+ch];
tplot = find(ismember(tcomp, tcompx)); 
tseq = tplot(1:50*round(log10(sampling_rate), 0):end); 
% tseq = tplot(end);
Animation_all(tseq, a_his, a_d_his, ahat_his, Lids, N_L, Lo_his, ...
    mhat_his, sigma3_m_his, alpha_lim, beta_lim, delta_lim, vid_path)

exportgraphics(fig4, fig_path + ".png", 'Resolution', 300)

%% Dock Figures
if vis == "on"
    set([fig1, fig1b, fig2, fig3, fig4], 'WindowStyle', 'Docked')
end
