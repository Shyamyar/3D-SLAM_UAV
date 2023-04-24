function Animation_all(tseq, a_his, a_d_his, ahat_his, Lids, N_L, Lo_his,...
    mhat_his, sigma3_m_his, alpha_lim, beta_lim, delta_lim, vid_path)

VIDEO = false;
if VIDEO
    video = VideoWriter(vid_path, 'Motion JPEG AVI');
    video.FrameRate = 30;
    video.Quality = 95;
    open(video);
end

% Plot Landmarks
scatter3(N_L(2, :), N_L(1, :), -N_L(3, :), 50, 'bo',...
    'DisplayName', 'True Landmarks'); % True Landmarks
xlabel('North', "FontName", "times")
ylabel('East', "FontName", "times")
zlabel('Up', "FontName", "times")
axis('equal')
view(-37.5, 30)
hold on
grid on
xlim([-10, 10])
ylim([-10, 10])
zlim([-10, 10])
rotate3d on
if size(N_L, 2) <= 30
    text(N_L(2, :), N_L(1, :), -N_L(3, :), string(Lids));
end

for i = tseq
    % Quad 1 Estimated
    quad1 = Draw_Quad(ahat_his(7, i), -ahat_his(8, i), ...
        pi/2 - ahat_his(9, i), ahat_his(2, i), ahat_his(1, i),...
        - ahat_his(3, i), 'k--', 0.3);
    quad1_traj = plot3(ahat_his(2, 1:i), ahat_his(1, 1:i), ...
        -(ahat_his(3, 1:i)), '--', 'DisplayName',...
        'Estimated Trajectory');
    quad1_traj.Color = 'k'; % Estimated Trajectory

    % Quad 2 True
    quad2 = Draw_Quad(a_his(7, i), -a_his(8, i), ...
        pi/2 - a_his(9, i), a_his(2, i), a_his(1, i),...
        -a_his(3, i), 'b-', 1); 
    quad2_traj = plot3(a_his(2, 1:i), a_his(1, 1:i), ...
        -(a_his(3, 1:i)), '-',  'DisplayName',...
        'True Trajectory');
    quad2_traj.Color = 'b'; % True Trajectory
    
    % Quad Desired Trajectory
    traj = plot3(a_d_his(2, 1:i), a_d_his(1, 1:i), -(a_d_his(3, 1:i)),...
        '-.',  'DisplayName', 'Desired Trajectory');
    traj.Color = 'r'; % Desired Trajectory

    % Observed Landmarks
    L_o_lmks = scatter3(Lo_his(i).N_L(2, :), Lo_his(i).N_L(1, :), ...
        -Lo_his(i).N_L(3, :), 50, 'm*', 'DisplayName', 'Observed Landmarks');

    % Estimated Landmarks
    est_lmks = scatter3(mhat_his(i).N_L(2, :), mhat_his(i).N_L(1, :), ...
        -mhat_his(i).N_L(3, :), 50, 'k+', 'DisplayName', 'Estimated Landmarks');
    est_lmks_ell = [];
%     est_lmks_ell = lmk_ellipsoids(mhat_his(i), sigma3_m_his(:, :, i));

    % FOV Generation
    [X, Y, Z] = fov_3D_rec(alpha_lim(2), beta_lim(2), delta_lim);
    M = makehgtform('translate', [a_his([2, 1], i); -a_his(3, i)], ...
        'zrotate', pi/2 - a_his(9, i), 'yrotate', -a_his(8, i), ...
        'xrotate', a_his(7, i));
    fov = surf(X, Y, Z, 'Parent', hgtransform('Matrix', M), ...
        'LineStyle', 'none', 'FaceAlpha', 0.1);
    
    % Legend
    leg = [];
    h = findobj(gca,{'DisplayName', 'True Landmarks', '-or',...
                    'DisplayName','Observed Landmarks', '-or', ...
                    'DisplayName','Estimated Landmarks', '-or', ...
                    'DisplayName', 'True Trajectory', '-or', ...
                    'DisplayName','Estimated Trajectory', '-or', ...
                    'DisplayName', 'Desired Trajectory', '-or', ...
                    'DisplayName', 'Confidence Ellipsoid'});
    
    if isempty(est_lmks_ell)
        leg = legend(h(end-5:end), 'FontName', 'times', "FontSize", 12);
    else
        leg = legend(h(end-6:end), 'FontName', 'times', "FontSize", 12);
    end
    leg.Location = 'northwest';
    set(gca, 'fontname', 'times', 'fontsize', 12)

    % Creating animation
    drawnow limitrate;
    if VIDEO
        frame = getframe(gcf);
        writeVideo(video, frame);
    end

    handles2del = [quad1; quad1_traj; quad2; quad2_traj; ...
        traj; L_o_lmks; est_lmks; est_lmks_ell; fov; leg];
    if i<tseq(end)
        delete(handles2del);
    else
        delete(quad1);
    end

end