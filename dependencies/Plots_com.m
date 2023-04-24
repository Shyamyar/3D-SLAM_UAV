function Plots_com(var, t, units, leg_name, var_col, style, tag, sp)

range = 1:length(t);
a_size = size(var, 1);

c = 1;
j = 1;
while j < a_size
    plt = subplot(sp(1), sp(2), c);
    for i = 1:3
        plot(t, var(j, range), 'Color', var_col(i,:), ...
            'LineStyle', style, 'DisplayName', leg_name(j), 'Tag', tag);
        hold on
        grid on
        j = j + 1;
    end
    if contains(tag, 'Desired')
        h = findobj(plt, 'Type', 'Line', 'Tag', 'True');
        legend(h, 'Interpreter', 'Latex');
    end
    xlabel('Time (sec)')
    ylabel(units(c), 'Interpreter', 'Latex')
    set(gca, 'fontname', 'times', 'fontsize', 16)
    c = c + 1;
end