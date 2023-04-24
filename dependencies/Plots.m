function Plots(var, t, style, ylabel_text, sp)

range = 1:length(t);
a_size = size(var, 1);

for j = 1:a_size
    subplot(sp(1), sp(2), j)
    plot(t, var(j, range), style)
    hold on
    grid on
%     xlabel('Time (sec)')
    ylabel(ylabel_text(j), 'Interpreter', 'Latex')
    set(gca, 'fontname', 'times', 'fontsize', 12)
end