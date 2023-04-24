function Plots_error(var, t, style, ylabel_text, sp)

range = 1:length(t);
rangel = 50:length(t)-50;
var_size = size(var, 1);

for j = 1:var_size
    subplot(sp(1), sp(2), j)
    plot(t, var(j, range), style)
    hold on
    grid on
%     xlabel('Time (sec)')
    ylabel(ylabel_text(j), 'Interpreter', 'Latex')
    set(gca, 'fontname', 'times' , "FontSize", 12)
    if all(~isnan(var(j, rangel))) && ~isempty(rangel)
        max_ylim = 1.2 * max(abs(var(j, rangel)));
        xlim([0, t(end)])
        ylim([-max_ylim, max_ylim])
    end
end