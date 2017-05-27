function SetupPlot(plotTitle, xLabel, yLabel, fontSize, legendValues)
    %SETUPPLOT setup current figure with some basic settings
    set(gca, 'fontsize', fontSize);
    grid();
    title(plotTitle);
    xlabel(xLabel);
    ylabel(yLabel);

    if ~isempty(legendValues)
        legends = cell(1, length(legendValues));
        for i = 1:length(legendValues)
            legends{i} = num2str(legendValues(i));
        end
        legend(legends{:}, 'Location', 'southeast');
    end
end

