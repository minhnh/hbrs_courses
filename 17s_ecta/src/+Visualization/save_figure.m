function save_figure(figID, name, fontsize)
%SAVE_FIGURE Takes a figure ID, a name (string) and font size
% Needs the save2pdf script which is included
% ! Needs pdfcrop to be installed on the system (only tested for linux)
    fig = figure(figID);
    
    % Formatting
    set(fig, 'Units', 'Centimeters');
    for c=1:length(fig.Children)
        if isa(fig.Children(c),'matlab.graphics.axis.Axes')
            fig.Children(c).FontSize = fontsize;
            fig.Children(c).XLabel.FontSize = fontsize;
            fig.Children(c).YLabel.FontSize = fontsize;
        end
    end
    fig.CurrentAxes.FontSize = fontsize;
    fig.CurrentAxes.XLabel.FontSize = fontsize;
    fig.CurrentAxes.YLabel.FontSize = fontsize;
    grid on;
    
    % Save FIG and export to PDF
    savefig([name '.fig']);
    Visualization.save2pdf([name '.pdf'],fig,600,[12 12]);  % 600 dpi, 12x12 inches paper size
    
    % Crops the PDF, gets rid of borders. Only works if pdfcrop is
    % installed
    system(['pdfcrop ' [name '.pdf'] ' ' [name '.pdf']]);

end

