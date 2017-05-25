% DEMO visualization and exporting fiure
x = [-4:0.1:4];
y1 = sin(x);y2 = sin(x+0.3);


fig = figure(1);
    plot_handle = plot(x,y1,'LineWidth',2);
    hold on;
    plot_handle = plot(x,y2,'LineWidth',2);
    
    ax = gca; % Get current axis object
    axis([-4 4 -1.5 1.5]);
    ax.XTick = [-4:2:4];
    ax.YTick = [-1:1:1];
    
    % Non-format customization of image
    l = legend('Sine','Moved Sine');
    l.Location
    xlabel('Input');
    ylabel('Fitness');
    
% Set formatting, save to Matlab .fig and export to cropped PDF, fontsize of 48 is used here    
save_figure(fig, ['hillclimber_comparison'], 36);
