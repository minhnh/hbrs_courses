function VisualizeGraph(adjacency, colorSolution, kcolors, figNum, graphTitle, fileName)
    colors = jet(kcolors);
    % Create graph. As an adjacency matrix for an undirected graph is symmetrical, we can ignore the lower triangle.
    g = graph(adjacency,'upper');

    % draw graph
    fig = figure(figNum);
    h = plot(g, 'MarkerSize', 12, 'LineWidth', 3);
    for i = 1:kcolors
        highlight(h, find(colorSolution==i), 'NodeColor', colors(i, :));
    end
    title(graphTitle);
    ax = gca;
    ax.XTick = []; ax.YTick = [];
    Visualization.save_figure(fig, ['figure_', fileName], 18); 

end

