%% Visualization of cities
function VisualizeCities(coords, cityOrder, figNum)
    figure(figNum);
    orderedCoords = coords(cityOrder, :);
    orderedCoords = [orderedCoords; coords(cityOrder(1), :)];
    plot(orderedCoords(:, 1), orderedCoords(:, 2), '-r*');
end

