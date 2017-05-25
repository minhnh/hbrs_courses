function VisualizeFoil( individual, nacafoil, nacaParam, figNum )
%VISUALIZEFOIL Summary of this function goes here
%   Detailed explanation goes here
[foil, nurbs] = pts2ind(individual, size(nacafoil, 2));
fig = figure(figNum);
plot(nacafoil(1, :), nacafoil(2, :), 'LineWidth', 3);
hold on;
plot(foil(1, :), foil(2, :), 'r', 'LineWidth', 3);
plot(nurbs.coefs(1, 1:int16(end/2)), nurbs.coefs(2, 1:int16(end/2)), 'rx', 'LineWidth', 3);
axis equal;
axis([0 1 -0.7 0.7]);
legend(['NACA ' mat2str(nacaParam) ' target'], 'Approximated Shape');
ax = gca;
ax.FontSize = 24;
drawnow;
hold off;
Visualization.save_figure(fig, ['naca_coil_solution_' num2str(figNum)], 24);
end

