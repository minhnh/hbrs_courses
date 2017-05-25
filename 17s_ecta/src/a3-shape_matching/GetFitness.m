function fitness = GetFitness(genomes, nacafoil, ~)
%GETFITNESS Fitness function for shape matching problem

numGenome = size(genomes, 1);
fitness = zeros(numGenome, 1);
parfor i = 1:numGenome
    % points as the NACA profile
    [foil, ~] = pts2ind(genomes(i, :)', size(nacafoil, 2));
    % Calculate pairwise error
    [~, errorTop] = dsearchn(nacafoil(:, 1:end/2)', foil(:, 1:end/2)');
    [~, errorBottom] = dsearchn(nacafoil(:, 1+end/2:end)', foil(:, 1+end/2:end)');
    % Total fitness
    fitness(i) = -mean([errorTop.^2; errorBottom.^2]);
end


end

