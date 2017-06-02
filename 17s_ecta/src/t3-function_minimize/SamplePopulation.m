function [population, invSqrtC] = SamplePopulation(mean, sigma, covariance,...
                                                   nacafoil, populationSize)
%SAMPLEPOPULATION sample a number of individuals around a normal distribution
    population = repmat(mean', populationSize, 1);
    numGene = length(mean);
    covariance = triu(covariance) + triu(covariance, 1)';   % enforce symmetry
    [B, D] = eig(covariance);                               % eigen decomposition, B==normalized eigenvectors
    D = sqrt(diag(D));                                      % D is a vector of standard deviations now
    if ~isreal(D)
%         disp(['eigen value complex' mat2str(imag(D))]);
        D = real(D);
    end
    invSqrtC = B * diag(D .^ -1) * B';
    % sample from the normal distribution with given covariance
    parfor i = 1:populationSize
        population(i, :) = population(i, :) + sigma * (B * (D .* randn(numGene, 1)))';
    end
    % sort population by fitness
    fitness = GetFitness(population, nacafoil, -1);
    [~, sortedFitnessIndices] = sort(fitness, 'descend');
    population = population(sortedFitnessIndices, :);
end

