function fitness = GetFitness(genomes, ~, distances)
    numGenes = size(genomes, 2);
    numGenomes = size(genomes, 1);
    fitness = zeros(numGenomes, 1);
    for i = 1:numGenomes
        distanceSum = distances(1, numGenes);
        for j = 1:numGenes - 1
            genePair = genomes(i, j:j + 1);
            distanceSum = distanceSum + distances(genePair(1), genePair(2));
        end
        fitness(i) = -distanceSum;
    end
end
