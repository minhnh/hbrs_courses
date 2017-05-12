function fitness = GetFitness(genomes, ~, constraints)
    numGenes = size(genomes, 2);
    if numGenes ~= size(constraints{2}, 1)
        error('number of nodes and number of genes is not equal')
    end
    numGenomes = size(genomes, 1);
    fitness = zeros(numGenomes, 1);
    for i = 1:numGenomes
        penalty = 0;
        for node = 1:numGenes
            for neigh = find(constraints{2}(node, :) == 1)
                if genomes(i, node) == genomes(i, neigh)
                    penalty = penalty + 1;
                end
            end
        end
        fitness(i) = -penalty;
    end
end
