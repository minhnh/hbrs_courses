function fitness = GetFitness(genomes, targetFunction, ~)
%GETFITNESS Fitness function for function minimization problem
    fitness = -targetFunction(genomes');
end

