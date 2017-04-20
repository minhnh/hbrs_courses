%% OneMax.m
% Solution for the OneMax problem
% Author:   Minh Nguyen
% Date: 2017-04-15
%%
POPULATION_SIZE = 10;
MAX_INIT_VALUE = 6;
BIT_LENGTH = 8;
oneMax =  GeneticEncoding.BinaryEncoding(...
                            POPULATION_SIZE, MAX_INIT_VALUE, BIT_LENGTH,...
                            @GetFitness, @SelectWinners, @SingleCrossover,...
                            @SingleMutate, @CheckConvergence, ones(1, BIT_LENGTH));

function fitness = GetFitness(gene, target)
    difference = xor(gene, target);
    fitness = sum(difference == 0, 2);
end

function winners = SelectWinners(obj, selection_size)
    winners = zeros(selection_size, obj.BitLength);
    for i = 1:selection_size
        parentIndices = randperm(length(obj.Population), 2);
        parents = obj.Population(parentIndices, :);
        [~, maxArg] = max(obj.funcGetFitness(parents, obj.Target));
        winners(i, :) = parents(maxArg, :);
    end
end

function child = SingleCrossover(obj, parents)
    midPoint = int16(obj.BitLength / 2 + 0.5);
    child = [parents(1, 1 : midPoint), parents(2, midPoint + 1 : obj.BitLength)];
end

function child = SingleMutate(obj, child)
    changedBit = randi(obj.BitLength);
    child(changedBit) = 1 - child(changedBit);
end

function converging = CheckConvergence(obj)
    converging = false;
    fitness = obj.funcGetFitness(obj.Population, obj.Target);
    targetFitness = obj.BitLength;
    if abs(max(fitness) - targetFitness) < 1
        converging = true;
    end
end