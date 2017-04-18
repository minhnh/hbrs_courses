%% OneMax.m
% Solution for the OneMax problem
% Author:   Minh Nguyen
% Date: 2017-04-15
%%
POPULATION_SIZE = 10;
MAX_INIT_VALUE = 6;
BIT_LENGTH = 8;
oneMax =  BinaryEncoding(POPULATION_SIZE, MAX_INIT_VALUE, BIT_LENGTH,...
                         @GetFitness, @SelectWinners, @SingleCrossover,...
                         @SingleMutate, @CheckConvergence, 2^BIT_LENGTH - 1);

function fitness = GetFitness(gene)
    fitness = bi2de(gene, 'left-msb');
end

function winners = SelectWinners(obj, selection_size)
    winners = zeros(selection_size, obj.BitLength);
    for i = 1:selection_size
        parentIndices = randperm(length(obj.Population), 2);
        parents = obj.Population(parentIndices, :);
        [~, minArg] = min(abs(obj.funcGetFitness(parents) - obj.TargetFitness));
        winners(i, :) = parents(minArg, :);
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
    fitness = obj.funcGetFitness(obj.Population);
    if sum(abs(fitness - obj.TargetFitness)) < 1
        converging = true;
    end
end