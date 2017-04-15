%% OneMax.m
% Solution for the OneMax problem
% Author:   Minh Nguyen
% Date: 2017-04-15
%%
POPULATION_SIZE = 10;
MAX_INIT_VALUE = 6;
BIT_LENGTH = 8;
oneMax =  BinaryEncoding(POPULATION_SIZE, MAX_INIT_VALUE, BIT_LENGTH,...
                         @GetFitness, @SelectWinners, @Crossover, @Mutate);

function fitness = GetFitness(gene, ~)
    % fitness is number of 1's
    fitness = sum(gene == 1, 2);
end

function winners = SelectWinners(obj, selection_size)
    winners = zeros(selection_size, obj.BitLength);
    for i = 1:selection_size
        parentIndices = randperm(length(obj.Population), 2);
        parents = obj.Population(parentIndices, :);
        [~, maxArg] = max(obj.funcGetFitness(parents, obj.TargetFitness));
        winners(i, :) = parents(maxArg, :);
%         disp(parents);
%         disp(winners(i, :));
    end
end

function child = Crossover(obj, parents, rate)
    numBitChange = int16(obj.BitLength * rate + 0.5);
    child = [parents(1, 1 : numBitChange),...
             parents(2, numBitChange + 1 : obj.BitLength)];
end

function child = Mutate(obj, child, rate)
    numBitChange = int16(obj.BitLength * rate + 0.5);
    mutatedBits = randperm(obj.BitLength, numBitChange);
%     display(child);
%     display(mutatedBits);
    child(mutatedBits) = 1 - child(mutatedBits);
%     display(child);
end