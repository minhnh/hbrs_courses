%% BinaryEncoding.m
% Class structure for various evolutionary algorithm problems using
% binary encoding
% Author:   Minh Nguyen
% Date: 2017-04-14
%%
classdef BinaryEncoding

    properties
        MIN_ARG_NUM = 8;
        Population
        BitLength
        Target
        funcGetFitness
        funcSelectWinners
        funcSingleCrossover
        funcSingleMutate
        funcCheckConvergence
    end

    methods
        function obj = BinaryEncoding(size, maxInitValue, bitLength,...
                                      getFitness, selectWinners, crossover,...
                                      singleMutate, checkConvergence, target)
            if nargin < obj.MIN_ARG_NUM
                error(['GenerationBinary constructor requires at least '...
                       num2str(obj.MIN_ARG_NUM) ' arguments']);
            elseif nargin > obj.MIN_ARG_NUM
                obj.Target = target;
            else
                % default target is all bits are 1's
                obj.Target = ones(1, bitLength);
            end

            obj.Population = InitPopulation(size, maxInitValue, bitLength);
            obj.BitLength = bitLength;
            obj.funcGetFitness = getFitness;
            obj.funcSelectWinners = selectWinners;
            obj.funcSingleCrossover = crossover;
            obj.funcSingleMutate = singleMutate;
            obj.funcCheckConvergence = checkConvergence;
            disp(['Initialized population with ' num2str(size) ' members:']);
            obj.DisplayPopulation();
        end

        function winners = Select(obj, elitism)
            selection_size = length(obj.Population);
            if elitism
                selection_size = selection_size - 1;
            end
            winners = obj.funcSelectWinners(obj, selection_size);
        end

        function children = Crossover(obj, selectedParents, crossoverRate)
            children = zeros(size(selectedParents));
            numParents = length(selectedParents);
            numCrossover = int32(crossoverRate * numParents);
            for k = 1:numCrossover
                parentIndices = randperm(numParents, 2);
                children(k, :) = obj.funcSingleCrossover(...
                        obj, selectedParents(parentIndices, :));
            end
            for k = 1:(numParents - numCrossover)
                children(numCrossover + k, :) = selectedParents(randi(numParents));
            end
        end

        function children = Mutate(obj, children, mutationRate)
            for k = 1:length(children)
                if rand() < mutationRate
                    children(k, :) = obj.funcSingleMutate(obj, children(k, :));
                end
            end
        end

        function numIteration = Iterate(obj, iterNum, elitism, crossoverRate,...
                                        mutationRate)
            for i = 1:iterNum
                if obj.funcCheckConvergence(obj)
                    disp(['converged after ' int2str(i) ' iterations'])
                    break
                end
                selectedParents = obj.Select(elitism);
                children = obj.Crossover(selectedParents, crossoverRate);
                children = obj.Mutate(children, mutationRate);
                if elitism
                    fitness = obj.funcGetFitness(obj.Population, obj.Target);
                    [~, argMax] = max(fitness);
                    children = [children; obj.Population(argMax, :)];
                end
                obj.Population = children;
                obj.DisplayPopulation();
            end
            numIteration = i;
        end

        function DisplayPopulation(obj)
            disp(obj.Population);
            fitness = obj.funcGetFitness(obj.Population, obj.Target);
            [~, argMax] = max(fitness);
            disp(['Best fitness:    ' num2str(fitness(argMax))]);
            disp(['Average fitness: ' num2str(mean(fitness))]);
        end
    end
end

function population = InitPopulation(size, maxInit, bitLength)
    initNums = randi(maxInit, 1, size);
    population = de2bi(initNums, bitLength, 'left-msb');
end


