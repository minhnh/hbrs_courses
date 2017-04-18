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
        TargetFitness
        funcGetFitness
        funcSelectWinners
        funcSingleCrossover
        funcSingleMutate
        funcCheckConvergence
    end

    methods
        function obj = BinaryEncoding(size, maxInitValue, bitLength,...
                                      getFitness, selectWinners, crossover,...
                                      singleMutate, checkConvergence, targetFitness)
            if nargin < obj.MIN_ARG_NUM
                error(['GenerationBinary constructor requires at least '...
                       num2str(obj.MIN_ARG_NUM) ' arguments']);
            elseif nargin > obj.MIN_ARG_NUM
                obj.TargetFitness = targetFitness;
            else
                % default target is all bits are 1's
                obj.TargetFitness = bitLength;
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

        function selected = Select(obj, elitism)
            selection_size = length(obj.Population);
            selected = [];
            if elitism
                [~, minArg] = min(abs(obj.funcGetFitness(obj.Population) -...
                                      obj.TargetFitness));
                selected = [selected; obj.Population(minArg, :)];
                selection_size = selection_size - 1;
            end

            winners = obj.funcSelectWinners(obj, selection_size);
            selected = [selected; winners];
        end

        function children = Crossover(obj, selectedParents, crossoverRate)
            children = zeros(size(obj.Population));
            populationSize = length(obj.Population);
            numCrossover = int32(crossoverRate * populationSize);
            numParents = length(selectedParents);
            for k = 1:numCrossover
                parentIndices = randperm(numParents, 2);
                children(k, :) = obj.funcSingleCrossover(...
                        obj, selectedParents(parentIndices, :));
            end
            for k = 1:(populationSize - numCrossover)
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

        function Iterate(obj, iterNum, elitism, crossoverRate, mutationRate)
            for i = 1:iterNum
                if obj.funcCheckConvergence(obj)
                    disp(['converged after ' int2str(i) ' iterations'])
                    break
                end
                selectedParents = obj.Select(elitism);
                children = obj.Crossover(selectedParents, crossoverRate);
                obj.Population = obj.Mutate(children, mutationRate);
                obj.DisplayPopulation();
            end
        end

        function DisplayPopulation(obj)
            disp(obj.Population);
            fitness = obj.funcGetFitness(obj.Population);
            [~, argMin] = min(abs(fitness - obj.TargetFitness));
            disp(['Best fitness:    ' num2str(fitness(argMin))]);
            disp(['Average fitness: ' num2str(mean(fitness))]);
        end
    end
end

function population = InitPopulation(size, maxInit, bitLength)
    initNums = randi(maxInit, 1, size);
    population = de2bi(initNums, bitLength, 'left-msb');
end


