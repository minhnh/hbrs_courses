%% BinaryEncoding.m
% Class structure for various evolutionary algorithm problems using
% binary encoding
% Author:   Minh Nguyen
% Date: 2017-04-14
%%
classdef BinaryEncoding

    properties
        MIN_ARG_NUM = 7;
        Population
        BitLength
        TargetFitness
        funcGetFitness
        funcSelectWinners
        funcCrossover
        funcMutate
    end

    methods
        function obj = BinaryEncoding(size, maxInitValue, bitLength,...
                                      getFitness, selectWinners, crossover,...
                                      mutate, targetFitness)
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
            obj.funcCrossover = crossover;
            obj.funcMutate = mutate;
            disp(['Initialized population with ' num2str(size) ' members:']);
            obj.DisplayPopulation();
        end

        function selected = Select(obj, elitism)
            selection_size = length(obj.Population);
            selected = [];
            if elitism
                [~, maxArg] = max(obj.funcGetFitness(obj.Population,...
                                                     obj.TargetFitness));
                selected = [selected; obj.Population(maxArg, :)];
                selection_size = selection_size - 1;
            end

            winners = obj.funcSelectWinners(obj, selection_size);
            selected = [selected; winners];
        end

        function children = GetChildren(obj, selectedParents, crossoverRate)
            children = zeros(size(obj.Population));
            for k = 1:length(obj.Population)                
                parentIndices = randperm(length(selectedParents), 2);
                children(k, :) = obj.funcCrossover(...
                        obj, obj.Population(parentIndices, :), crossoverRate);
            end
        end

        function Iterate(obj, iterNum, elitism, crossoverRate, mutationRate)
            for i = 1:iterNum
                selectedParents = obj.Select(elitism);
                children = obj.GetChildren(selectedParents, crossoverRate);
                for k = 1:length(children)
                    obj.Population(k, :) = obj.funcMutate(obj, children(k, :),...
                                                          mutationRate);
                end
                obj.DisplayPopulation();
            end
        end

        function DisplayPopulation(obj)
            disp(obj.Population);
            fitness = obj.funcGetFitness(obj.Population, obj.TargetFitness);
            disp(['Best fitness:    ' num2str(max(fitness))]);
            disp(['Average fitness: ' num2str(mean(fitness))]);
        end
    end
end

function population = InitPopulation(size, maxInit, bitLength)
    initNums = randi(maxInit, 1, size);
    population = de2bi(initNums, bitLength, 'left-msb');
end


