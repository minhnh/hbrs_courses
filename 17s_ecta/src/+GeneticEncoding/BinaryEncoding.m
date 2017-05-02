%% BinaryEncoding.m
% Class structure for various evolutionary algorithm problems using
% binary encoding
% Author:   Minh Nguyen
% Date: 2017-04-14
%%
classdef BinaryEncoding < matlab.mixin.SetGet

    properties
        MIN_ARG_NUM = 7;
        Population
        BitLength
        Target
        Constraints
        Verbose
        funcGetFitness
        funcSelectWinners
        funcSingleCrossover
        funcCheckConvergence
    end

    methods
        function obj = BinaryEncoding(size, maxInitValue, bitLength, constraints,...
                                      getFitness, selectWinners, crossover,...
                                      checkConvergence, target, verbose)
            if nargin < obj.MIN_ARG_NUM
                error(['GenerationBinary constructor requires at least '...
                       num2str(obj.MIN_ARG_NUM) ' arguments']);
            elseif nargin > obj.MIN_ARG_NUM + 1
                obj.Verbose = verbose;
                obj.Target = target;
            elseif nargin > obj.MIN_ARG_NUM
                obj.Target = target;
                obj.Verbose = true;
            else
                % default target is all bits are 1's
                obj.Target = ones(1, bitLength);
                obj.Verbose = true;
            end

            obj.InitPopulation(size, maxInitValue, bitLength);
            obj.BitLength = bitLength;
            obj.Constraints = constraints;
            obj.funcGetFitness = getFitness;
            obj.funcSelectWinners = selectWinners;
            obj.funcSingleCrossover = crossover;
            obj.funcCheckConvergence = checkConvergence;
            if obj.Verbose
                disp(['Initialized population with ' num2str(size) ' members:']);
                obj.DisplayPopulation();
            end
        end

        function InitPopulation(obj, size, maxInit, bitLength)
            initNums = randi(maxInit, 1, size);
            population = de2bi(initNums, bitLength, 'left-msb');
            set(obj, 'Population', population);
        end

        function winners = Select(obj, elitism)
            selection_size = size(obj.Population, 1);
            if elitism
                selection_size = selection_size - 1;
            end
            winners = obj.funcSelectWinners(obj, selection_size);
        end

        function children = Crossover(obj, selectedParents, crossoverRate)
            children = zeros(size(selectedParents));
            numParents = size(selectedParents, 1);
            numCrossover = int32(crossoverRate * numParents);
            for k = 1:numCrossover
                parentIndices = randperm(numParents, 2);
                children(k, :) = obj.funcSingleCrossover(...
                        obj, selectedParents(parentIndices, :));
            end
            for k = 1:(numParents - numCrossover)
                children(numCrossover + k, :) = selectedParents(randi(numParents), :);
            end
        end

        function children = Mutate(~, children, mutationRate)
            mutationMatrix = rand(size(children)) < mutationRate;
            children = xor(children, mutationMatrix);
        end

        function [bestFitness, medianFitness, minFitness] =...
                    Iterate(obj, iterNum, elitism, crossoverRate, mutationRate)
            bestFitness = zeros(1, iterNum);
            medianFitness = zeros(1, iterNum);
            minFitness = zeros(1, iterNum);
            for i = 1:iterNum
                % record fitness
                fitness = obj.funcGetFitness(obj.Population, obj.Target,...
                                             obj.Constraints);
                bestFitness(i) = max(fitness);
                medianFitness(i) = median(fitness);
                minFitness(i) = min(fitness);

                % check convergence
                if obj.funcCheckConvergence(obj)
                    if obj.Verbose
                        disp(['converged after ' int2str(i) ' iterations']);
                    end
                    break
                end

                % evolve
                selectedParents = obj.Select(elitism);
                children = obj.Crossover(selectedParents, crossoverRate);
                children = obj.Mutate(children, mutationRate);
                if elitism
                    fitness = obj.funcGetFitness(obj.Population, obj.Target,...
                                                 obj.Constraints);
                    [~, argMax] = max(fitness);
                    obj.Population(size(obj.Population, 1), :)...
                                = obj.Population(argMax, :);
                end
                obj.Population(1:size(children, 1), :) = children;

                % print result
                if obj.Verbose
                    obj.DisplayPopulation();
                end
            end

            % record last fitness
            fitness = obj.funcGetFitness(obj.Population, obj.Target,...
                                         obj.Constraints);
            bestFitness = [bestFitness, max(fitness)];
            medianFitness = [medianFitness, median(fitness)];
            minFitness = [minFitness, min(fitness)];
        end

        function argMax = DisplayPopulation(obj)
%             disp(obj.Population);
            fitness = obj.funcGetFitness(obj.Population, obj.Target,...
                                         obj.Constraints);
            [~, argMax] = max(fitness);
            disp(['Best fitness:    ' num2str(fitness(argMax))]);
            disp(['Average fitness: ' num2str(mean(fitness))]);
        end

        function bestChild = GetBestChild(obj)
            fitness = obj.funcGetFitness(obj.Population, obj.Target,...
                                         obj.Constraints);
            [~, argMax] = max(fitness);
            bestChild = obj.Population(argMax, :);
        end
    end
end
