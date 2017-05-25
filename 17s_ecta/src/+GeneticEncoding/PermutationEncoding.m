 %% PermutationEncoding.m
% Class structure for various evolutionary algorithm problems using
% permutation encoding
% Author:   Minh Nguyen
% Date: 2017-04-14
%%
classdef PermutationEncoding < matlab.mixin.SetGet
    properties
        Population
        NumGene
        Target
        Constraints
        Verbose
        funcInitPopulation
        funcGetFitness
        funcSelectWinners
        funcSingleCrossover
        funcMutate
        funcCheckConvergence
    end

    methods
        function obj = PermutationEncoding(populationSize, numGene, target,...
                                           constraints, initPopulation,...
                                           getFitness, selectWinners, crossover,...
                                           mutate, checkConvergence, verbose)
            % Set properties
            obj.Target = target;
            obj.Verbose = verbose;
            obj.NumGene = numGene;
            obj.Constraints = constraints;
            obj.funcInitPopulation = initPopulation;
            obj.funcGetFitness = getFitness;
            obj.funcSelectWinners = selectWinners;
            obj.funcSingleCrossover = crossover;
            obj.funcMutate = mutate;
            obj.funcCheckConvergence = checkConvergence;

            % Initialize population
            obj.funcInitPopulation(obj, populationSize, numGene, constraints);
            if obj.Verbose
                disp(['Initialized population with ' num2str(populationSize) ' members:']);
                obj.DisplayPopulation();
            end
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
%             disp(selectedParents);
            for k = 1:numCrossover
                parentIndices = randperm(numParents, 2);
                children(k, :) = obj.funcSingleCrossover(...
                                     selectedParents(parentIndices, :));
            end
            for k = 1:(numParents - numCrossover)
                children(numCrossover + k, :) = selectedParents(randi(numParents), :);
            end
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
%                 disp(selectedParents)
                children = obj.Crossover(selectedParents, crossoverRate);
                children = obj.funcMutate(children, mutationRate);
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

        function bestChild = GetBestChild(obj)
            fitness = obj.funcGetFitness(obj.Population, obj.Target,...
                                         obj.Constraints);
            [~, argMax] = max(fitness);
            bestChild = obj.Population(argMax, :);
        end

        function argMax = DisplayPopulation(obj)
%             disp(obj.Population);
            fitness = obj.funcGetFitness(obj.Population, obj.Target,...
                                         obj.Constraints);
            [~, argMax] = max(fitness);
            disp(['Best fitness:    ' num2str(fitness(argMax))]);
            disp(['Average fitness: ' num2str(mean(fitness))]);
        end
    end
    
end

