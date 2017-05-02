%% Evaluation.m
% Class for evaluating genetic algorithm solutions
% Author:   Minh Nguyen
% Date:     2017-04-20
%%
classdef Evaluation < matlab.mixin.SetGet

    properties
        DefaultPopulationSize
        DefaultConstructorParams
        DefaultIterateParams
        ConvergeFunctionParamIndex
        NumIterParamIndex
        EncodingConstructor
        NumIterOverPopulationResult
        NumIterOverRatesResult
        MaxFitnessOverCrossoverRatesResult
        MedianFitnessOverCrossoverRatesResult
        MinFitnessOverCrossoverRatesResult
        MaxFitnessOverMutationRatesResult
        MedianFitnessOverMutationRatesResult
        MinFitnessOverMutationRatesResult
    end

    methods
        function obj = Evaluation(encodingConstructor, size, constructorParams,...
                                  iterateParams, convergeFuncIndex, numIterIndex)
            obj.EncodingConstructor = encodingConstructor;
            obj.DefaultPopulationSize = size;
            obj.DefaultConstructorParams = constructorParams;
            obj.DefaultIterateParams = iterateParams;
            obj.ConvergeFunctionParamIndex = convergeFuncIndex;
            obj.NumIterParamIndex = numIterIndex;
        end

        function EvalNumIterOverPopulation(obj, populationSizes,...
                                           numRunPerSize)
            % generate random seed for same initial population
            s = rng(randseed());

            numSizes = length(populationSizes);
            results = zeros(1, numSizes);
            for i = 1:numSizes
                % restore random seed
                rng(s);

                numIterations = zeros(1, numRunPerSize);
                for j = 1:numRunPerSize
                    encodingObj = obj.EncodingConstructor(...
                                      populationSizes(i),...
                                      obj.DefaultConstructorParams{:});
                    [bestFitness, ~] = encodingObj.Iterate(...
                                           obj.DefaultIterateParams{:});
                    numIterations(j) = length(bestFitness);
                end
                results(i) = mean(numIterations);
            end

            set(obj, 'NumIterOverPopulationResult', [populationSizes; results]);
        end

        function EvalNumIterOverRates(obj, rates, numRunPerRate,...
                                      rateParamIndex)
            % generate random seed for same initial population
            s = rng(randseed());

            numRates = length(rates);
            results = zeros(1, numRates);
            for i = 1:numRates
                % restore random seed
                rng(s);

                numIterations = zeros(1, numRunPerRate);
                for j = 1:numRunPerRate
                    encodingObj = obj.EncodingConstructor(...
                                      obj.DefaultPopulationSize,...
                                      obj.DefaultConstructorParams{:});
                    params = obj.DefaultIterateParams;
                    params{rateParamIndex} = rates(i);
                    [bestFitness, ~] = encodingObj.Iterate(params{:});
                    numIterations(j) = length(bestFitness);
                end
                results(i) = median(numIterations);
            end

            set(obj, 'NumIterOverRatesResult', [rates; results])
        end

        function EvalFitnessOverRates(obj, rates, iterNum, numRunPerRate,...
                                      rateParamIndex, rateName)
            % generate random seed for same initial population
            s = rng(randseed());
            % change CheckConverge function in constructor params to always
            % returning false
            constructorParams = obj.DefaultConstructorParams;
            constructorParams{obj.ConvergeFunctionParamIndex} = @NeverConverge;
            % change default number of iterations when calling Iterate
            iterateParams = obj.DefaultIterateParams;
            iterateParams{obj.NumIterParamIndex} = iterNum;

            maxResults = zeros(iterNum + 1, length(rates));
            medianResults = zeros(iterNum + 1, length(rates));
            minResults = zeros(iterNum + 1, length(rates));
            for i = 1:length(rates)
                % restore random seed
                rng(s);

                averageMedianFitness = zeros(numRunPerRate, iterNum + 1);
                averageMaxFitness = zeros(numRunPerRate, iterNum + 1);
                averageMinFitness = zeros(numRunPerRate, iterNum + 1);
                for j = 1:numRunPerRate
                    encodingObj = obj.EncodingConstructor(obj.DefaultPopulationSize,...
                                                          constructorParams{:});
                    iterateParams{rateParamIndex} = rates(i);
                    [maxFitness, medianFitness, minFitness] =...
                                  encodingObj.Iterate(iterateParams{:});
                    averageMaxFitness(j, :) = maxFitness;
                    averageMinFitness(j, :) = minFitness;
                    averageMedianFitness(j, :) = medianFitness;
                end
                maxResults(:, i) = mean(averageMaxFitness, 1);
                minResults(:, i) = mean(averageMinFitness, 1);
                medianResults(:, i) = mean(averageMedianFitness, 1);
            end

            set(obj, ['MaxFitnessOver' rateName 'RatesResult'], maxResults);
            set(obj, ['MedianFitnessOver' rateName 'RatesResult'], medianResults);
            set(obj, ['MinFitnessOver' rateName 'RatesResult'], minResults);
        end
    end

    methods(Static)
        function setupPlot(plotTitle, xLabel, yLabel, fontSize)
            set(gca, 'fontsize', fontSize)
            grid();
            title(plotTitle);
            xlabel(xLabel);
            ylabel(yLabel);
        end

        function lgd = SetupLegends(legendValues)
            legends = cell(1, length(legendValues));
            for i = 1:length(legendValues)
                legends{i} = num2str(legendValues(i));
            end
            lgd = legend(legends{:}, 'Location', 'southeast');
        end
    end

end

function converging = NeverConverge(~, ~)
    converging = false;
end
