%% Evaluation.m
% Class for evaluating gnetic algorithm solutions
% Author:   Minh Nguyen
% Date:     2017-04-20
%%
classdef Evaluation

    properties
        DefaultPopulationSize
        DefaultConstructorParams
        DefaultIterateParams
        ConvergeFunctionParamIndex
        NumIterParamIndex
        EncodingConstructor
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
                                           numRunPerSize, figureNum)
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

            figure(figureNum);
            plot(populationSizes, results, '--rs');
            setupPlot('Average iteration number over population size',...
                      'Population size', 'Number of iterations', 18);
        end

        function EvalNumIterOverRates(obj, rates, numRunPerRate,...
                                      rateParamIndex, rateName, figureNum)
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
                results(i) = mean(numIterations);
            end

            figure(figureNum);
            plot(rates, results, '--rs');
            setupPlot(['Average iteration number over ' rateName ' rates'],...
                      [rateName ' rates'], 'Number of iterations', 18);
        end

        function EvalFitnessOverRates(obj, rates, iterNum, numRunPerRate,...
                                      rateParamIndex, rateName, figureNum)
            % generate random seed for same initial population
            s = rng(randseed());
            % change CheckConverge function in constructor params to always
            % returning false
            constructorParams = obj.DefaultConstructorParams;
            constructorParams{obj.ConvergeFunctionParamIndex} = @NeverConverge;
            % change default number of iterations when calling Iterate
            iterateParams = obj.DefaultIterateParams;
            iterateParams{obj.NumIterParamIndex} = iterNum;

            results = zeros(iterNum + 1, length(rates));
            for i = 1:length(rates)
                % restore random seed
                rng(s);

                averageAverageFitness = zeros(numRunPerRate, iterNum + 1);
                for j = 1:numRunPerRate
                    encodingObj = obj.EncodingConstructor(obj.DefaultPopulationSize,...
                                                     constructorParams{:});
                    initAverageFitness = mean(encodingObj.funcGetFitness(...
                                             encodingObj.Population,...
                                             encodingObj.Target));
                    iterateParams{rateParamIndex} = rates(i);
                    [~, averageFitness] = encodingObj.Iterate(iterateParams{:});
                    averageAverageFitness(j, :) = [initAverageFitness averageFitness];
                end
                results(:, i) = mean(averageAverageFitness, 1);
            end

            figure(figureNum);
            plot(results, '--s');
            setupPlot(['Fitness each Iteration for different ' rateName ' rates'],...
                      'Iteration number', 'Fitnes', 18);
            legends = cell(1, 3);
            for i = 1:length(rates)
                legends{i} = num2str(rates(i));
            end
            lgd = legend(legends{:}, 'Location', 'southeast');
            title(lgd, [rateName ' rates']);
        end
    end

end

function setupPlot(plotTitle, xLabel, yLabel, fontSize)
    set(gca, 'fontsize', fontSize)
    grid();
    title(plotTitle);
    xlabel(xLabel);
    ylabel(yLabel);
end

function converging = NeverConverge(~)
    converging = false;
end
