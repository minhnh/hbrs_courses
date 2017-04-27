%% OneMax.m
% Solution for the OneMax problem
% Author:   Minh Nguyen
% Date: 2017-04-15
%%
POPULATION_SIZE = 20;
MAX_INIT_VALUE = 6;
BIT_LENGTH = 10;
ITERATION_NUM = 50;
ELITISM = true;
CROSSOVER_RATE = 0.7;
MUTATION_RATE = 0.1;
NUM_TRIES = 20;

DEFAULT_CONSTRUCTOR_PARAMS = { MAX_INIT_VALUE, BIT_LENGTH, @GetFitness,...
                               @SelectWinners, @SingleCrossover,...
                               @CheckConvergence, ones(1, BIT_LENGTH), false};
DEFAULT_ITERATE_PARAMS = {ITERATION_NUM, ELITISM, CROSSOVER_RATE, MUTATION_RATE};

CHECK_CONVERGE_INDEX = 6;
NUM_ITER_PARAM_INDEX = 1;
CROSSOVER_PARAM_INDEX = 3;
MUTATION_PARAM_INDEX = 4;

%% Create evaluation object
evalPopulation = GeneticEncoding.Evaluation(@GeneticEncoding.BinaryEncoding,...
                                            POPULATION_SIZE,...
                                            DEFAULT_CONSTRUCTOR_PARAMS,...
                                            DEFAULT_ITERATE_PARAMS,...
                                            CHECK_CONVERGE_INDEX,...
                                            NUM_ITER_PARAM_INDEX);

%% Vary population
sizes = 10:5:60;
evalPopulation.EvalNumIterOverPopulation(sizes, NUM_TRIES);
figure(1);
plot(evalPopulation.NumIterOverPopulationResult(1, :),...
     evalPopulation.NumIterOverPopulationResult(2, :), '--rs');
GeneticEncoding.Evaluation.setupPlot(...
        'Average iteration number over population size',...
        'Population size', 'Number of iterations', 18);

%% Vary crossover rate - plot average number of iterations
rates = 0.1:0.1:0.9;
evalPopulation.EvalNumIterOverRates(rates, NUM_TRIES, CROSSOVER_PARAM_INDEX)
figure(2);
plot(evalPopulation.NumIterOverRatesResult(1, :),...
     evalPopulation.NumIterOverRatesResult(2, :), '--rs');
GeneticEncoding.Evaluation.setupPlot(...
        'Average iteration number over crossover rates',...
        'crossover rates', 'Number of iterations', 18);

%% Vary crossover rate - plot fitness over each iteration
rates = 0.45:0.15:0.9;
rateName = 'Crossover';
evalPopulation.EvalFitnessOverRates(rates, 40, NUM_TRIES,...
                                    CROSSOVER_PARAM_INDEX, rateName);

figure(3);
plot(evalPopulation.FitnessOverCrossoverRatesResult, '--s');
GeneticEncoding.Evaluation.setupPlot(...
        ['Fitness each Iteration for different ' rateName ' rates'],...
        'Iteration number', 'Fitnes', 18);
lgd = GeneticEncoding.Evaluation.SetupLegends(rates);
title(lgd, [rateName ' rates']);

%% Vary mutation rate - plot fitness over each iteration
rates = 0.1:0.1:0.4;
rateName = 'Mutation';
evalPopulation.EvalFitnessOverRates(rates, 40, NUM_TRIES,...
                                    MUTATION_PARAM_INDEX, rateName)
figure(4);
plot(evalPopulation.FitnessOverMutationRatesResult, '--s');
GeneticEncoding.Evaluation.setupPlot(...
        ['Fitness each Iteration for different ' rateName ' rates'],...
        'Iteration number', 'Fitnes', 18);
lgd = GeneticEncoding.Evaluation.SetupLegends(rates);
title(lgd, [rateName ' rates']);

%% functions specific to the OneMax task
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

function converging = CheckConvergence(obj)
    converging = false;
    fitness = obj.funcGetFitness(obj.Population, obj.Target);
    targetFitness = obj.BitLength;
    if abs(max(fitness) - targetFitness) < 1
        converging = true;
    end
end