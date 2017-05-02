
POPULATION_SIZE = 30;
BIT_LENGTH = 22;
MAX_INIT_VALUE = 2 ^ BIT_LENGTH - 1;
ITERATION_NUM = 100;
ELITISM = true;
CROSSOVER_RATE = 0.7;
MUTATION_RATE = 0.1;
NUM_TRIES = 10;

items = importdata('items.csv');
weights = items.data(:, 1)';
values = items.data(:, 2)';

DEFAULT_CONSTRUCTOR_PARAMS = { MAX_INIT_VALUE, BIT_LENGTH, items, @GetFitness,...
                               @SelectWinners, @SingleCrossover,...
                               @CheckConvergence, ones(1, BIT_LENGTH), false};
DEFAULT_ITERATE_PARAMS = {ITERATION_NUM, ELITISM, CROSSOVER_RATE, MUTATION_RATE};

CHECK_CONVERGE_INDEX = 6;
NUM_ITER_PARAM_INDEX = 1;
CROSSOVER_PARAM_INDEX = 3;
MUTATION_PARAM_INDEX = 4;

binEncoding = GeneticEncoding.BinaryEncoding(POPULATION_SIZE, DEFAULT_CONSTRUCTOR_PARAMS{:});
[bestFit, medianFit, minFit] = binEncoding.Iterate(DEFAULT_ITERATE_PARAMS{:});

bestChild = binEncoding.GetBestChild();
bestChildWeight = sum(weights .* bestChild);
bestChildValue = sum(values .* bestChild);

disp(['Best solution weight: ' num2str(bestChildWeight)]);
disp(['Best solution value: ' num2str(bestChildValue)]);
disp('Items:');
disp(items.textdata(bestChild == 1, 1));

%% Create evaluation object
evalPopulation = GeneticEncoding.Evaluation(@GeneticEncoding.BinaryEncoding,...
                                            POPULATION_SIZE,...
                                            DEFAULT_CONSTRUCTOR_PARAMS,...
                                            DEFAULT_ITERATE_PARAMS,...
                                            CHECK_CONVERGE_INDEX,...
                                            NUM_ITER_PARAM_INDEX);

%% Vary mutation rate - plot fitness over each iteration
rates = 0.04:0.02:0.08;
rateName = 'Mutation';
evalPopulation.EvalFitnessOverRates(rates, ITERATION_NUM, NUM_TRIES,...
                                    MUTATION_PARAM_INDEX, rateName)
figure(2);
plot(evalPopulation.MaxFitnessOverMutationRatesResult, '--s');
GeneticEncoding.Evaluation.setupPlot(...
        ['Maximum Fitness each Iteration for different ' rateName ' rates'],...
        'Iteration number', 'Fitnes', 18);
lgd = GeneticEncoding.Evaluation.SetupLegends(rates);
title(lgd, [rateName ' rates']);

figure(3);
medianMaxFitness = [evalPopulation.MaxFitnessOverMutationRatesResult(:, end)...
                    evalPopulation.MedianFitnessOverMutationRatesResult(:, end)];
plot(1:ITERATION_NUM + 1, medianMaxFitness, '--s');
GeneticEncoding.Evaluation.setupPlot(...
        ['Median and maximum fitness each Iteration for ' rateName ' rate of ' num2str(rates(end))],...
        'Iteration number', 'Fitnes', 18);
lgd = legend('Maximum fitness', 'Median fitness', 'Location', 'southeast');

%% functions specific to the Knapsack task
function fitness = GetFitness(gene, ~, items)
    weights = items.data(:, 1)';
    values = items.data(:, 2)';
    fitness = sum(gene .* values, 2);
    penalties = 400 - sum(gene .* weights, 2);
    penalties(penalties > 0) = 0;
    fitness = fitness + penalties*2;
end

function winners = SelectWinners(obj, selection_size)
    winners = zeros(selection_size, obj.BitLength);
    for i = 1:selection_size
        parentIndices = randperm(size(obj.Population, 1), 2);
        parents = obj.Population(parentIndices, :);
        fitness = obj.funcGetFitness(parents, obj.Target, obj.Constraints);
        [~, maxArg] = max(fitness);
        winners(i, :) = parents(maxArg, :);
    end
end

function child = SingleCrossover(obj, parents)
    midPoint = int16(obj.BitLength / 2 + 0.5);
    child = [parents(1, 1 : midPoint), parents(2, midPoint + 1 : obj.BitLength)];
end

function converging = CheckConvergence(~)
    converging = false;
end