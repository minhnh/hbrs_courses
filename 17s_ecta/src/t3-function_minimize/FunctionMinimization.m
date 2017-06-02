%% Solution for the function minimization problem using CMA-ES
% Author:   Minh Nguyen
% Date:     2017-06-01
%%
% clear;clc;
POPULATION_SIZE = 15;
VERBOSE = false;
ELITISM = false; % built into weighted mean
NUM_ITERATION = 1000;
NUM_TRIES = 20;

%% Common CMA-ES parameters
mu_ = POPULATION_SIZE / 2;
weights = mu_ + 1 - (1:mu_)';
%weights = log(mu_ + 1/2) - log(1:mu_)'; % wikipedia weight ratio
weights = weights / sum(weights);
mueff = 1 / sum(weights .^ 2);


%% 2D frosen
dimensions = [2, 12, 2, 12];
functions = {@frosen, @frosen, @frastrigin, @frastrigin};
varNames = [];
bestFitnessAllTriesStruct = struct();
medianBestFitnessStruct = struct();
bestChildScruct = struct();
bestChildrenStruct = struct();

for i=1:length(dimensions)
    NUM_GENE = dimensions(i);
    TARGET = functions{i};
    varName = [func2str(TARGET) num2str(NUM_GENE) 'D' ];
    CONSTRAINTS = struct('weights', weights,...
                         'mueff', mueff,...
                         'covariance', eye(NUM_GENE, NUM_GENE),...
                         'sigma', 0.3,...
                         'cmu', mueff / NUM_GENE ^ 2);
    % Initialize population
    ie = GeneticEncoding.ValueEncoding(POPULATION_SIZE, NUM_GENE, TARGET, CONSTRAINTS,...
                                      @GeneratePopulation, @GetFitness,...
                                      @SelectWinners, @WeightedMeanCrossover,...
                                      @Mutate, @CheckConvergence,...
                                      VERBOSE);
    % start timer
    tic
    bestFitnessAllTries = zeros(NUM_TRIES, NUM_ITERATION + 1);
    bestChildren = zeros(NUM_TRIES, NUM_GENE);
    parfor k = 1:NUM_TRIES
        ie.funcInitPopulation(ie, POPULATION_SIZE, NUM_GENE, CONSTRAINTS);
        population = ie.Population;
        [bestFitness, ~, ~] = ie.Iterate(NUM_ITERATION, ELITISM, -1, -1);
        bestFitnessAllTries(k, :) = bestFitness;
        bestChildren(k, :) = ie.GetBestChild();
    end
    medianBestFitness = median(bestFitnessAllTries, 1);
    [~, argMax] = max(GetFitness(bestChildren, TARGET, CONSTRAINTS));
    bestChild = bestChildren(argMax, :);

    % time
    toc
    % save data
    bestFitnessAllTriesStruct.(varName) = bestFitnessAllTries;
    medianBestFitnessStruct.(varName) = medianBestFitness;
    bestChildStruct.(varName) = bestChild;
    bestChildrenStruct.(varName) = bestChildren;
    save('./median_fitness.mat',...
         'bestFitnessAllTriesStruct', 'medianBestFitnessStruct', 'bestChildStruct', 'bestChildrenStruct',...
         '-append');
    fig = figure(i);
    plot(medianBestFitness)
    Visualization.SetupPlot(['Median fitness over ' num2str(NUM_TRIES) ' runs for '...
                            num2str(NUM_ITERATION) ' iterations each - ' varName],...
                            'Iteration number', 'Fitness (negative value)', 20, [])
%     Visualization.save_figure(fig, ['t3-funcMin-fitness-' varName], 24);
    fig = figure(4 + i);
    boxplot(GetFitness(bestChildren, TARGET, -1));
    Visualization.SetupPlot(['Boxplot of best children fitness over ' num2str(NUM_TRIES) ' runs for '...
                            num2str(NUM_ITERATION) ' iterations each - ' varName],...
                            '', 'Fitness (negative value)', 20, [])
    disp(['Bestchild fitness ' varName ': ' num2str(GetFitness(bestChild, TARGET, -1))]);
end



%% Functions specific to Shape Matching problem
function GeneratePopulation(obj, populationSize, numGene, ~)
    % initialize population using a randomized mean and adding noise from a
    % normal distribution.
    mean = rand(1, numGene)';
    obj.Constraints.mean = mean;
    [population, ~] = SamplePopulation(mean, obj.Constraints.sigma,...
                                       obj.Constraints.covariance,...
                                       obj.Target, populationSize);
    set(obj, 'Population', population);
end

function winners = SelectWinners(obj, selection_size)
    % assuming sorted parents from SamplePopulation
    winners = obj.Population(1:selection_size, :);
end

function children = WeightedMeanCrossover(obj, parents, ~)
    % assuming sorted parents from SelectWinners
    numGenome = size(parents, 1);
    mu_ = length(obj.Constraints.weights);
    mean = parents(1:mu_, :)' * obj.Constraints.weights;
    children = repmat(mean', numGenome, 1);
end

function children = Mutate(obj, children, ~)
    % expecting a repetition of new mean as children
    newMean = children(1, :)';
    numGenome = size(children, 1);
    mu_ = length(obj.Constraints.weights);
    cmu = obj.Constraints.cmu;
    sigma = obj.Constraints.sigma;
    % rule mu update
    temp = (1 / sigma) * (children(1:mu_, :) - repmat(obj.Constraints.mean', mu_, 1));
    newCovariance = (1 - cmu) * obj.Constraints.covariance + ...
                    cmu * temp' * diag(obj.Constraints.weights) * temp;
    % sample new children
    [children, ~] = SamplePopulation(newMean, sigma, newCovariance,...
                                     obj.Target, numGenome);
    % save parameters
    obj.Constraints.mean = newMean;
    obj.Constraints.covariance = newCovariance;
end

function converging = CheckConvergence(~)
    converging = false;
end