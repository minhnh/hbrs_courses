%% Solution for the Shape Matching problem using CMA-ES
% Author:   Minh Nguyen
% Date:     2017-05-25
%%
% clear;clc;
NUM_GENE = 32;
POPULATION_SIZE = 15;
VERBOSE = false;
ELITISM = false; % built into weighted mean
NUM_ITERATION = 1200;
NUM_TRIES = 30;

%% Precompute euclidean distance between cities
% Create a NACA foil
numEvalPts = 256;                            % Num evaluation points
nacaNum = [0, 0, 1, 2];                      % NACA Parameters
% nacaNum = [5, 5, 2, 2];                      % NACA Parameters
% nacaNum = [9, 7, 3, 5];                      % NACA Parameters
nacafoil = create_naca(nacaNum, numEvalPts); % Create foil
TARGET = nacafoil;

mu_ = POPULATION_SIZE / 2;
weights = mu_ + 1 - (1:mu_)';
%weights = log(mu_ + 1/2) - log(1:mu_)'; % wikipedia weight ratio
weights = weights / sum(weights);
mueff = 1 / sum(weights .^ 2);
CONSTRAINTS = struct('weights', weights,...
                     'mueff', mueff,...
                     'covariance', eye(NUM_GENE, NUM_GENE),...
                     'sigma', 0.3,...
                     'cmu', mueff / NUM_GENE ^ 2);

%% Initialize population
ie = GeneticEncoding.ValueEncoding(POPULATION_SIZE, NUM_GENE, TARGET, CONSTRAINTS,...
                                  @GeneratePopulation, @GetFitness,...
                                  @SelectWinners, @WeightedMeanCrossover,...
                                  @Mutate, @CheckConvergence,...
                                  VERBOSE);
% start timer
tic
VisualizeFoil(ie.GetBestChild()', nacafoil, nacaNum, 1);
bestFitnessAllTries = zeros(NUM_TRIES, NUM_ITERATION + 1);
bestChildren = zeros(NUM_TRIES, NUM_GENE);
parfor i = 1:NUM_TRIES
    tic
    ie.funcInitPopulation(ie, POPULATION_SIZE, NUM_GENE, CONSTRAINTS);
    population = ie.Population;
    [bestFitness, ~, ~] = ie.Iterate(NUM_ITERATION, ELITISM, -1, -1);
    bestFitnessAllTries(i, :) = bestFitness;
    bestChildren(i, :) = ie.GetBestChild();
    toc
end
medianBestFitness = median(bestFitnessAllTries, 1);
[~, argMax] = max(GetFitness(bestChildren, TARGET, CONSTRAINTS));
bestChild = bestChildren(argMax, :);
VisualizeFoil(bestChild', nacafoil, nacaNum, 2);
% time
toc
% save data
save('./median_fitness.mat',...
     'bestFitnessAllTries', 'medianBestFitness', 'bestChild', 'bestChildren');

%% Functions specific to Shape Matching problem
function GeneratePopulation(obj, populationSize, numGene, ~)
    % initialize population using a randomized mean and adding noise from a
    % normal distribution.
    mean = rand(1, numGene)' - 0.5;
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