%% Solution for the Graph Coloring problem
% Author:   Minh Nguyen
% Date:     2017-05-11
%%
clear;clc;
POPULATION_SIZE = 100;
VERBOSE = false;
TARGET = 0;
ELITISM = true;
NUM_ITERATION = 60;
NUM_TRIES = 3;
NUM_COLOR = 4;

%% 2) Setup the graphs
% 2a) Setup a small graph for testing your code
adjacency = bucky;

NUM_GENE = size(adjacency, 1);
CONSTRAINTS = {NUM_COLOR, adjacency};

%% Initialize population
ie = GeneticEncoding.PermutationEncoding(POPULATION_SIZE, NUM_GENE, TARGET, CONSTRAINTS,...
                                         @GeneratePopulation, @GetFitness,...
                                         @SelectWinners, @RandomCrossover,...
                                         @Mutate, @CheckConvergence,...
                                         VERBOSE);

VisualizeGraph(adjacency, ie.GetBestChild()', NUM_COLOR, 1, 'Initialized Graph', 'initial')

%% Iterate algorithm
bestFitnessFinal = zeros(NUM_TRIES, NUM_ITERATION + 1);
bestChildren = zeros(NUM_TRIES, NUM_GENE);
parfor i = 1:NUM_TRIES
    GeneratePopulation(ie, POPULATION_SIZE, NUM_GENE, TARGET);
    [bestFitness, ~, ~] = ie.Iterate(NUM_ITERATION, ELITISM, 0.8, 0.01);
    bestFitnessFinal(i, :) = bestFitness;
    bestChildren(i, :) = ie.GetBestChild();
end
medianBestFitness = median(bestFitnessFinal, 1);
[~, argMax] = max(GetFitness(bestChildren, TARGET, CONSTRAINTS));
bestChild = bestChildren(argMax, :);

VisualizeGraph(adjacency, bestChild', NUM_COLOR, 2, "Solution Graph", 'solution')
disp(['Best Fitness: ' num2str(GetFitness(bestChild, TARGET, CONSTRAINTS))]);

%% Problem specific functions
function GeneratePopulation(obj, populationSize, numGene, ~)
    population = randi(obj.Constraints{1}, populationSize, numGene);
    set(obj, 'Population', population);
end

function winners = SelectWinners(obj, selection_size)
    winners = zeros(selection_size, obj.NumGene);
    for i = 1:selection_size
        parentIndices = randperm(size(obj.Population, 1), 2);
        parents = obj.Population(parentIndices, :);
        fitness = obj.funcGetFitness(parents, obj.Target, obj.Constraints);
        [~, maxArg] = max(fitness);
        winners(i, :) = parents(maxArg, :);
    end
end

function children = Mutate(children, mutationRate)
    mutationMatrix = rand(size(children)) < mutationRate;
    children(mutationMatrix) = randi(4, size(children(mutationMatrix)));
end

function converging = CheckConvergence(~)
    converging = false;
end
