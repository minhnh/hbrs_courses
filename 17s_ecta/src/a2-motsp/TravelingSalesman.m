%% Solution for the Traveling Salesman problem
% Author:   Minh Nguyen
% Date:     2017-05-01
%%
POPULATION_SIZE = 200;
VERBOSE = false;
TARGET = 0;
ELITISM = true;
NUM_ITERATION = 1700;
cities = importdata('cities.csv');
coords = cities.data(:, 2:3);
numCities = size(coords, 1);

%% Precompute euclidean distance between cities
distances = PrecomputeDistance(coords);
CONSTRAINTS = distances;
NUM_GENE = numCities;

%% Initialize population
ie = GeneticEncoding.PermutationEncoding(POPULATION_SIZE, NUM_GENE, TARGET, CONSTRAINTS,...
                                         @GeneratePopulation, @GetFitness,...
                                         @SelectWinners, @RandomCrossover,...
                                         @MutateOrderChange, @CheckConvergence,...
                                         VERBOSE);
population = ie.Population;

% RandomCrossover & MutateOrderChange
% VisualizeCities(coords, ie.GetBestChild(), 1);
% [bestFitness, medianFitness, minFitness] = ie.Iterate(NUM_ITERATION, ELITISM,...
%                                                       0.9, 0.05);
% VisualizeCities(coords, ie.GetBestChild(), 2);

% RandomCrossover & MutateSwitchNeighbor
set(ie, 'Population', population);
set(ie, 'funcMutate', @MutateSwitchNeighbor)
VisualizeCities(coords, ie.GetBestChild(), 3);
[bestFitness2, medianFitness2, minFitness2] = ie.Iterate(NUM_ITERATION, ELITISM,...
                                                         0.9, 0.02);
VisualizeCities(coords, ie.GetBestChild(), 4);
figure(5)

%% Helper Functions for TSP
function distances = PrecomputeDistance(coords)
    distances = squareform(pdist(coords, 'euclidean'));
end

function GeneratePopulation(obj, populationSize, numGene, ~)
    population = zeros(populationSize, numGene);
    for i = 1:populationSize
        population(i, :) = randperm(numGene);
    end
    set(obj, 'Population', population);
end

function fitness = GetFitness(genomes, ~, distances)
    numGenes = size(genomes, 2);
    numGenomes = size(genomes, 1);
    fitness = zeros(numGenomes, 1);
    for i = 1:numGenomes
        distanceSum = distances(1, numGenes);
        for j = 1:numGenes - 1
            genePair = genomes(i, j:j + 1);
            distanceSum = distanceSum + distances(genePair(1), genePair(2));
        end
        fitness(i) = -distanceSum;
    end
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

function children = MutateOrderChange(children, mutationRate)
    mutationMatrix = rand(size(children)) < mutationRate;
    numGenome = size(children, 1);
    numGene = size(children, 2);
    numSwap = sum(mutationMatrix, 2);
    for i = 1 : numGenome
        if numSwap(i) == 0
            continue;
        end
        for j = 1 : numGene
            if ~mutationMatrix(i, j)
                continue;
            end
            swapIndex = randi(numGene);
            swapTemp = children(i, swapIndex);
            children(i, swapIndex) = children(i, j);
            children(i, j) = swapTemp;
        end
    end
end

function children = MutateSwitchNeighbor(children, mutationRate)
    mutationMatrix = rand(size(children)) < mutationRate;
    numGenome = size(children, 1);
    numGene = size(children, 2);
    numSwap = sum(mutationMatrix, 2);
    for i = 1 : numGenome
        if numSwap(i) == 0
            continue;
        end
        for j = 1 : numGene
            if ~mutationMatrix(i, j)
                continue;
            end
            if j == numGene
                swapIndex = 1;
            else
                swapIndex = j + 1;
            end
            swapTemp = children(i, swapIndex);
            children(i, swapIndex) = children(i, j);
            children(i, j) = swapTemp;
        end
    end
end

function child = SinglePointCrossover(parents)
    numGene = size(parents, 2);
    midPoint = int16(numGene / 2);
    child = -ones(1, numGene);
    child(1 : midPoint) = parents(1, 1 : midPoint);
    parent2 = parents(2, :);
    child(midPoint + 1 : numGene) = parent2(~ismember(parent2, child));
end

function child = RandomCrossover(parents)
    numGene = size(parents, 2);
    numChange = int16(numGene / 2);
    child = -ones(1, numGene);
    changeIndices = randperm(numGene, numChange);
    child(changeIndices) = parents(1, changeIndices);
    child(child == -1) = parents(2, ~ismember(parents(2, :), child));
end

function converging = CheckConvergence(~)
    converging = false;
end
