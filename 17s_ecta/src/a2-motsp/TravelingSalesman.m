%% Solution for the Traveling Salesman problem
% Author:   Minh Nguyen
% Date:     2017-05-01
%%
POPULATION_SIZE = 20;
cities = importdata('cities_small.csv');
coords = cities.data(:, 2:3);
numCities = size(coords, 1);

%% Precompute euclidean distance between cities
distances = PrecomputeDistance(coords);

%% Initialize population
ie = GeneticEncoding.PermutationEncoding(POPULATION_SIZE, numCities, 0, distances,...
                                         @GeneratePopulation, @GetFitness,...
                                         @SelectWinners, @SingleCrossover,...
                                         @Mutate, @CheckConvergence, true);

VisualizeCities(coords, ie.Population(1, :))

%% Helper Functions for TSP
function distances = PrecomputeDistance(coords)
    distances = squareform(pdist(coords, 'euclidean'));
end

function GeneratePopulation(obj, populationSize, numGene, ~)
    population = zeros(populationSize, numGene);
    for i = 1:populationSize
        population(i, :) = randperm(numGene);
    end
    disp(numGene)
    set(obj, 'Population', population);
end

function fitness = GetFitness(genomes, ~, distances)
    numGenes = size(genomes, 2);
    numGenomes = size(genomes, 1);
    fitness = zeros(numGenomes, 1);
    for i = 1:numGenomes
        distanceSum = distances(1, numGenes);
        for j = 1:numGenes - 1
            genePair = num2cell(genomes(i, j:j + 1));
            distanceSum = distanceSum + distances(genePair{:});
        end
        fitness(i) = -distanceSum;
    end
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

function Mutate()
end

function child = SingleCrossover(obj, parents)
    midPoint = int16(obj.BitLength / 2 + 0.5);
    child = [parents(1, 1 : midPoint), parents(2, midPoint + 1 : obj.BitLength)];
end

function converging = CheckConvergence(~)
    converging = false;
end

%% Visualization of cities
function VisualizeCities(coords, cityOrder)
    figure(1);
    orderedCoords = coords(cityOrder, :);
    orderedCoords = [orderedCoords; coords(cityOrder(1), :)];
    plot(orderedCoords(:, 1), orderedCoords(:, 2), '-r*');
end