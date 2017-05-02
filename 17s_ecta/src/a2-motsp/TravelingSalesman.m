%% Solution for the Traveling Salesman problem
% Author:   Minh Nguyen
% Date:     2017-05-01
%%
POPULATION_SIZE = 20;
cities = importdata('cities_small.csv');
coords = cities.data(:, 2:3);

%% Precompute euclidean distance between cities
distances = PrecomputeDistance(coords);

%% Initialize population
population = GeneratePopulation(coords, POPULATION_SIZE);
fitness = GetFitness(population, distances);

VisualizeCities(coords, population(1, :))

%% Helper Functions for TSP
function distances = PrecomputeDistance(coords)
    distances = squareform(pdist(coords, 'euclidean'));
end

function population = GeneratePopulation(coords, numGenome)
    population = zeros(numGenome, size(coords, 1));
    for i = 1:numGenome
        population(i, :) = randperm(size(coords, 1));
    end
end

function fitness = GetFitness(genomes, distances)
    numGenes = size(genomes, 2);
    numGenomes = size(genomes, 1);
    fitness = zeros(numGenomes, 1);
    for i = 1:numGenomes
        distanceSum = distances(1, numGenes);
        for j = 1:numGenes - 1
            genePair = num2cell(genomes(i, j:j + 1));
            distanceSum = distanceSum + distances(genePair{:});
        end
        fitness(i) = distanceSum;
    end
end

%% Visualization of cities
function VisualizeCities(coords, cityOrder)
    figure(1);
    orderedCoords = coords(cityOrder, :);
    orderedCoords = [orderedCoords; coords(cityOrder(1), :)];
    plot(orderedCoords(:, 1), orderedCoords(:, 2), '-r*');
end