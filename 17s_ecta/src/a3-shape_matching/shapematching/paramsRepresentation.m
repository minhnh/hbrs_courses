%% Initialize Static Algorithm Parameters

%% Representation Constants
p.numEvalPts = 256;
p.numPts = 16; 			% number of control points (N)
p.grange = [-0.7 0.7];
p.minCtlPtDist = 1e-3;

%% Visualization Parameters
d.headless = true;         % Toggle visualization
d.terminalOutputRate = 100;
d.foilPlotRate = 1;
d.foilPlotFig = 1;
d.lineGraphRate = 50;       % How often (in gens) to update improvement graph
d.lineGraphFig = 11;       % How often (in gens) to update improvement graph
d.mapUpdateRate = 100;      % How often (in gens) to update MAP-Elites MAP
d.mapFig = 21;       % How often (in gens) to update improvement graph
d.smoothing = 50;           % Number of generations to include in smoothing
