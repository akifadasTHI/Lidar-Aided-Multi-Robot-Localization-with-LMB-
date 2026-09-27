%% Initialise variables
simulationLength = simulation_time - time_EKF_solo;
% Struct containing objects' Bernoulli parameters and metadata
% Output struct
hypotheses = model.hypotheses;
objects = model.trajectory;
stateEstimates.labels = cell(simulationLength, 1);
stateEstimates.mu = cell(simulationLength, 1);
stateEstimates.Sigma = cell(simulationLength, 1);
stateEstimates.objects = objects;