% RUNMULTISENSORFILTERS - Run the multi-sensor LMB or LMBM filters

%% Admin
close all; clc;
setPath;
%% Select type of filter
filterType = 'LMBM'; % 'IC', 'PU', 'LMBM'
%% Generate the model
numberOfSensors = 2;
clutterRates = [5 5];
detectionProbabilities = [0.90 0.90];
q = [1 1];
model = generateMultisensorModel(numberOfSensors, clutterRates, detectionProbabilities, q, 'PU', 'LBP', 'Fixed');
%% Generate observations
[X_truth, z_v, z_t, Nsteps, Nv, P_vehicle] = import_scenario();

for t = 1: 500
    
%     z_v{1} = [1 2]'; z_v{2} = [3 4]';
    z_v_single{1} = z_v{1}(:,t); z_v_single{2} = z_v{2}(:,t);
    % --------------------------------------
    % Project targets from Loc to Glob
    [z_t] = loc2glob_target(z_t, z_v_single, t);
    
    % --------------------------------------
    % Generate measurements cell
    % to run prewritten MS-LMB filter
    term1 = mat2cell(z_t.glob{1}{t}', 2, [ones(1, size(z_t.glob{1}{t}, 1), 1)])';
    term2 = mat2cell(z_t.glob{2}{t}', 2, [ones(1, size(z_t.glob{2}{t}, 1), 1)])';
    measurements = {term1; term2};

    % --------------------------------------
    % Run MS-LMBM Filter
    % Input: Model parameters and Measurements at time k
    % Output: Estimated States
    stateEstimates{t} = runMultisensorLmbmFilter(model, measurements);

    % ---------------------------------------
    % Display outcomes
    disp("Total number of objects: " + num2str(length(stateEstimates{t}.mu{1})));
    plotMultisensorResults(measurements, stateEstimates{t}, X_truth.target(:,1), t)
end

function plotMultisensorResults(measurements, stateEstimates, x_truth, t)
color = ['b','r'];
x_min = [10 40]; y_min = [-5 25];
legends = "Meas";
if t == 1
    figure
    for s = 1: length(measurements)
        for o =  1: length(measurements{s})
            plot(measurements{s}{o}(1), measurements{s}{o}(2), 'x', 'Color', color(s)) , hold on
        end
    end
    if length(stateEstimates.mu{1})
        for o = 1: length(stateEstimates.mu{1})
            plot(stateEstimates.mu{1}{o}(1), stateEstimates.mu{1}{o}(2), 'o', 'Color', color(s))
        end
        legends(end + 1) = "Estim";
    end
else 
    for s = 1: length(measurements)
        for o =  1: length(measurements{s}) 
            plot(measurements{s}{o}(1), measurements{s}{o}(2), 'x', 'Color', color(s)) , hold on
        end
    end
    if length(stateEstimates.mu{1})
        for o = 1: length(stateEstimates.mu{1})
            plot( stateEstimates.mu{1}{o}(1), stateEstimates.mu{1}{o}(2), 'o', 'Color', 'g')
        end
        legends(end + 1) = "Estim";
    end
end
for gt = 1:5
        plot( x_truth(1:4:end), x_truth(2:4:end), 'o', 'Color', [0 0 0])
end
legends(end + 1) = "Gt";
xlim(x_min); ylim(y_min);
pause(.1);
xlabel("m"), ylabel("m")
legend(legends);
clf
end