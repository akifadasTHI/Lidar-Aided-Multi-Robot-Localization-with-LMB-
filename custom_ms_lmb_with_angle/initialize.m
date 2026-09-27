

obj_loc = []; z_target = [];
% Define Path to LMBM functions
setPath;
% Select type of filter
filterType = 'LMBM'; % 'IC', 'PU', 'LMBM'
% Generate the model
numberOfSensors = 2;
clutterRates = [4 4];
detectionProbabilities = [0.90 0.90];
q = [0.5 1.2];

% Generate LMBM model parameters
model = generateMultisensorModel(numberOfSensors, clutterRates, detectionProbabilities, q, 'LMBM', 'Gibbs', 'Fixed');

% Import observations and reshape
[X_truth, z_gnss, z_t, Nsteps, Nv, P_vehicle] = import_scenario();

% Import EKF parameters
ekf_params;

% Bayesian Recursion initialization
Init1.x = [z_gnss{1}(:,1); 0; 0];
Init1.cov = diag([10000^2, 10000^2, 100, 100]); % Initial Covariance

Init2.x = [z_gnss{2}(:,1); 0; 0];
Init2.cov = diag([10000^2, 10000^2, 100, 100]); % Initial Covariance

meas_obj{1} = []; meas_obj{2} = []; 

z.val(2).cov = R_target;
z.val(2).val = z_target;
z.val(3).cov = R_target;
z.val(3).val = z_target;
