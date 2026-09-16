function cfg = defaults(rootDir)
%DEFAULTS Settings shared by setup, simulation, scoring and search.
cfg.version = 'mpctune-1.0';
cfg.rootDir = rootDir;
cfg.payloadMass = 0.5;
cfg.trainMasses = 0.5;
cfg.validationMasses = [0 0.5 1];
cfg.compensatePayload = false;
cfg.gravity = [0 0 -9.81];
cfg.toolOffset = [0.34349 0.04674 0.00607];
cfg.tauMax = [5 40 5];
cfg.fbSlewMax = [50 200 50];
cfg.linearizationDelta = 1e-5;
cfg.outputScales = [0.25 0.25 0.25, 0.05 0.05 0.05, 0.10 0.10 0.10];
cfg.positionShape = [1 2.5 1];
% Group weights: integral, position, velocity, MV, MVRate.
% Joint ratios stay fixed; this release does NOT tune all 15 independently.
cfg.initialWeights = [0.1 1 0.3 0.1 0.5];
cfg.weightMin = [0.01 0.2 0.02 0.001 0.03];
cfg.weightMax = [2 5 3 1 10];
cfg.initialHorizons = [30 5];
cfg.predictionChoices = [15 20 30 40 50];
cfg.controlChoices = [2 3 5 8 10];
cfg.seed = 68;
cfg.quickTrials = 12;           % includes baseline; validation is additional
cfg.fullTrials = 40;
cfg.validationTop = 2;         % baseline + up to two best search candidates
cfg.minimumImprovement = 0.005;
cfg.score.positionScale = [0.02 0.02 0.02];
cfg.score.velocityScale = [0.05 0.05 0.05];
cfg.score.weights = [1 0.5 0.10 0.02 0.005];
cfg.score.rejectPositionError = 1.0; % failure threshold, NOT a joint limit
cfg.score.rejectVelocity = 10.0;    % rad/s; failure threshold, NOT an OV bound
cfg.score.torqueTolerance = 1e-5;
cfg.solverRelTol = 1e-5;
cfg.solverAbsTol = 1e-7;
cfg.simTimeout = 600;           % wall-clock seconds per simulation
cfg.failureScore = 1e12;
end
