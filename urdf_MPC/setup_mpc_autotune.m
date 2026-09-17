%% SETUP MPC + OFFLINE AUTOTUNING (simulation only)
% Run this file before test_mpc_autotune and autotune_mpc.
% A unique model copy is used; urdf.slx is never overwritten here.
setupDir = fileparts(mfilename('fullpath'));
addpath(setupDir);
cfg = mpctune.defaults(setupDir);

%% FIXED EXPERIMENT SETTINGS -- not optimization variables
cfg.payloadMass = 0.5;
cfg.trainMasses = 0.5;
cfg.validationMasses = [0 0.5 1];
cfg.compensatePayload = false;
cfg.gravity = [0 0 -9.81];

% Total actuator limits and feedback slew limits used by every candidate.
cfg.tauMax = [5 40 5];
cfg.fbSlewMax = [50 200 50];

% Fixed score definition used by every candidate.
cfg.score.positionScale = [0.02 0.02 0.02];
cfg.score.velocityScale = [0.05 0.05 0.05];
cfg.score.weights = [1 0.5 0.10 0.02 0.005];

% Use the previously validated QUICK winner as the FULL-search baseline.
% Group order: [Qz, base-Qe, Qed, R, Rdu]; Qe is multiplied by [1 2.5 1].
% Therefore trial 1 of a FULL run is the known quick controller, and a new
% controller is selected only if held-out validation improves by >= 0.5%.
cfg.initialWeights = [0.1376 3.1641 0.2884 0.2268 0.3757];
cfg.initialHorizons = [20 10];

%% Build nonzero nominal model and prepare an isolated Simulink copy
mpcTune = mpctune.build_context(cfg);
mpctune.publish(mpcTune,mpcTune.baseline,cfg.payloadMass);
mpcTune = mpctune.prepare_model(mpcTune);
mpcModel = mpcTune.model;
mpcobj = mpctune.controller(mpcTune,mpcTune.baseline);
sys_nominal = mpcTune.sys;

fprintf('\nReady: %s\n',mpcModel);
fprintf('Assumed total torque limits: %s N*m\n',mat2str(cfg.tauMax));
fprintf('Conservative feedback bounds: %s to %s N*m\n', ...
    mat2str(mpcTune.fbMin,4),mat2str(mpcTune.fbMax,4));
fprintf(['Next: test_mpc_autotune; then ' ...
    'result_full = autotune_mpc(''full'');\n']);
open_system(mpcModel);
