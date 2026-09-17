%% SETUP MPC + OFFLINE AUTOTUNING (simulation only)
% Run in the MATLAB Command Window. urdf.slx is NEVER overwritten.
% No clear/clear all: these would erase a running tuner's local variables.
setupDir = fileparts(mfilename('fullpath'));
addpath(setupDir);
cfg = mpctune.defaults(setupDir);

%% USER SETTINGS -- fixed constraints, NOT optimization variables
cfg.payloadMass = 0.5;           % kg-equivalent downward FORCE, not added inertia
cfg.trainMasses = 0.5;
cfg.validationMasses = [0, 0.5, 1];
cfg.compensatePayload = false;  % unknown external force, not in feedforward
cfg.gravity = [0 0 -9.81];

% EXAMPLE simulation limits, NOT verified motor specifications.
cfg.tauMax = [5 40 5];          % TOTAL torque limits, N*m (symmetric)
cfg.fbSlewMax = [50 200 50];    % feedback torque rate, N*m/s
% This feedback-rate bound is NOT a jerk or total-torque-rate bound.

% FIXED scoring scales and weights, identical for every candidate.
cfg.score.positionScale = [0.02 0.02 0.02];  % rad
cfg.score.velocityScale = [0.05 0.05 0.05]; % rad/s
cfg.score.weights = [1, 0.5, 0.10, 0.02, 0.005];
% Terms: position MSE, velocity MSE, peak-position error squared,
%        normalized total-torque increments, normalized total-torque usage.

%% Nonzero nominal model; cache reference-dependent A/B/feedforward once
mpcTune = mpctune.build_context(cfg);
mpctune.publish(mpcTune, mpcTune.baseline, cfg.payloadMass);
mpcTune = mpctune.prepare_model(mpcTune);
mpcModel = mpcTune.model;
mpcobj = mpctune.controller(mpcTune, mpcTune.baseline);
sys_nominal = mpcTune.sys;

fprintf('\nReady: %s\n', mpcModel);
fprintf('Assumed total torque limits: %s N*m\n', mat2str(cfg.tauMax));
fprintf('Conservative feedback bounds: %s to %s N*m\n', ...
    mat2str(mpcTune.fbMin,4), mat2str(mpcTune.fbMax,4));
fprintf('Next: test_mpc_autotune; then result = autotune_mpc(''quick'');\n');
open_system(mpcModel);
