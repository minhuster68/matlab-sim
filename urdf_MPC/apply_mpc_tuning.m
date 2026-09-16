function apply_mpc_tuning(resultFile)
%APPLY_MPC_TUNING Publish a validated result to the CURRENT prepared model.
% Does not overwrite setup_mpc.m or urdf.slx; run this again after a new setup.
assert(nargin==1 && isfile(resultFile),'Supply a saved mpc_tuning_result.mat path.');
assert(evalin('base','exist(''mpcTune'',''var'')==1'),'Run setup_mpc first.');
ctx=evalin('base','mpcTune'); saved=load(resultFile,'result'); r=saved.result;
assert(r.validated && r.selectedOK,'Result is not validated/admissible.');
assert(isequaln(r.signature,mpctune.signature(ctx)), ...
    ['Setup/model/trajectory/constraints differ from the tuned experiment. ' ...
     'Restore the original settings or rerun tuning; weights were not applied.']);
assert(bdIsLoaded(ctx.model),'Prepared model is closed. Run setup_mpc again.');
assert(strcmp(get_param(ctx.model,'SimulationStatus'),'stopped'),'Stop simulation first.');
mpctune.publish(ctx,r.theta,ctx.cfg.payloadMass);
assignin('base','mpcTuningResult',r);
fprintf('Applied trial %d to %s. Source urdf.slx is unchanged.\n',r.selectedTrial,ctx.model);
disp(r.parameters);
open_system(ctx.model);
end
