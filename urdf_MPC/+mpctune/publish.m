function publish(ctx,theta,mass)
%PUBLISH Workspace values for interactive Run on the prepared model.
v=ctx.data; names=fieldnames(v);
for k=1:numel(names), assignin('base',names{k},v.(names{k})); end
[obj,p]=mpctune.controller(ctx,theta);
assignin('base','mpcobj',obj); assignin('base','mpcParameters',p);
assignin('base','ts_Fload',timeseries(repmat(mass*ctx.cfg.gravity,numel(ctx.t),1),ctx.t));
assignin('base','payloadMass',mass);
assignin('base','q_full',ctx.q); assignin('base','qd_full',ctx.dq);
assignin('base','qdd_full',ctx.ddq); assignin('base','t_full',ctx.t);
end
