function ctx = build_context(cfg)
%BUILD_CONTEXT Validate inputs and cache reference models/feedforward once.
% The reviewed controller schedules models by reference, not measured q/dq.
assert(~cfg.compensatePayload,'mpctune:UnknownLoad', ...
    ['This autotuner is for unknown force loads. Keep compensatePayload=false. ' ...
     'Known-load compensation needs a separately validated model and feedforward.']);
for name={'tauMax','fbSlewMax','positionShape'}
    validateattributes(cfg.(name{1}),{'double'},{'size',[1 3],'finite','positive'});
end
validateattributes(cfg.outputScales,{'double'},{'size',[1 9],'positive','finite'});
validateattributes(cfg.score.positionScale,{'double'},{'size',[1 3],'positive','finite'});
validateattributes(cfg.score.velocityScale,{'double'},{'size',[1 3],'positive','finite'});
validateattributes(cfg.score.weights,{'double'},{'size',[1 5],'nonnegative','finite'});
assert(any(cfg.score.weights>0),'Score weights cannot all be zero.');
validateattributes(cfg.initialWeights,{'double'},{'size',[1 5],'positive','finite'});
validateattributes(cfg.weightMin,{'double'},{'size',[1 5],'positive','finite'});
validateattributes(cfg.weightMax,{'double'},{'size',[1 5],'positive','finite'});
assert(all(cfg.weightMax>cfg.weightMin) && ...
    all(cfg.initialWeights>=cfg.weightMin & cfg.initialWeights<=cfg.weightMax), ...
    'Initial weights must lie within positive search bounds.');
validateattributes(cfg.predictionChoices,{'double'},{'vector','integer','>=',2});
validateattributes(cfg.controlChoices,{'double'},{'vector','integer','positive'});
assert(min(cfg.controlChoices)<=min(cfg.predictionChoices),'No valid control horizon.');
validateattributes(cfg.initialHorizons,{'double'},{'size',[1 2],'integer','positive'});
assert(cfg.initialHorizons(2)<=cfg.initialHorizons(1),'Initial Nc must not exceed Np.');
masses=[cfg.payloadMass cfg.trainMasses cfg.validationMasses];
validateattributes(masses,{'double'},{'vector','finite','nonnegative','nonempty'});
assert(~isempty(cfg.trainMasses) && ~isempty(cfg.validationMasses),'Load lists cannot be empty.');
src=fullfile(cfg.rootDir,'urdf.slx');
assert(isfile(src),'mpctune:MissingModel','Missing urdf.slx in %s.',cfg.rootDir);
tr=load(fullfile(cfg.rootDir,'trajectory_new.mat'));
for f={'t','q','qd','qdd'}
    assert(isfield(tr,f{1}),'Trajectory is missing %s.',f{1});
    values=tr.(f{1});
    assert(isnumeric(values) && all(isfinite(values(:))), ...
        'Trajectory %s must be finite numeric data.',f{1});
end
t=tr.t(:); N=numel(t);
assert(N>=3 && abs(t(1))<1e-12 && all(diff(t)>0),'Time must increase from zero.');
assert(isequal(size(tr.q),[N 3],size(tr.qd),size(tr.qdd)), ...
    'q/qd/qdd must each have N rows and 3 columns.');
% Canonicalize the grid: median(diff(t)) can be slightly below 0.01 due to
% floating-point cancellation and otherwise delay ZOH data by one sample.
Ts=round(median(diff(t)),12);
assert(Ts>0,'Invalid sample time.');
assert(max(abs(diff(t)-Ts))<1e-8*max(1,Ts),'Trajectory must have a uniform sample time.');
t=(0:N-1)'*Ts;
q=tr.q.*[1 -1 -1]; dq=tr.qd.*[1 -1 -1]; ddq=tr.qdd.*[1 -1 -1];
assert(max(abs([q(1,:) dq(1,:)]))<1e-9, ...
    'This source model requires the reference to start with q=dq=0.');
robot=importrobot(fullfile(cfg.rootDir,'urdf.urdf'));
robot.DataFormat='column'; robot.Gravity=cfg.gravity;
assert(numel(homeConfiguration(robot))==3,'Expected a 3-DOF robot.');
if ~any(strcmp(robot.BodyNames,'tool_tip'))
    tip=rigidBody('tool_tip'); jt=rigidBodyJoint('tool_tip_fixed','fixed');
    setFixedTransform(jt,trvec2tform(cfg.toolOffset));
    tip.Joint=jt; tip.Mass=0; tip.CenterOfMass=[0 0 0]; tip.Inertia=zeros(1,6);
    addBody(robot,tip,robot.BodyNames{end});
end
A=zeros(9,9,N); B=zeros(9,3,N); ff=zeros(N,3);
fprintf('Caching %d reference models/feedforward samples (once per setup)...\n',N);
timer=tic;
for k=1:N
    [A(:,:,k),B(:,:,k)]=mpctune.error_model(robot,q(k,:)',dq(k,:)',ddq(k,:)', ...
        Ts,cfg.linearizationDelta);
    ff(k,:)=inverseDynamics(robot,q(k,:)',dq(k,:)',ddq(k,:)')';
    if k==1 || mod(k,250)==0 || k==N
        fprintf('  %d/%d samples, %.1f s\n',k,N,toc(timer)); drawnow;
    end
end
assert(all(isfinite(A(:))) && all(isfinite(B(:))) && all(isfinite(ff(:))), ...
    'Nonfinite model or feedforward.');
% Intersection over ALL time samples: tauMin <= ff(t)+u <= tauMax.
% Conservative, constant bounds. FF is held with ZOH in the prepared model.
fbMin=-cfg.tauMax-min(ff,[],1);
fbMax= cfg.tauMax-max(ff,[],1);
assert(all(fbMin<0 & fbMax>0),'mpctune:TorqueBudget', ...
    ['Feedforward exceeds assumed total-torque limits or leaves no zero-input ' ...
     'margin. Peak abs FF=%s N*m. Verify units/model/trajectory/limits; ' ...
     'the tuner will NOT increase limits automatically.'],mat2str(max(abs(ff),[],1),5));
ctx.cfg=cfg; ctx.t=t; ctx.Ts=Ts; ctx.q=q; ctx.dq=dq; ctx.ddq=ddq;
ctx.robot=robot; ctx.ff=ff; ctx.fbMin=fbMin; ctx.fbMax=fbMax;
ctx.A0=A(:,:,1); ctx.B0=B(:,:,1);
ctx.sys=ss(ctx.A0,ctx.B0,eye(9),zeros(9,3),Ts);
ctx.baseline=[log10(cfg.initialWeights) cfg.initialHorizons];
ctx.data.Ts=Ts;
ctx.data.ts_q=timeseries(q,t); ctx.data.ts_qd=timeseries(dq,t);
ctx.data.ts_qdd=timeseries(ddq,t);
ctx.data.mpc_A=timeseries(A,t); ctx.data.mpc_B=timeseries(B,t);
ctx.data.mpc_tau_ff=timeseries(ff,t);
ctx.data.mpc_gravity=cfg.gravity;
ctx.data.mpc_tool_offset=cfg.toolOffset;
ctx.data.mpc_tau_limit=cfg.tauMax(:);
ctx.data.robot=robot;
ctx.sourceModel=src;
ctx.created=datestr(now,30);
end
