function [metric,trace,out] = run_trial(ctx,theta,mass,stopTime,strictSolver)
%RUN_TRIAL One independent simulation; failed trials can never be winners.
if nargin<4, stopTime=ctx.t(end); end
if nargin<5, strictSolver=false; end
metric=struct('ok',false,'score',ctx.cfg.failureScore,'message','', ...
    'mass',mass,'wallSeconds',0);
trace=[]; out=[]; timer=tic;
try
    assert(isnumeric(ctx.cfg.simTimeout) && isreal(ctx.cfg.simTimeout) && ...
        isscalar(ctx.cfg.simTimeout) && isfinite(ctx.cfg.simTimeout) && ...
        ctx.cfg.simTimeout>0, ...
        'mpctune:Configuration','cfg.simTimeout must be a finite positive scalar.');
    obj=mpctune.controller(ctx,theta);
    in=Simulink.SimulationInput(ctx.model);
    fields=fieldnames(ctx.data);
    for k=1:numel(fields), in=in.setVariable(fields{k},ctx.data.(fields{k})); end
    in=in.setVariable('mpcobj',obj);
    in=in.setVariable('ts_Fload',timeseries( ...
        repmat(mass*ctx.cfg.gravity,numel(ctx.t),1),ctx.t));
    in=in.setModelParameter('StopTime',num2str(stopTime,17), ...
        'SimulationMode','normal','TimeOut',ctx.cfg.simTimeout);
    if strictSolver
        in=in.setModelParameter('RelTol',num2str(ctx.cfg.solverRelTol/10), ...
            'AbsTol',num2str(ctx.cfg.solverAbsTol/10), ...
            'MaxStep',num2str(ctx.Ts/4,17));
    end
    out=sim(in);
    if ~isempty(out.ErrorMessage), error('mpctune:Simulation','%s',out.ErrorMessage); end
    trace=mpctune.extract_trace(out,ctx,stopTime);
    metric=mpctune.score_trace(trace,ctx.cfg,ctx.Ts);
    metric.mass=mass;
catch ex
    % Ctrl+C should stop the search, not be recorded as a bad controller.
    if contains(lower(ex.identifier),'interrupt'), rethrow(ex); end
    metric.ok=false; metric.score=ctx.cfg.failureScore;
    metric.message=getReport(ex,'basic','hyperlinks','off');
    metric.mass=mass;
end
metric.wallSeconds=toc(timer);
end
