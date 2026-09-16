function result = autotune_mpc(mode)
%AUTOTUNE_MPC Reproducible random + local search, with held-out-load validation.
% Run: setup_mpc; test_mpc_autotune; result=autotune_mpc('quick');
% 'quick': 12 candidates. 'full': 40. A positive integer sets another budget.
% Requires MATLAB/Simulink/MPC/Robotics/Simscape Multibody, not Optimization Toolbox.
if nargin<1, mode='quick'; end
assert(evalin('base','exist(''mpcTune'',''var'')==1'), ...
    'mpctune:Setup','Run setup_mpc first.');
ctx=evalin('base','mpcTune');
assert(isfield(ctx,'smokePassed') && ctx.smokePassed, ...
    'mpctune:Smoke','Run test_mpc_autotune successfully before autotuning.');
assert(bdIsLoaded(ctx.model),'Prepared model is closed. Run setup_mpc again.');
if isnumeric(mode)
    validateattributes(mode,{'double'},{'scalar','integer','>=',2}); budget=mode;
elseif strcmpi(mode,'quick'), budget=ctx.cfg.quickTrials;
elseif strcmpi(mode,'full'), budget=ctx.cfg.fullTrials;
else, error('Use ''quick'', ''full'', or an integer >= 2.'); end

searchDir=tempname(ctx.runDir); mkdir(searchDir);
result=struct('version',ctx.cfg.version,'created',datestr(now,30), ...
    'cfg',ctx.cfg,'validated',false,'selectedOK',false,'isImproved',false);
result.file=fullfile(searchDir,'mpc_tuning_result.mat');
result.history=struct('theta',{},'score',{},'ok',{},'wallSeconds',{},'message',{},'cases',{});
result.validation={}; result.baseline=ctx.baseline;
checkpoint=fullfile(searchDir,'checkpoint.mat');
savedRng=rng; cleanup=onCleanup(@()rng(savedRng)); %#ok<NASGU>
rng(ctx.cfg.seed,'twister');
best=ctx.baseline; bestScore=ctx.cfg.failureScore;
tried=zeros(0,7);
fprintf('\n%d candidates x %d training load(s); full %.2f s trajectory each.\n', ...
    budget,numel(ctx.cfg.trainMasses),ctx.t(end));
fprintf('Validation uses tighter solver tolerances. No global-optimum guarantee.\n');
fprintf('Checkpoint: %s\n',checkpoint);
for i=1:budget
    if i==1, theta=ctx.baseline;
    else, theta=mpctune.propose(ctx,i,budget,best,tried); end
    tried(end+1,:)=theta; %#ok<AGROW>
    fprintf('\nTrial %d/%d: Np=%d Nc=%d weights=%s\n', ...
        i,budget,theta(6),theta(7),mat2str(10.^theta(1:5),3));
    [value,ok,cases,elapsed,message]=evaluate(ctx,theta,ctx.cfg.trainMasses,false);
    result.history(end+1)=struct('theta',theta,'score',value,'ok',ok, ...
        'wallSeconds',elapsed,'message',message,'cases',{cases}); %#ok<AGROW>
    if ok && value<bestScore, best=theta; bestScore=value; end
    result.bestTrainingTheta=best; result.bestTrainingScore=bestScore;
    result.rngState=rng;
    save_checkpoint(checkpoint,result);
    fprintf('  score=%.6g, valid=%d, wall time=%.1f s\n',value,ok,elapsed);
    if ~ok, fprintf('  Rejected: %s\n',message); end
    drawnow;
end

% Baseline is always rechecked under the same validation loads/tolerances.
scores=[result.history.score]; [~,order]=sort(scores);
selected=1;
for k=1:numel(order)
    j=order(k);
    if j~=1 && result.history(j).ok
        selected(end+1)=j; %#ok<AGROW>
        if numel(selected)>=ctx.cfg.validationTop+1, break; end
    end
end
fprintf('\nValidating baseline and %d candidate(s) at loads %s kg-equivalent.\n', ...
    numel(selected)-1,mat2str(ctx.cfg.validationMasses));
values=inf(1,numel(selected)); valid=false(1,numel(selected));
for k=1:numel(selected)
    theta=result.history(selected(k)).theta;
    fprintf('Validation %d/%d, source trial %d\n',k,numel(selected),selected(k));
    [values(k),valid(k),cases,elapsed,msg,traces]= ...
        evaluate(ctx,theta,ctx.cfg.validationMasses,true);
    result.validation{k}=struct('trial',selected(k),'theta',theta, ...
        'score',values(k),'ok',valid(k),'cases',{cases}, ...
        'wallSeconds',elapsed,'message',msg,'traces',{traces});
    save_checkpoint(checkpoint,result);
end
winner=1;
if any(valid)
    admissible=values; admissible(~valid)=inf;
    [~,candidate]=min(admissible);
    if ~valid(1) || admissible(candidate)<values(1)*(1-ctx.cfg.minimumImprovement)
        winner=candidate;
    end
end
result.validated=true;
result.selectedValidation=winner;
result.selectedTrial=selected(winner);
result.selectedOK=valid(winner);
result.isImproved=result.selectedOK && winner~=1;
result.theta=result.history(result.selectedTrial).theta;
[~,result.parameters]=mpctune.controller(ctx,result.theta);
result.validationScore=values(winner);
result.baselineValidationScore=values(1);
result.message='No valid controller found; do not apply this result.';
if result.selectedOK
    if result.isImproved, result.message='Validated improvement selected.';
    else, result.message='No sufficient validated improvement; baseline retained.'; end
end
% Snapshot ties saved weights to the exact trajectory and physical model.
result.signature=mpctune.signature(ctx);
save_checkpoint(result.file,result);
save_checkpoint(checkpoint,result);
try
    write_history(searchDir,result);
    if result.selectedOK, mpctune.plot_result(result,searchDir); end
catch ex
    warning('mpctune:OptionalExport', ...
        'MAT result was saved, but optional CSV/plot export failed: %s',ex.message);
end
fprintf('\n%s\n',result.message); disp(result.parameters);
fprintf('Result: %s\n',result.file);
fprintf('To apply to the prepared model: apply_mpc_tuning(result.file)\n');
% We intentionally do not silently publish/activate the winner.
end

function [value,ok,cases,elapsed,msg,traces]=evaluate(ctx,theta,masses,strict)
cases=cell(1,numel(masses)); traces=cases; values=zeros(size(masses));
ok=true; elapsed=0; messages={};
for j=1:numel(masses)
    [cases{j},traces{j}]=mpctune.run_trial(ctx,theta,masses(j),ctx.t(end),strict);
    m=cases{j}; values(j)=m.score; ok=ok && m.ok; elapsed=elapsed+m.wallSeconds;
    if ~m.ok, messages{end+1}=sprintf('load %.3g: %s',masses(j),m.message); end %#ok<AGROW>
end
if ok, value=mean(values)+0.25*max(values);
else, value=ctx.cfg.failureScore; end
msg=strjoin(messages,' | ');
end

function save_checkpoint(file,result)
folder=fileparts(file); tmp=[tempname(folder) '.mat'];
save(tmp,'result','-v7.3');
movefile(tmp,file,'f'); % only this run's own checkpoint/result is replaced
end

function write_history(folder,r)
n=numel(r.history); theta=vertcat(r.history.theta);
T=table((1:n)',[r.history.ok]',[r.history.score]',[r.history.wallSeconds]', ...
    'VariableNames',{'Trial','Valid','Score','WallSeconds'});
names={'Wz','We','Wv','Wu','Wdu'};
for j=1:5, T.(names{j})=10.^theta(:,j); end
T.Np=theta(:,6); T.Nc=theta(:,7); T.Message={r.history.message}';
writetable(T,fullfile(folder,'trial_history.csv'));
end
