function ctx = prepare_model(ctx)
%PREPARE_MODEL Modify a unique copy, never the user's source SLX.
root=fullfile(ctx.cfg.rootDir,'tuning_runs');
if ~isfolder(root), mkdir(root); end
ctx.runDir=tempname(root); mkdir(ctx.runDir);
[~,token]=fileparts(ctx.runDir);
mdl=matlab.lang.makeValidName(['mpc_tune_' token]);
ctx.model=mdl; ctx.modelFile=fullfile(ctx.runDir,[mdl '.slx']);
copyfile(ctx.sourceModel,ctx.modelFile);
load_system(ctx.modelFile);
assert(strcmp(get_param(mdl,'SimulationStatus'),'stopped'),'Model must be stopped.');

% Fail clearly if the source has changed instead of guessing new wiring.
required={'Adaptive MPC Controller','MATLAB Function','Inverse Dynamics', ...
    'Mux','Mux1','Mux2','Sum','Sum1','Sum2','Demux','Plant_new'};
for k=1:numel(required)
    assert(getSimulinkBlockHandle([mdl '/' required{k}])>0, ...
        'mpctune:Topology','Source model lacks expected block: %s.',required{k});
end
mp=[mdl '/Adaptive MPC Controller'];
assert(strcmp(get_param(mp,'state_inport'),'off'), ...
    'mpctune:Topology','Expected built-in state estimation, not a custom state port.');
ph=get_param(mp,'PortHandles');
assert(numel(ph.Inport)==3 && numel(ph.Outport)==1, ...
    'mpctune:Topology','Expected three MPC inputs and one output in source model.');
must_connect(mdl,'Mux',1,'Sum',1);
must_connect(mdl,'Mux1',1,'Sum1',1);
must_connect(mdl,'Mux2',1,'Adaptive MPC Controller',2);
must_connect(mdl,'Adaptive MPC Controller',1,'Sum2',2);
assert(strcmp(get_param([mdl '/Sum'],'Inputs'),'|+-') && ...
       strcmp(get_param([mdl '/Sum1'],'Inputs'),'|+-'), ...
       'mpctune:ErrorSign','Expected e = actual - reference.');
for cb={'InitFcn','StartFcn','StopFcn','PreLoadFcn','PostLoadFcn'}
    assert(isempty(strtrim(get_param(mdl,cb{1}))), ...
        'mpctune:Callback','Source has a %s callback; review it before tuning.',cb{1});
end
% Ongoing solver/estimator state must not leak between trials.
set_param(mdl,'FastRestart','off','SimulationMode','normal', ...
    'StartTime','0','StopTime',num2str(ctx.t(end),17), ...
    'SolverType','Variable-step','Solver','ode15s', ...
    'MaxStep',num2str(ctx.Ts/2,17),'RelTol',num2str(ctx.cfg.solverRelTol), ...
    'AbsTol',num2str(ctx.cfg.solverAbsTol),'ReturnWorkspaceOutputs','on', ...
    'LoadInitialState','off','SaveFinalState','off','SaveState','off', ...
    'SignalLogging','off','SaveOutput','off');
set_param(mp,'return_qpstatus','on','updateMPCinCallerWorkspace','off');
ph=get_param(mp,'PortHandles');
assert(numel(ph.Outport)==2,'Expected mv and qp.status outputs.');

% Replace repeated reference-only linearization with precomputed A/B samples.
old=[mdl '/MATLAB Function']; ports=get_param(old,'PortHandles');
assert(numel(ports.Outport)==8,'Expected A,B,C,D,U,Y,X,DX.');
dst=cell(1,8);
for k=1:8, dst{k}=destinations(ports.Outport(k)); end
disconnect_and_delete(old);
names={'A','B','C','D','U','Y','X','DX'};
expr={'mpc_A','mpc_B','eye(9)','zeros(9,3)', ...
      'zeros(3,1)','zeros(9,1)','zeros(9,1)','zeros(9,1)'};
for k=1:8
    block=[mdl '/Cached_' names{k}];
    pos=[1130 350+45*k 1250 375+45*k];
    if k<=2
        add_source(block,expr{k},pos);
    else
        % Adaptive MPC model ports require true 2-D matrix signals.  With
        % the Constant block default, an N-by-1 value is compiled as a
        % one-dimensional vector and Model.U/Y/X/DX rejects it.
        add_block('simulink/Sources/Constant',block,'Value',expr{k}, ...
            'VectorParams1D','off','Position',pos);
    end
    p=get_param(block,'PortHandles');
    for j=1:numel(dst{k}), add_line(mdl,p.Outport(1),dst{k}(j),'autorouting','on'); end
    lh=get_param(p.Outport(1),'Line'); set_param(lh,'Name',names{k});
end

% Same unloaded inverse dynamics, but computed once and applied with ZOH.
% No live persistent robot or evalin remains inside a MATLAB Function.
old=[mdl '/Inverse Dynamics']; p=get_param(old,'PortHandles');
dst=destinations(p.Outport(1)); disconnect_and_delete(old);
add_source([mdl '/Cached_Feedforward'],'mpc_tau_ff',[1090 2 1240 55]);
p=get_param([mdl '/Cached_Feedforward'],'PortHandles');
for j=1:numel(dst), add_line(mdl,p.Outport(1),dst(j),'autorouting','on'); end

integrators=find_system(mdl,'SearchDepth',1,'BlockType','DiscreteIntegrator');
assert(numel(integrators)==1,'Expected one external error integrator.');
set_param(integrators{1},'SampleTime','Ts', ...
    'IntegratorMethod','Integration: Forward Euler','InitialCondition','[0 0 0]');
sources=find_system(mdl,'LookUnderMasks','none','BlockType','FromWorkspace');
for k=1:numel(sources)
    set_param(sources{k},'SampleTime','Ts','Interpolate','off', ...
        'OutputAfterFinalValue','Holding final value');
end
% Catch accidental branch deletion when removing the two original blocks.
must_connect(mdl,'Mux',1,'Sum',1);
must_connect(mdl,'Mux1',1,'Sum1',1);
assert_input_variable([mdl '/Sum'],2,'ts_q');
assert_input_variable([mdl '/Sum1'],2,'ts_qd');

% Simscape reports SourceType differently across MATLAB releases.  These
% paths are part of the source-model topology already checked above, so use
% the stable block names instead of matching a release-specific SourceType.
grav=[mdl '/Plant_new/MechanismConfiguration'];
assert_block_parameter(grav,'GravityVector');
set_param(grav,'GravityVector','mpc_gravity');
toolTransform=[mdl '/Plant_new/Rigid Transform2'];
assert_block_parameter(toolTransform,'TranslationCartesianOffset');
set_param(toolTransform, ...
    'TranslationCartesianOffset','mpc_tool_offset');
% Mesh paths in the original model are relative. Make the copy portable
% between Current Folders within this project.
blocks=find_system(mdl,'LookUnderMasks','all','FollowLinks','off','Type','Block');
for k=1:numel(blocks)
    dp=get_param(blocks{k},'DialogParameters');
    if isstruct(dp) && isfield(dp,'ExtGeomFileName')
        mesh=get_param(blocks{k},'ExtGeomFileName');
        [~,name,ext]=fileparts(mesh);
        mesh=fullfile(ctx.cfg.rootDir,'meshes',[name ext]);
        assert(isfile(mesh),'Missing mesh: %s.',mesh);
        set_param(blocks{k},'ExtGeomFileName',mesh);
    end
end

% Backup actuator saturation is not used to hide infeasible candidates:
% command vs applied is logged, and any clipping causes candidate rejection.
sat=[mdl '/Total_Torque_Limit'];
if getSimulinkBlockHandle(sat)>0
    % The normal runtime model may already contain this safety block.  Reuse
    % it so the same urdf.slx can serve runtime and autotuning workflows.
    assert(strcmp(get_param(sat,'BlockType'),'Saturate'), ...
        'mpctune:Topology', ...
        'Existing Total_Torque_Limit must be a Simulink Saturation block.');
    assert_block_parameter(sat,'UpperLimit');
    assert_block_parameter(sat,'LowerLimit');
    set_param(sat,'UpperLimit','mpc_tau_limit', ...
        'LowerLimit','-mpc_tau_limit');
    must_connect(mdl,'Sum2',1,'Total_Torque_Limit',1);
    sp=get_param(sat,'PortHandles');
    destinations(sp.Outport(1)); % assert that the limiter still drives the plant
else
    old=[mdl '/Sum2']; p=get_param(old,'PortHandles');
    lh=get_param(p.Outport(1),'Line'); dst=destinations(p.Outport(1));
    delete_line(lh);
    add_block('simulink/Discontinuities/Saturation',sat, ...
        'UpperLimit','mpc_tau_limit','LowerLimit','-mpc_tau_limit', ...
        'Position',[1570 35 1630 75]);
    sp=get_param(sat,'PortHandles'); add_line(mdl,p.Outport(1),sp.Inport(1));
    for j=1:numel(dst)
        add_line(mdl,sp.Outport(1),dst(j),'autorouting','on');
    end
end

log_output(mdl,'Mux',1,'tune_q',780);
log_output(mdl,'Mux1',1,'tune_dq',820);
log_output(mdl,'Mux2',1,'tune_x',860);
log_output(mdl,'Sum2',1,'tune_tau_command',900);
log_output(mdl,'Total_Torque_Limit',1,'tune_tau',940);
log_output(mdl,'Adaptive MPC Controller',1,'tune_fb',980);
log_output(mdl,'Adaptive MPC Controller',2,'tune_qp',1020);
scopes=find_system(mdl,'LookUnderMasks','none','BlockType','Scope');
for k=1:numel(scopes)
    set_param(scopes{k},'OpenAtSimulationStart','off');
end
% Explicitly set physical joint initial states, consistent with source q=dq=0.
% Use the URDF joint names because SourceType is release dependent here too.
joints={
    [mdl '/Plant_new/base_joint']
    [mdl '/Plant_new/shoulder_joint']
    [mdl '/Plant_new/elbow_joint']};
for k=1:3
    assert_block_parameter(joints{k},'PositionTargetSpecify');
    assert_block_parameter(joints{k},'VelocityTargetSpecify');
    set_param(joints{k},'PositionTargetSpecify','on','PositionTargetValue','0', ...
        'PositionTargetValueUnits','rad','VelocityTargetSpecify','on', ...
        'VelocityTargetValue','0','VelocityTargetValueUnits','rad/s');
end
save_system(mdl,ctx.modelFile);
fprintf('Prepared copy: %s\n',ctx.modelFile);
end

function assert_block_parameter(block,param)
assert(getSimulinkBlockHandle(block)>0, ...
    'mpctune:Topology','Source model lacks expected block: %s.',block);
dp=get_param(block,'DialogParameters');
assert(isstruct(dp) && isfield(dp,param), ...
    'mpctune:Topology','Block %s lacks expected parameter %s.',block,param);
end

function add_source(path,variable,pos)
add_block('simulink/Sources/From Workspace',path,'VariableName',variable, ...
    'SampleTime','Ts','Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value','Position',pos);
end

function dst=destinations(port)
lh=get_param(port,'Line');
assert(lh~=-1,'Expected connected output port.');
dst=get_param(lh,'DstPortHandle'); dst=dst(dst~=-1);
assert(~isempty(dst),'Expected at least one destination.');
end

function disconnect_and_delete(block)
p=get_param(block,'PortHandles');
% Delete only the requested source/destination branch, preserving references.
for k=1:numel(p.Inport)
    lh=get_param(p.Inport(k),'Line');
    if lh~=-1
        src=source_port(lh);
        assert(src~=-1,'Cannot resolve incoming branch source.');
        delete_line(get_param(block,'Parent'),src,p.Inport(k));
    end
end
for k=1:numel(p.Outport)
    lh=get_param(p.Outport(k),'Line');
    if lh~=-1, delete_line(lh); end
end
delete_block(block);
end

function must_connect(mdl,src,sp,dst,dp)
a=get_param([mdl '/' src],'PortHandles');
b=get_param([mdl '/' dst],'PortHandles');
lh=get_param(b.Inport(dp),'Line');
assert(lh~=-1 && source_port(lh)==a.Outport(sp), ...
    'mpctune:Topology','Unexpected connection to %s input %d.',dst,dp);
end

function assert_input_variable(dst,port,variable)
p=get_param(dst,'PortHandles'); lh=get_param(p.Inport(port),'Line');
assert(lh~=-1,'Reference disconnected.');
src=get_param(source_port(lh),'Parent');
assert(strcmp(get_param(src,'VariableName'),variable),'Wrong reference source.');
end

function src=source_port(lh)
src=get_param(lh,'SrcPortHandle');
while src==-1
    lh=get_param(lh,'LineParent');
    assert(lh~=-1 && lh~=0,'Cannot resolve source of a line branch.');
    src=get_param(lh,'SrcPortHandle');
end
end

function log_output(mdl,block,index,variable,y)
path=[mdl '/Log_' variable];
add_block('simulink/Sinks/To Workspace',path,'VariableName',variable, ...
    'SaveFormat','Timeseries','MaxDataPoints','inf','Decimation','1', ...
    'SampleTime','Ts','Position',[1700 y 1830 y+25]);
a=get_param([mdl '/' block],'PortHandles'); b=get_param(path,'PortHandles');
add_line(mdl,a.Outport(index),b.Inport(1),'autorouting','on');
end
