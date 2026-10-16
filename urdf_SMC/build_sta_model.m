function model = build_sta_model()
% Create a separate STA model from this folder's urdf.slx using Simulink API.
% Run setup_sta first. Existing urdf_sta.slx is never silently overwritten.
here = fileparts(mfilename('fullpath'));
model = 'urdf_sta_discrete';
target = fullfile(here,[model '.slx']);
assert(~isfile(target) && ~bdIsLoaded(model), ...
    'urdf_sta_discrete already exists/is open. Use it, or rename it before rebuilding.');
assert(evalin('base','exist(''robot'',''var'') && exist(''k1STA'',''var'')'), ...
    'Run setup_sta in the base workspace first.');
copyfile(fullfile(here,'urdf.slx'), target);
try
    load_system(target);
    old = [model '/SMC Core'];
    ph = get_param(old,'PortHandles');
    assert(numel(ph.Inport)==7 && numel(ph.Outport)==2, ...
        'Unexpected baseline SMC interface.');
    assert(strcmp(get_param([model '/Sum'],'Inputs'),'-+|') && ...
        strcmp(get_param([model '/Sum1'],'Inputs'),'-+|'), ...
        'Check baseline error convention: expected actual minus reference.');
    src = zeros(1,6); dst = cell(1,2);
    for k=1:6
        ln = get_param(ph.Inport(k),'Line');
        src(k) = get_param(ln,'SrcPortHandle');
    end
    for k=1:2
        ln = get_param(ph.Outport(k),'Line');
        dst{k} = get_param(ln,'DstPortHandle');
    end
    pos = get_param(old,'Position');
    % Disconnect only these connections; retain other branches to scopes.
    for k=1:6
        delete_line(model,src(k),ph.Inport(k));
    end
    phiPH = get_param([model '/phi'],'PortHandles');
    delete_line(model,phiPH.Outport(1),ph.Inport(7));
    for k=1:2
        for d=dst{k}(:)'
            delete_line(model,ph.Outport(k),d);
        end
    end
    delete_block(old);
    delete_block([model '/phi']);
    set_param([model '/lambda'],'Value','lambdaSTA');
    set_param([model '/ks'],'Value','k1STA','Name','k1');
    set_param([model '/kr'],'Value','k2STA','Name','k2');

    core = [model '/STA Core'];
    add_block('built-in/SubSystem',core,'Position',pos);
    names = {'e','de','ddqRef','lambdaSTA','k1STA','k2STA'};
    for k=1:6
        add_block('simulink/Sources/In1',[core '/' names{k}], ...
            'Port',num2str(k),'PortDimensions','3', ...
            'Position',[25 30+45*k 55 44+45*k]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [core '/STA law'],'Position',[180 85 370 290]);
    rt = sfroot;
    chart = rt.find('-isa','Stateflow.EMChart','Path',[core '/STA law']);
    assert(numel(chart)==1,'Cannot resolve STA MATLAB Function.');
    chart.Script = fileread(fullfile(here,'sta_core.m'));
    % Fixed column vectors prevent accidental implicit 3x3 expansion.
    data = chart.find('-isa','Stateflow.Data');
    for k=1:numel(data)
        if strcmp(data(k).Scope,'Input') || strcmp(data(k).Scope,'Output')
            data(k).Props.Array.Size = '[3 1]';
        end
    end
    add_block('simulink/Discrete/Unit Delay',[core '/z integral'], ...
        'InitialCondition','z0STA','SampleTime','TsSTA', ...
        'Position',[440 325 480 365]);
    add_block('simulink/Sinks/Out1',[core '/v'], ...
        'Port','1','Position',[470 95 500 109]);
    add_block('simulink/Sinks/Out1',[core '/s'], ...
        'Port','2','Position',[470 165 500 179]);
    drawnow;
    for k=1:6
        add_line(core,[names{k} '/1'],['STA law/' num2str(k)],'autorouting','on');
    end
    add_line(core,'STA law/1','v/1','autorouting','on');
    add_line(core,'STA law/2','s/1','autorouting','on');
    add_block('simulink/Math Operations/Gain',[core '/Ts times dz'], ...
        'Gain','TsSTA','Position',[380 410 440 440]);
    add_block('simulink/Math Operations/Sum',[core '/z next'], ...
        'Inputs','++','Position',[500 405 525 445]);
    add_line(core,'STA law/3','Ts times dz/1','autorouting','on');
    add_line(core,'Ts times dz/1','z next/1','autorouting','on');
    add_line(core,'z integral/1','z next/2','autorouting','on');
    add_line(core,'z next/1','z integral/1','autorouting','on');
    add_line(core,'z integral/1','STA law/7','autorouting','on');
    cp = get_param(core,'PortHandles');
    for k=1:6
        if k <= 3
            holdName = ['STA sample ' num2str(k)];
            add_block('simulink/Discrete/Zero-Order Hold',[model '/' holdName], ...
                'SampleTime','TsSTA','Position',[1050 600+50*k 1100 630+50*k]);
            hp = get_param([model '/' holdName],'PortHandles');
            add_line(model,src(k),hp.Inport(1),'autorouting','on');
            add_line(model,hp.Outport(1),cp.Inport(k),'autorouting','on');
        else
            add_line(model,src(k),cp.Inport(k),'autorouting','on');
        end
    end
    for k=1:2
        for d=dst{k}(:)'
            add_line(model,cp.Outport(k),d,'autorouting','on');
        end
    end
    % Sample actual q,dq into inverse dynamics at the same controller hits.
    idp = get_param([model '/Inverse Dynamics'],'PortHandles');
    for k=1:2
        ln = get_param(idp.Inport(k),'Line');
        sp = get_param(ln,'SrcPortHandle');
        delete_line(model,sp,idp.Inport(k));
        hn = [model '/ID sample ' num2str(k)];
        add_block('simulink/Discrete/Zero-Order Hold',hn, ...
            'SampleTime','TsSTA','Position',[1300 600+50*k 1350 630+50*k]);
        hp = get_param(hn,'PortHandles');
        add_line(model,sp,hp.Inport(1),'autorouting','on');
        add_line(model,hp.Outport(1),idp.Inport(k),'autorouting','on');
    end
    % Explicit hold on torque before every plant/scope/log destination.
    ln = get_param(idp.Outport(1),'Line');
    targets = get_param(ln,'DstPortHandle');
    delete_line(ln);
    hn = [model '/Torque hold'];
    add_block('simulink/Discrete/Zero-Order Hold',hn, ...
        'SampleTime','TsSTA','Position',[1400 820 1460 850]);
    hp = get_param(hn,'PortHandles');
    add_line(model,idp.Outport(1),hp.Inport(1),'autorouting','on');
    for d=targets(:)'
        add_line(model,hp.Outport(1),d,'autorouting','on');
    end
    % Setup is explicit: avoid a copied callback resetting TsSTA/gains.
    cb = get_param(model,'InitFcn');
    if ~isempty(regexp(cb,'setup_(smc|sta)','once'))
        warning('STA:SetupCallback','Removed copied setup InitFcn; run setup_sta before sim.');
        set_param(model,'InitFcn','');
    end
    % Explicit logs, returned inside the SimulationOutput object.
    log_signal(model,'STA Core/1','sta_v',[1520 410 1640 440]);
    log_signal(model,'STA Core/2','sta_s',[1520 460 1640 490]);
    log_signal(model,'Torque hold/1','sta_tau',[1520 510 1640 540]);
    log_signal(model,'Sum/1','sta_e',[1520 560 1640 590]);
    log_signal(core,'z integral/1','sta_z',[530 325 650 355]);
    set_param(model,'StopTime','t_full(end)', ...
        'SolverType','Variable-step','Solver','ode15s', ...
        'MaxStep','maxStepSTA','RelTol','1e-3','AbsTol','1e-5', ...
        'ReturnWorkspaceOutputs','on');
    % Mesh paths in the source model are relative to urdf_SMC.
    oldDir = pwd;
    cleanup = onCleanup(@() cd(oldDir));
    cd(here);
    set_param(model,'SimulationCommand','update');
    save_system(model,target);
    open_system(model);
    fprintf('Built and compiled %s. Run: outSTA = sim(''%s'');\n',target,model);
catch err
    if bdIsLoaded(model), close_system(model,0); end
    if isfile(target), delete(target); end
    rethrow(err);
end
end

function log_signal(system,source,name,position)
add_block('simulink/Sinks/To Workspace',[system '/' name], ...
    'VariableName',name,'SaveFormat','Timeseries', ...
    'MaxDataPoints','inf','Decimation','1','Position',position);
add_line(system,source,[name '/1'],'autorouting','on');
end
