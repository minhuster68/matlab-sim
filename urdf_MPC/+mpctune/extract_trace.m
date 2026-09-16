function trace = extract_trace(out,ctx,stopTime)
%EXTRACT_TRACE Align complete, finite logs; reject missing/truncated outputs.
grid=ctx.t(ctx.t<=stopTime+1e-9);
assert(numel(grid)>1,'Not enough samples to score.');
trace.t=grid;
map={'q','tune_q',3; 'dq','tune_dq',3; 'x','tune_x',9; ...
    'tau','tune_tau',3; 'command','tune_tau_command',3; ...
    'fb','tune_fb',3; 'qp','tune_qp',1};
for k=1:size(map,1)
    ts=out.get(map{k,2});
    assert(isa(ts,'timeseries'),'Missing timeseries: %s.',map{k,2});
    time=double(ts.Time(:)); data=double(ts.Data);
    if ~ts.IsTimeFirst
        data=permute(data,[ndims(data) 1:ndims(data)-1]);
    end
    data=reshape(data,numel(time),[]);
    assert(size(data,2)==map{k,3},'Unexpected log dimensions: %s.',map{k,2});
    assert(all(isfinite(time)) && all(isfinite(data(:))),'Nonfinite simulation output.');
    [time,ix]=unique(time,'last'); data=data(ix,:);
    assert(numel(time)>1 && time(1)<=grid(1)+1e-8 && time(end)>=grid(end)-1e-8, ...
        'mpctune:Incomplete','Simulation/log ended early (timeout or stop).');
    assert(max(diff(time))<=1.5*ctx.Ts,'Logging contains missing samples.');
    % QP status/torques use sample-and-hold; never interpolate away a failure.
    if any(strcmp(map{k,1},{'tau','command','fb','qp'})), method='previous';
    else, method='linear'; end
    if strcmp(map{k,1},'qp')
        assert(all(data(:)>0),'mpctune:QP','QP status contains zero or negative values.');
    end
    trace.(map{k,1})=interp1(time,data,min(max(grid,time(1)),time(end)),method);
end
trace.qref=interp1(ctx.t,ctx.q,grid,'linear');
trace.dqref=interp1(ctx.t,ctx.dq,grid,'linear');
assert(max(abs(trace.x(:,4:6)-(trace.q-trace.qref)),[],'all')<1e-6, ...
    'mpctune:ErrorWiring','Position error ordering/sign does not match measured state.');
assert(max(abs(trace.x(:,7:9)-(trace.dq-trace.dqref)),[],'all')<1e-6, ...
    'mpctune:ErrorWiring','Velocity error ordering/sign does not match measured state.');
end
