function plot_result(r,folder)
%PLOT_RESULT Baseline vs selected result on the same validation load.
v=r.validation{r.selectedValidation}; b=r.validation{1};
[~,j]=min(abs(r.cfg.validationMasses-r.cfg.payloadMass));
s=v.traces{j}; base=b.traces{j};
if isempty(s), return; end
fig=figure('Name','MPC autotune validation','Color','w');
for k=1:3
    subplot(3,2,2*k-1); hold on; grid on;
    if ~isempty(base), plot(base.t,base.q(:,k)-base.qref(:,k),'--','DisplayName','Baseline'); end
    plot(s.t,s.q(:,k)-s.qref(:,k),'DisplayName','Selected');
    ylabel(sprintf('e_%d (rad)',k)); legend('show');
    subplot(3,2,2*k); hold on; grid on;
    if ~isempty(base), plot(base.t,base.dq(:,k)-base.dqref(:,k),'--','DisplayName','Baseline'); end
    plot(s.t,s.dq(:,k)-s.dqref(:,k),'DisplayName','Selected');
    ylabel(sprintf('de_%d (rad/s)',k)); legend('show');
end
sgtitle(sprintf('Validation: %.2f kg-equivalent external force',r.cfg.validationMasses(j)));
saveas(fig,fullfile(folder,'validation_errors.png'));
fig=figure('Name','MPC total torque validation','Color','w');
for k=1:3
    subplot(3,1,k); hold on; grid on;
    plot(s.t,s.tau(:,k)); yline(r.cfg.tauMax(k),'r--'); yline(-r.cfg.tauMax(k),'r--');
    ylabel(sprintf('tau_%d (N*m)',k));
end
xlabel('Time (s)'); saveas(fig,fullfile(folder,'validation_torque.png'));
end
