function m = score_trace(s,cfg,Ts)
%SCORE_TRACE Fixed external objective; never use MPC's variable-weight cost.
e=s.q-s.qref; ev=s.dq-s.dqref;
duration=s.t(end)-s.t(1);
assert(duration>0 && all(isfinite([e(:);ev(:);s.tau(:);s.fb(:)])), ...
    'Nonfinite or empty trace.');
m.ok=false; m.score=cfg.failureScore; m.message='';
m.rmseQ=sqrt(trapz(s.t,e.^2,1)/duration);
m.rmseDq=sqrt(trapz(s.t,ev.^2,1)/duration);
m.peakQ=max(abs(e),[],1);
m.peakTau=max(abs(s.tau),[],1);
m.minQP=min(s.qp);
m.maxClip=max(abs(s.command-s.tau),[],'all');
m.peakFeedbackRate=max(abs(diff([zeros(1,3);s.fb],1,1)),[],1)/Ts;
m.peakVelocity=max(abs(s.dq),[],1);
tol=cfg.score.torqueTolerance;
if any(s.qp<=0)
    m.message='QP solver failed or returned a suboptimal status.'; return
elseif m.maxClip>tol
    m.message='Total-torque saturation clipped the requested control.'; return
elseif any(m.peakTau>cfg.tauMax+tol)
    m.message='Total-torque limit exceeded.'; return
elseif any(m.peakFeedbackRate>cfg.fbSlewMax+tol/Ts)
    m.message='Feedback torque rate constraint exceeded.'; return
elseif any(m.peakQ>cfg.score.rejectPositionError) || any(m.peakVelocity>cfg.score.rejectVelocity)
    m.message='Runaway/poor-tracking rejection threshold exceeded.'; return
end
dTau=diff(s.tau,1,1)./cfg.tauMax;
util=s.tau./cfg.tauMax;
terms=[mean((m.rmseQ./cfg.score.positionScale).^2), ...
       mean((m.rmseDq./cfg.score.velocityScale).^2), ...
       mean((m.peakQ./cfg.score.positionScale).^2), ...
       mean(dTau(:).^2), mean(trapz(s.t,util.^2,1)/duration)];
m.terms=terms;
m.score=sum(cfg.score.weights.*terms);
m.ok=isfinite(m.score) && m.score<cfg.failureScore;
if ~m.ok, m.score=cfg.failureScore; m.message='Invalid objective value.'; end
end
