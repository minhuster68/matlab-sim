function test_score
%TEST_SCORE Unit tests independent of robot/MPC; synthetic data only.
cfg=mpctune.defaults(pwd); Ts=0.01;
s.t=(0:Ts:1)'; n=numel(s.t);
s.q=zeros(n,3); s.dq=s.q; s.qref=s.q; s.dqref=s.q;
s.tau=s.q; s.command=s.q; s.fb=s.q; s.qp=ones(n,1);
a=mpctune.score_trace(s,cfg,Ts); assert(a.ok && a.score==0);
s.q(:,1)=cfg.score.positionScale(1);
b=mpctune.score_trace(s,cfg,Ts); assert(b.ok && b.score>a.score);
s.q(:,1)=2*cfg.score.positionScale(1);
c=mpctune.score_trace(s,cfg,Ts); assert(c.ok && abs(c.score-4*b.score)<1e-10);
s.qp(3)=0; d=mpctune.score_trace(s,cfg,Ts); assert(~d.ok);
s.qp(:)=1; s.command(4,1)=0.1;
d=mpctune.score_trace(s,cfg,Ts); assert(~d.ok); % clipping cannot win
s.command(:)=0; s.tau(:,1)=cfg.tauMax(1)+1;
d=mpctune.score_trace(s,cfg,Ts); assert(~d.ok);
s.tau(:)=0; s.fb(1,1)=2*cfg.fbSlewMax(1)*Ts;
d=mpctune.score_trace(s,cfg,Ts); assert(~d.ok);
fprintf('PASS: synthetic scoring and rejection unit tests.\n');
end
