function test_mpc_autotune
%TEST_MPC_AUTOTUNE Static invariants plus a 2-second actual Simulink smoke test.
assert(evalin('base','exist(''mpcTune'',''var'')==1'),'Run setup_mpc first.');
ctx=evalin('base','mpcTune');
ctx.smokePassed=false; assignin('base','mpcTune',ctx);
assert(isequal(size(ctx.A0),[9 9]) && isequal(size(ctx.B0),[9 3]));
assert(norm(ctx.A0)>0 && norm(ctx.B0)>0);
assert(norm(ctx.A0(1:3,:)-[eye(3) ctx.Ts*eye(3) zeros(3)],'fro')<1e-12);
assert(norm(ctx.B0(1:3,:),'fro')<1e-12);
assert(all(ctx.ff+ctx.fbMin>=-ctx.cfg.tauMax-1e-10,'all'));
assert(all(ctx.ff+ctx.fbMax<= ctx.cfg.tauMax+1e-10,'all'));
% Check the finite-difference linearization at a second perturbation size.
[A,B]=mpctune.error_model(ctx.robot,ctx.q(1,:)',ctx.dq(1,:)',ctx.ddq(1,:)', ...
    ctx.Ts,ctx.cfg.linearizationDelta/2);
assert(norm(A-ctx.A0,'fro')<1e-4*max(1,norm(A,'fro')),'Linearization step sensitivity.');
assert(norm(B-ctx.B0,'fro')<1e-4*max(1,norm(B,'fro')),'Input matrix step sensitivity.');
mpctune.test_score;
fprintf('Running two-second smoke test on the prepared model...\n');
[m,~,out]=mpctune.run_trial(ctx,ctx.baseline,ctx.cfg.payloadMass,min(2,ctx.t(end)));
assert(m.ok,'mpctune:SmokeFailure','Smoke test failed:\n%s',m.message);
ctx.smokePassed=true; assignin('base','mpcTune',ctx);
assignin('base','mpcSmokeOutput',out);
fprintf('PASS. RMSE q=%s rad; RMSE dq=%s rad/s; wall %.1f s.\n', ...
    mat2str(m.rmseQ,4),mat2str(m.rmseDq,4),m.wallSeconds);
fprintf('This is a smoke test, not full-trajectory validation.\n');
end
