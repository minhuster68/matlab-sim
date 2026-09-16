function [obj,p] = controller(ctx,theta)
%CONTROLLER A fresh object per run: never carry estimator state across trials.
assert(numel(theta)==7 && all(isfinite(theta)),'Expected seven finite parameters.');
w=10.^theta(1:5); Np=round(theta(6)); Nc=round(theta(7));
assert(Np>=2 && Nc>=1 && Nc<=Np,'Require 1 <= Nc <= Np.');
p.Qz=w(1)*ones(1,3); p.Qe=w(2)*ctx.cfg.positionShape;
p.Qed=w(3)*ones(1,3); p.R=w(4)*ones(1,3); p.Rdu=w(5)*ones(1,3);
p.Np=Np; p.Nc=Nc;
obj=mpc(ctx.sys,ctx.Ts,Np,Nc);
obj.Weights.OutputVariables=[p.Qz p.Qe p.Qed];
obj.Weights.ManipulatedVariables=p.R;
obj.Weights.ManipulatedVariablesRate=p.Rdu;
obj.Model.Nominal.X=zeros(9,1); obj.Model.Nominal.Y=zeros(9,1);
obj.Model.Nominal.U=zeros(3,1); obj.Model.Nominal.DX=zeros(9,1);
for j=1:9
    obj.OV(j).ScaleFactor=ctx.cfg.outputScales(j);
end
for j=1:3
    obj.MV(j).ScaleFactor=ctx.cfg.tauMax(j);
    obj.MV(j).Min=ctx.fbMin(j); obj.MV(j).Max=ctx.fbMax(j);
    obj.MV(j).MinECR=0; obj.MV(j).MaxECR=0;
    obj.MV(j).RateMin=-ctx.cfg.fbSlewMax(j)*ctx.Ts;
    obj.MV(j).RateMax= ctx.cfg.fbSlewMax(j)*ctx.Ts;
    obj.MV(j).RateMinECR=0; obj.MV(j).RateMaxECR=0;
end
% Retain the built-in estimator, initialized with a physical nominal model.
end
