function s=signature(ctx)
%SIGNATURE Exact, portable experiment identity. Do not reuse under other limits.
s.version=ctx.cfg.version;
s.t=ctx.t; s.q=ctx.q; s.dq=ctx.dq; s.ddq=ctx.ddq;
s.ff=ctx.ff; s.A0=ctx.A0; s.B0=ctx.B0;
s.tauMax=ctx.cfg.tauMax; s.fbSlewMax=ctx.cfg.fbSlewMax;
s.gravity=ctx.cfg.gravity; s.toolOffset=ctx.cfg.toolOffset;
s.outputScales=ctx.cfg.outputScales; s.positionShape=ctx.cfg.positionShape;
s.delta=ctx.cfg.linearizationDelta;
s.urdfText=fileread(fullfile(ctx.cfg.rootDir,'urdf.urdf'));
% Byte comparison includes plant/source wiring, not only controller dimensions.
fid=fopen(ctx.sourceModel,'rb');
assert(fid~=-1,'Cannot read source model identity.');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
s.modelBytes=fread(fid,inf,'*uint8');
end
