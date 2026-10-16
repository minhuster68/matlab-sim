function [v, s, dz] = sta_core(e, de, ddqRef, lambdaSTA, k1STA, k2STA, z)
%#codegen
% Sampled super-twisting law, evaluated at TsSTA sample hits.
% Error convention MUST be actual minus reference.
% External Unit Delay stores z[k]. Forward Euler computes
% z[k+1] = z[k] + TsSTA*dz[k], once per controller sample.
s = de + lambdaSTA .* e;
sigma = sign(s);
v = ddqRef - lambdaSTA .* de ...
    - k1STA .* sqrt(abs(s)) .* sigma + z;
dz = -k2STA .* sigma;
end
