% CHECK_SMC_LYAPUNOV_STABILITY
% Kiem tra cac dieu kien Lyapunov cho bo dieu khien SMC trong urdf_SMC.
%
% Cach chay:
%   1) Chay setup_smc.m.
%   2) Chay check_smc_lyapunov_stability.m.
%
% Mat truot va luat dieu khien dung trong mo hinh:
%   s = de + lambda .* e
%   sdot = -ks .* s - kr .* sat(s ./ phi) + d
%
% Voi V = 0.5*s'*s, khi d = 0:
%   Vdot = -sum(ks_i*s_i^2)
%          -sum(kr_i*s_i*sat(s_i/phi_i)) < 0, s ~= 0.
%
% Khi co tai ngoai, d la gia toc nhieu quy doi:
%   tau_d = J(q)' * wrench
%   d     = M(q)^(-1) * tau_d.
%
% Script nay kiem tra:
%   - On dinh danh dinh cua dong hoc mat truot.
%   - Dieu kien dua s vao lop bien khi tai ngoai bi chan.
%   - Can tren bao thu cho |s| va |e| trong lop bien.
%
% Gia thiet cua chung minh:
%   - Inverse Dynamics dung dung mo hinh M, C, g cua plant.
%   - Mo-men dieu khien khong bi bao hoa.
%   - Tai ngoai la nhieu matched, tac dong qua kenh mo-men/gia toc khop.
%   - Kiem tra tai duoc thuc hien doc theo quy dao tham chieu.

requiredVariables = { ...
    'robot', 'q_full', 't_full', 'lambda', 'ks', 'kr', 'phi'};

for iVar = 1:numel(requiredVariables)
    variableName = requiredVariables{iVar};
    if ~exist(variableName, 'var')
        error(['Thieu bien %s. Hay chay setup_smc.m truoc, sau do ', ...
               'chay check_smc_lyapunov_stability.m.'], variableName);
    end
end

t_full = t_full(:);
lambda = lambda(:);
ks = ks(:);
kr = kr(:);
phi = phi(:);

N = numel(t_full);
n = size(q_full, 2);

if n ~= 3 || size(q_full, 1) ~= N
    error('q_full phai co kich thuoc N-by-3 va khop voi t_full.');
end

if any(diff(t_full) <= 0)
    error('t_full phai tang nghiem ngat.');
end

if any([numel(lambda), numel(ks), numel(kr), numel(phi)] ~= n)
    error('lambda, ks, kr va phi phai la cac vector 3 phan tu.');
end

if any(lambda <= 0) || any(ks <= 0) || any(kr < 0) || any(phi <= 0)
    error(['Can lambda > 0, ks > 0, kr >= 0 va phi > 0 ', ...
           'cho tat ca cac khop.']);
end

% Lay luc tai da duoc setup_smc.m tao. Neu khong co tai thi dung vector 0.
if exist('ts_Fload', 'var') && size(ts_Fload.Data, 1) == N ...
        && size(ts_Fload.Data, 2) == 3
    forceWorld = ts_Fload.Data;
elseif exist('Fload_array', 'var') && isequal(size(Fload_array), [N, 3])
    forceWorld = Fload_array;
else
    forceWorld = zeros(N, 3);
    warning('Khong tim thay ts_Fload hop le; dang kiem tra truong hop khong tai.');
end

if exist('toolTipName', 'var')
    eeBody = toolTipName;
else
    eeBody = 'tool_tip';
end

disturbanceTorque = zeros(N, n);
disturbanceAcceleration = zeros(N, n);

fprintf('\nDang quy doi tai ngoai thanh nhieu gia toc khop...\n');

for k = 1:N
    q_k = q_full(k, :)';
    M_k = massMatrix(robot, q_k);
    J_k = geometricJacobian(robot, q_k, eeBody);

    % geometricJacobian dung thu tu [omega; linear velocity], vi vay
    % wrench tuong ung co thu tu [moment; force].
    wrench_k = [zeros(3, 1); forceWorld(k, :)'];
    tau_d_k = J_k.' * wrench_k;
    d_k = M_k \ tau_d_k;

    disturbanceTorque(k, :) = tau_d_k.';
    disturbanceAcceleration(k, :) = d_k.';
end

maxDisturbance = max(abs(disturbanceAcceleration), [], 1).';

% Ngoai lop bien |s_i| >= phi_i:
% Vdot_i <= -|s_i|*(ks_i*|s_i| + kr_i - dbar_i).
% Tai bien |s_i| = phi_i, dieu kien huong vao trong la:
%   ks_i*phi_i + kr_i - dbar_i > 0.
boundaryMargin = ks .* phi + kr - maxDisturbance;

% Dieu kien kinh dien cua SMC dung sign(s), bao thu hon khi co ks:
%   kr_i > dbar_i.
idealSwitchingMargin = kr - maxDisturbance;

% Can ultimate component-wise cua s. Neu lop bien la bat bien, can nam
% ben trong lop saturation. Neu nhieu lon hon, dung can o mien ngoai.
ultimateSBound = zeros(n, 1);
insideBoundary = false(n, 1);

for i = 1:n
    if maxDisturbance(i) <= ks(i)*phi(i) + kr(i)
        ultimateSBound(i) = maxDisturbance(i) ...
            / (ks(i) + kr(i)/phi(i));
        insideBoundary(i) = ultimateSBound(i) <= phi(i);
    else
        ultimateSBound(i) = (maxDisturbance(i) - kr(i))/ks(i);
        insideBoundary(i) = false;
    end
end

% Tu edot = -lambda.*e + s:
% limsup |e_i| <= limsup |s_i|/lambda_i.
ultimateErrorBound = ultimateSBound ./ lambda;

nominalPass = all(lambda > 0) && all(ks > 0) ...
    && all(kr >= 0) && all(phi > 0);
boundaryLayerPass = all(boundaryMargin > 0);

if nominalPass
    nominalStatus = 'PASS';
else
    nominalStatus = 'FAIL';
end

if boundaryLayerPass
    disturbanceStatus = 'PASS';
else
    disturbanceStatus = 'NOT ESTABLISHED';
end

fprintf('\n========== KET QUA LYAPUNOV SMC ==========\n');
fprintf('min(lambda)                  = %.6e 1/s\n', min(lambda));
fprintf('min(ks)                      = %.6e 1/s\n', min(ks));
fprintf('Nominal sliding stability    : %s\n', nominalStatus);
fprintf('Vdot nominal bound           : Vdot <= -%.6e V\n', 2*min(ks));

fprintf('\nGia toc nhieu cuc dai |d_i| [rad/s^2]:\n');
disp(maxDisturbance.');
fprintf('Bien kr - dbar [rad/s^2]:\n');
disp(idealSwitchingMargin.');
fprintf('Bien ks*phi + kr - dbar [rad/s^2]:\n');
disp(boundaryMargin.');
fprintf('Robust boundary-layer check  : %s\n', disturbanceStatus);

fprintf('\nCan ultimate bao thu cua |s_i| [rad/s]:\n');
disp(ultimateSBound.');
fprintf('Can ultimate bao thu cua |e_i| [rad]:\n');
disp(ultimateErrorBound.');

if nominalPass
    fprintf(['KET LUAN DANH DINH: V=0.5*s''*s co Vdot < 0 voi s ~= 0; ', ...
             's hoi tu ve 0. Vi lambda > 0, e va de cung hoi tu ve 0.\n']);
end

if boundaryLayerPass
    fprintf(['KET LUAN CO TAI: tai uoc luong doc quy dao khong pha vo ', ...
             'dieu kien dua s vao lop bien. Voi saturation, he dat ', ...
             'on dinh thuc hanh va sai so bi chan.\n']);
else
    fprintf(['KET LUAN CO TAI: chua chung minh duoc lop bien la bat bien ', ...
             'tai tat ca cac khop. Dieu nay khong tu dong co nghia he ', ...
             'mat on dinh; can tang gain hoac danh gia bound chat hon.\n']);
end

figure('Name', 'SMC Lyapunov stability check', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile;
plot(t_full, abs(disturbanceAcceleration), 'LineWidth', 1.2);
grid on;
xlabel('Time (s)'); ylabel('|d_i| (rad/s^2)');
title('Matched disturbance acceleration');
legend('|d_1|', '|d_2|', '|d_3|', 'Location', 'best');

nexttile;
marginTime = (ks.*phi + kr).' - abs(disturbanceAcceleration);
plot(t_full, marginTime, 'LineWidth', 1.2);
yline(0, 'k--'); grid on;
xlabel('Time (s)'); ylabel('margin (rad/s^2)');
title('Boundary-layer reaching margin');
legend('joint 1', 'joint 2', 'joint 3', 'Location', 'best');

nexttile;
bar([ultimateSBound, phi]);
grid on;
xlabel('Joint'); ylabel('rad/s');
title('Ultimate bound of s versus phi');
legend('|s| bound', 'phi', 'Location', 'best');

nexttile;
bar(ultimateErrorBound);
grid on;
xlabel('Joint'); ylabel('rad');
title('Conservative ultimate position-error bound');

smcLyapunovResults = struct;
smcLyapunovResults.time = t_full;
smcLyapunovResults.disturbanceTorque = disturbanceTorque;
smcLyapunovResults.disturbanceAcceleration = disturbanceAcceleration;
smcLyapunovResults.maxDisturbance = maxDisturbance;
smcLyapunovResults.idealSwitchingMargin = idealSwitchingMargin;
smcLyapunovResults.boundaryMargin = boundaryMargin;
smcLyapunovResults.ultimateSBound = ultimateSBound;
smcLyapunovResults.ultimateErrorBound = ultimateErrorBound;
smcLyapunovResults.insideBoundary = insideBoundary;
smcLyapunovResults.nominalPass = nominalPass;
smcLyapunovResults.boundaryLayerPass = boundaryLayerPass;