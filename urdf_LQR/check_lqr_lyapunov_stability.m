% CHECK_LQR_LYAPUNOV_STABILITY
% Danh gia on dinh Lyapunov cua bo dieu khien LQR gain-scheduling.
%
% Cach chay:
%   1) Chay setup_lqr.m.
%   2) Chay file nay.
%
% Script su dung cac bien do setup_lqr.m tao ra trong workspace, tu dung
% lai A(t), B(t), tinh nghiem Riccati P(t), sau do kiem tra:
%   - On dinh frozen-time: Acl(t) = A(t) - B(t)K(t) la Hurwitz.
%   - Chung nhan LTV: P(t) > 0 va
%                     W(t) = Q + K(t)'R K(t) - Pdot(t) > 0.
%
% Luu y: day la chung nhan cho mo hinh sai so tuyen tinh hoa doc theo quy
% dao. Ket luan cho plant phi tuyen la cuc bo quanh quy dao tham chieu.

requiredVariables = {'robot', 'q_full', 'qd_full', 't_full', 'Q', 'R'};
for iVar = 1:numel(requiredVariables)
    variableName = requiredVariables{iVar};
    if ~exist(variableName, 'var')
        error(['Thieu bien %s. Hay chay setup_lqr.m truoc, sau do ', ...
               'chay check_lqr_lyapunov_stability.m.'], variableName);
    end
end

t_full = t_full(:);
N = numel(t_full);
n = size(q_full, 2);
nx = 3*n;

if n ~= 3
    error('Script hien tai duoc viet cho tay may 3-DOF.');
end

if size(q_full, 1) ~= N || ~isequal(size(qd_full), size(q_full))
    error('Kich thuoc q_full, qd_full va t_full khong phu hop.');
end

if any(diff(t_full) <= 0)
    error('t_full phai tang nghiem ngat.');
end

if ~isequal(size(Q), [nx nx]) || ~isequal(size(R), [n n])
    error('Kich thuoc Q hoac R khong dung voi trang thai 9 va 3 dau vao.');
end

if min(real(eig((Q + Q')/2))) <= 0 || min(real(eig((R + R')/2))) <= 0
    error('Chung minh nay yeu cau Q va R xac dinh duong.');
end

% Dung cung buoc sai phan voi setup_lqr.m neu bien delta dang ton tai.
if exist('delta', 'var') && isscalar(delta) && delta > 0
    deltaLin = delta;
else
    deltaLin = 1e-5;
end

fprintf('\nDang kiem tra Lyapunov tai %d diem tren quy dao...\n', N);

K_lyap = zeros(n, nx, N);
P_array = zeros(nx, nx, N);
Acl_array = zeros(nx, nx, N);
closedLoopPoles = complex(zeros(nx, N));
careResidual = zeros(N, 1);

for k = 1:N
    q_k = q_full(k, :)';
    qd_k = qd_full(k, :)';

    M_k = massMatrix(robot, q_k);
    M_inv = M_k \ eye(n);  % On dinh so hon inv(M_k)

    Astiff = zeros(n, n);
    Adamp = zeros(n, n);

    for i = 1:n
        q_p = q_k;
        q_m = q_k;
        q_p(i) = q_p(i) + deltaLin;
        q_m(i) = q_m(i) - deltaLin;
        Astiff(:, i) = ...
            (gravityTorque(robot, q_p) - gravityTorque(robot, q_m)) ...
            / (2*deltaLin);

        qd_p = qd_k;
        qd_m = qd_k;
        qd_p(i) = qd_p(i) + deltaLin;
        qd_m(i) = qd_m(i) - deltaLin;
        Adamp(:, i) = ...
            (velocityProduct(robot, q_k, qd_p) ...
            - velocityProduct(robot, q_k, qd_m)) / (2*deltaLin);
    end

    A_k = [zeros(n), eye(n),   zeros(n);
           zeros(n), zeros(n), eye(n);
           zeros(n), -M_inv*Astiff, -M_inv*Adamp];

    B_k = [zeros(n);
           zeros(n);
           M_inv];

    [K_k, P_k, poles_k] = lqr(A_k, B_k, Q, R);
    P_k = (P_k + P_k')/2;
    Acl_k = A_k - B_k*K_k;

    K_lyap(:, :, k) = K_k;
    P_array(:, :, k) = P_k;
    Acl_array(:, :, k) = Acl_k;
    closedLoopPoles(:, k) = poles_k;

    riccatiResidual = Acl_k'*P_k + P_k*Acl_k ...
        + Q + K_k'*R*K_k;
    careResidual(k) = norm(riccatiResidual, 'fro') ...
        / max(1, norm(Q + K_k'*R*K_k, 'fro'));
end

% Tinh dao ham Pdot(t) theo tung phan tu. gradient ho tro ca time vector
% khong deu va dung sai phan mot phia tai hai dau quang thoi gian.
Pdot_array = zeros(size(P_array));
for i = 1:nx
    for j = 1:nx
        Pij = squeeze(P_array(i, j, :));
        Pdot_array(i, j, :) = reshape(gradient(Pij, t_full), 1, 1, N);
    end
end

minEigP = zeros(N, 1);
maxEigP = zeros(N, 1);
minEigW = zeros(N, 1);
maxRealPole = zeros(N, 1);
decayRate = zeros(N, 1);

for k = 1:N
    P_k = (P_array(:, :, k) + P_array(:, :, k)')/2;
    Pdot_k = (Pdot_array(:, :, k) + Pdot_array(:, :, k)')/2;
    K_k = K_lyap(:, :, k);
    Acl_k = Acl_array(:, :, k);

    % Vdot = -x' W(t) x.
    W_k = Q + K_k'*R*K_k - Pdot_k;
    W_k = (W_k + W_k')/2;

    eigP = real(eig(P_k));
    eigW = real(eig(W_k));

    minEigP(k) = min(eigP);
    maxEigP(k) = max(eigP);
    minEigW(k) = min(eigW);
    maxRealPole(k) = max(real(eig(Acl_k)));
    decayRate(k) = minEigW(k)/maxEigP(k);
end

% Dung tolerance chi de tranh ket luan sai do nhiễu tinh toan rat nho.
tol = 1e-8;
isPPositive = min(minEigP) > tol;
isFrozenStable = max(maxRealPole) < -tol;
isWPositive = min(minEigW) > tol;
isLTVCertified = isPPositive && isWPositive;

if isPPositive
    pStatus = 'PASS';
else
    pStatus = 'FAIL';
end

if isFrozenStable
    frozenStatus = 'PASS';
else
    frozenStatus = 'FAIL';
end

if isLTVCertified
    ltvStatus = 'PASS';
else
    ltvStatus = 'NOT ESTABLISHED';
end

gainMismatch = NaN;
if exist('K_array', 'var') && isequal(size(K_array), size(K_lyap))
    gainMismatch = max(abs(K_array(:) - K_lyap(:)));
end

fprintf('\n========== KET QUA ON DINH LQR ==========\n');
fprintf('min lambda(P)             = %.6e\n', min(minEigP));
fprintf('min lambda(W)             = %.6e\n', min(minEigW));
fprintf('max Re(lambda(Acl))        = %.6e\n', max(maxRealPole));
fprintf('max normalized CARE error = %.6e\n', max(careResidual));
fprintf('min decay-rate bound      = %.6e 1/s\n', min(decayRate));
if isfinite(gainMismatch)
    fprintf('max |K_setup - K_check|    = %.6e\n', gainMismatch);
end
fprintf('P(t) positive definite    : %s\n', pStatus);
fprintf('Frozen-time stable        : %s\n', frozenStatus);
fprintf('LTV Lyapunov certificate  : %s\n', ltvStatus);

if isLTVCertified
    fprintf(['KET LUAN: V(x,t)=x''P(t)x chung minh mo hinh sai so LTV ', ...
             'on dinh mu deu doc theo quy dao.\n']);
elseif isFrozenStable
    fprintf(['KET LUAN: moi mo hinh frozen-time deu on dinh, nhung P(t) ', ...
             'nay chua tao duoc chung nhan Lyapunov cho toan he LTV.\n']);
    fprintf(['Dieu nay KHONG dong nghia he LTV mat on dinh; can tim mot ', ...
             'ham Lyapunov khac hoac mot ma tran P chung.\n']);
else
    fprintf(['KET LUAN: co it nhat mot diem ma mo hinh dong vong ', ...
             'frozen-time khong on dinh.\n']);
end

figure('Name', 'LQR Lyapunov stability check', 'Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile;
plot(t_full, minEigP, 'LineWidth', 1.3);
yline(0, 'k--'); grid on;
xlabel('Time (s)'); ylabel('min eig(P)');
title('P(t) positive definite');

nexttile;
plot(t_full, minEigW, 'LineWidth', 1.3);
yline(0, 'k--'); grid on;
xlabel('Time (s)'); ylabel('min eig(W)');
title('LTV Lyapunov condition');

nexttile;
plot(t_full, maxRealPole, 'LineWidth', 1.3);
yline(0, 'k--'); grid on;
xlabel('Time (s)'); ylabel('max real pole');
title('Frozen-time closed-loop poles');

nexttile;
semilogy(t_full, max(careResidual, eps), 'LineWidth', 1.3);
grid on;
xlabel('Time (s)'); ylabel('normalized residual');
title('CARE residual');

lyapunovResults = struct;
lyapunovResults.time = t_full;
lyapunovResults.K = K_lyap;
lyapunovResults.P = P_array;
lyapunovResults.Pdot = Pdot_array;
lyapunovResults.Acl = Acl_array;
lyapunovResults.closedLoopPoles = closedLoopPoles;
lyapunovResults.minEigP = minEigP;
lyapunovResults.minEigW = minEigW;
lyapunovResults.maxRealPole = maxRealPole;
lyapunovResults.careResidual = careResidual;
lyapunovResults.decayRate = decayRate;
lyapunovResults.gainMismatch = gainMismatch;
lyapunovResults.isFrozenStable = isFrozenStable;
lyapunovResults.isLTVCertified = isLTVCertified;
