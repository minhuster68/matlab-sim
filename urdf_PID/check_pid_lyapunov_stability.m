%% CHECK_PID_LYAPUNOV_STABILITY
% Kiem tra on dinh cuc bo cua bo PID noi tang quanh quy dao tham chieu.
%
% Chay:
%   setup_pid
%   check_pid_lyapunov_stability
%
% Trang thai sai so dung trong chung minh:
%   x = [delta_q; delta_dq; xi]
%   delta_q  = q - q_ref
%   delta_dq = dq - dq_ref
%   dot(xi)  = e_v = -delta_dq - Kpp*delta_q
%
% Bo dieu khien trong urdf.slx:
%   tau_fb = Kt*(Kvp*e_v + Kvi*xi)
%
% Voi mo hinh sai so tuyen tinh hoa dot(x)=Acl(t)x, tai moi thoi diem
% script giai:
%   Acl(t)'*P(t) + P(t)*Acl(t) = -I.
% Sau do kiem tra dieu kien LTV:
%   W(t) = I - dot(P)(t) > 0.
% Neu P va W duong xac dinh deu tren quy dao thi
%   V(x,t)=x'*P(t)*x
% la chung chi Lyapunov cho on dinh mu deu cua MO HINH SAI SO LTV.
%
% Gioi han ket luan:
% - Day la chung minh cuc bo quanh quy dao cho mo hinh tuyen tinh hoa.
% - Khong bao gom bao hoa co cau chap han, tre, ma sat khong mo hinh hoa
%   hoac tai ngoai neu tai do khong duoc dua vao phep tuyen tinh hoa.

requiredVariables = {'robot','q_full','qd_full','qdd_full','t_full', ...
    'K_pp','K_vp','K_vi','torque_constant'};
for iVar = 1:numel(requiredVariables)
    assert(exist(requiredVariables{iVar},'var') == 1, ...
        'Thieu bien %s. Hay chay setup_pid truoc.',requiredVariables{iVar});
end

assert(size(q_full,2)==3 && isequal(size(q_full),size(qd_full),size(qdd_full)), ...
    'q_full, qd_full, qdd_full phai co cung kich thuoc N-by-3.');
assert(numel(t_full)==size(q_full,1) && all(diff(t_full)>0), ...
    't_full khong hop le.');

Kpp = diag(K_pp(:));
Kvp = diag(K_vp(:));
Kvi = diag(K_vi(:));
Kt  = torque_constant*eye(3);
assert(all(diag(Kpp)>0) && all(diag(Kvp)>0) && all(diag(Kvi)>0) ...
    && torque_constant>0,'Cac he so PID va torque_constant phai duong.');

N = numel(t_full);
nx = 9;
delta = 1e-5;
Qlyap = eye(nx);

Acl_all = zeros(nx,nx,N);
P_all = nan(nx,nx,N);
maxRealPole = nan(N,1);
minEigP = nan(N,1);
maxEigP = nan(N,1);
normalizedResidual = nan(N,1);

fprintf('Dang tuyen tinh hoa he PID doc theo quy dao...\n');
for k = 1:N
    qk   = q_full(k,:)';
    dqk  = qd_full(k,:)';
    ddqk = qdd_full(k,:)';

    M = massMatrix(robot,qk);
    assert(all(isfinite(M(:))) && rcond(M)>1e-12, ...
        'M(q) suy bien hoac khong huu han tai mau %d.',k);

    % Dao ham cua tau_ID(q,dq,ddq_ref) theo q va dq.
    Kq = zeros(3);
    Kv = zeros(3);
    for j = 1:3
        h = zeros(3,1);
        h(j) = delta;
        Kq(:,j) = (inverseDynamics(robot,qk+h,dqk,ddqk) ...
                  -inverseDynamics(robot,qk-h,dqk,ddqk))/(2*delta);
        Kv(:,j) = (inverseDynamics(robot,qk,dqk+h,ddqk) ...
                  -inverseDynamics(robot,qk,dqk-h,ddqk))/(2*delta);
    end

    % tau_fb = -Kt*Kvp*Kpp*delta_q - Kt*Kvp*delta_dq + Kt*Kvi*xi
    A21 = -(M\(Kq + Kt*Kvp*Kpp));
    A22 = -(M\(Kv + Kt*Kvp));
    A23 =  (M\(Kt*Kvi));

    Acl = [zeros(3), eye(3),  zeros(3); ...
           A21,       A22,     A23; ...
          -Kpp,      -eye(3), zeros(3)];
    Acl_all(:,:,k) = Acl;

    poles = eig(Acl);
    maxRealPole(k) = max(real(poles));
    if maxRealPole(k) < 0
        % lyap(A',Q) giai A'*P + P*A = -Q.
        P = lyap(Acl',Qlyap);
        P = (P+P')/2;
        P_all(:,:,k) = P;
        eigP = eig(P);
        minEigP(k) = min(real(eigP));
        maxEigP(k) = max(real(eigP));
        R = Acl'*P + P*Acl + Qlyap;
        normalizedResidual(k) = norm(R,'fro')/(1 + 2*norm(Acl,'fro')*norm(P,'fro'));
    end
end

frozenTimePass = all(maxRealPole < -1e-9) && all(isfinite(P_all(:)));
if ~frozenTimePass
    firstBad = find(maxRealPole >= -1e-9 | ~isfinite(minEigP),1,'first');
    error(['Kiem tra frozen-time FAIL tai t=%.6g s: max Re(lambda)=%.6e. ' ...
        'Khong the tao chung chi P(t) tren toan quy dao.'], ...
        t_full(firstBad),maxRealPole(firstBad));
end

% Dao ham so cua P(t). gradient xu ly ca buoc thoi gian khong deu.
Pdot_all = zeros(nx,nx,N);
for i = 1:nx
    for j = 1:nx
        Pdot_all(i,j,:) = reshape(gradient(squeeze(P_all(i,j,:)),t_full),1,1,N);
    end
end

minEigW = nan(N,1);
decayRateBound = nan(N,1);
for k = 1:N
    Pdot = (Pdot_all(:,:,k)+Pdot_all(:,:,k)')/2;
    W = Qlyap-Pdot;
    W = (W+W')/2;
    minEigW(k) = min(real(eig(W)));
    decayRateBound(k) = minEigW(k)/maxEigP(k);
end

tolP = 1e-10;
tolW = 1e-10;
pPositivePass = all(minEigP > tolP);
ltvPass = pPositivePass && all(minEigW > tolW);

fprintf('\n========== KET QUA ON DINH PID ==========\n');
fprintf('min lambda(P)               = %.6e\n',min(minEigP));
fprintf('min lambda(W=I-Pdot)        = %.6e\n',min(minEigW));
fprintf('max Re(lambda(Acl))         = %.6e\n',max(maxRealPole));
fprintf('max normalized Lyap error   = %.6e\n',max(normalizedResidual));
fprintf('min decay-rate bound        = %.6e 1/s\n',min(decayRateBound));
fprintf('P(t) positive definite      : %s\n',passFail(pPositivePass));
fprintf('Frozen-time stable          : %s\n',passFail(frozenTimePass));
fprintf('LTV Lyapunov certificate    : %s\n',passFail(ltvPass));

if ltvPass
    fprintf(['KET LUAN: V(x,t)=x''P(t)x chung minh mo hinh sai so LTV cua PID ' ...
        'on dinh mu deu cuc bo doc theo quy dao.\n']);
else
    fprintf(['KET LUAN: Chua co chung chi LTV tren toan quy dao vi W=I-Pdot ' ...
        'khong duong xac dinh tai moi thoi diem.\n']);
end
fprintf(['LUU Y: Ket qua tren khong tu dong chung minh on dinh toan cuc cua he ' ...
    'phi tuyen va khong bao gom bao hoa/tai ngoai chua mo hinh hoa.\n']);

figure('Name','PID Lyapunov stability check','Color','w');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t_full,minEigP,'LineWidth',1.25); grid on;
yline(0,'k--'); xlabel('Time (s)'); ylabel('min eig(P)');
title('P(t) positive definite');

nexttile;
plot(t_full,minEigW,'LineWidth',1.25); grid on;
yline(0,'k--'); xlabel('Time (s)'); ylabel('min eig(W)');
title('LTV Lyapunov condition: W=I-Pdot');

nexttile;
plot(t_full,maxRealPole,'LineWidth',1.25); grid on;
yline(0,'k--'); xlabel('Time (s)'); ylabel('max real pole');
title('Frozen-time closed-loop poles');

nexttile;
semilogy(t_full,max(normalizedResidual,realmin),'LineWidth',1.0); grid on;
xlabel('Time (s)'); ylabel('normalized residual');
title('Continuous Lyapunov equation residual');

function text = passFail(condition)
if condition
    text = 'PASS';
else
    text = 'FAIL';
end
end
