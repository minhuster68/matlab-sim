%% CHECK_MPC_STABILITY_CONDITIONS
% Kiem tra cac dieu kien can va chan doan on dinh cho Adaptive MPC hien tai.
%
% Chay:
%   setup_mpc
%   check_mpc_stability_conditions
%
% QUAN TRONG:
% Script nay KHONG goi ket qua la mot chung minh Lyapunov day du cua MPC.
% Cau hinh hien tai dung bai toan chan troi huu han, rang buoc cung va mo hinh
% thay doi theo quy dao, nhung setup_mpc.m chua khai bao terminal cost/terminal
% set. Vi vay cac dieu kien du kinh dien cho on dinh tiem can va de quy kha thi
% chua duoc thiet lap.
%
% Script kiem tra:
% 1) Tinh on dinh hoa duoc (stabilizability) cua moi mo hinh frozen-time.
% 2) Tinh quan sat duoc (C=I nen truc tiep quan sat toan bo trang thai).
% 3) Trong so stage cost co phat duong moi thanh phan trang thai hay khong.
% 4) Mien tau_FB co chua diem can bang u=0 va co do rong duong hay khong.
% 5) MathWorks MPC design review cho doi tuong mpcobj.

requiredVariables = {'robot','q_full','qd_full','qdd_full','t_full','Ts', ...
    'mpcobj','Qz','Qe','Qed','R','Rdu','outputScales','tauMax', ...
    'fbSlewMax','fbMin','fbMax','Np','Nc'};
for iVar = 1:numel(requiredVariables)
    assert(exist(requiredVariables{iVar},'var') == 1, ...
        'Thieu bien %s. Hay chay setup_mpc truoc.',requiredVariables{iVar});
end

assert(size(q_full,2)==3 && isequal(size(q_full),size(qd_full),size(qdd_full)), ...
    'q_full, qd_full, qdd_full phai co cung kich thuoc N-by-3.');
assert(numel(t_full)==size(q_full,1) && all(diff(t_full)>0) && Ts>0, ...
    't_full hoac Ts khong hop le.');

N = numel(t_full);
nx = 9;
delta = 1e-5;
spectralRadiusA = zeros(N,1);
minPbhSigma = inf(N,1);
numCriticalModes = zeros(N,1);
stabilizableAtSample = true(N,1);

fprintf('Dang kiem tra mo hinh Adaptive MPC doc theo quy dao...\n');
for k = 1:N
    [A,B] = localErrorModelForCheck(robot,q_full(k,:)',qd_full(k,:)', ...
        qdd_full(k,:)',Ts,delta);

    lambdaA = eig(A);
    spectralRadiusA(k) = max(abs(lambdaA));
    critical = find(abs(lambdaA) >= 1-1e-8);
    numCriticalModes(k) = numel(critical);

    for iMode = 1:numel(critical)
        lambda = lambdaA(critical(iMode));
        PBH = [lambda*eye(nx)-A, B];
        sigma = svd(PBH);
        sigmaMin = min(sigma);
        minPbhSigma(k) = min(minPbhSigma(k),sigmaMin);
        tolRank = 1e-9*max(1,norm(PBH,2));
        if rank(PBH,tolRank) < nx
            stabilizableAtSample(k) = false;
        end
    end

    if isempty(critical)
        minPbhSigma(k) = NaN;
    end
end

% C=I_9 trong setup_mpc.m, nen moi trang thai deu duoc do trong mo hinh MPC.
observablePass = true;
stabilizablePass = all(stabilizableAtSample);

stateWeights = [Qz(:);Qe(:);Qed(:)];
stageStateCostPass = all(isfinite(stateWeights)) && all(stateWeights>0) ...
    && numel(outputScales)==nx && all(outputScales(:)>0);
inputCostPass = all(isfinite(R(:))) && all(R(:)>=0) ...
    && all(isfinite(Rdu(:))) && all(Rdu(:)>0);

constraintWidth = fbMax(:)-fbMin(:);
zeroInputFeasible = all(fbMin(:)<=0) && all(fbMax(:)>=0);
constraintPass = all(isfinite(constraintWidth)) && all(constraintWidth>0) ...
    && zeroInputFeasible && all(fbSlewMax(:)>0) && all(tauMax(:)>0);

% setup_mpc.m hien khong co terminal cost, terminal controller hoac terminal set.
terminalIngredientsPresent = false;
formalCertificatePass = false;
numericalPreconditionsPass = stabilizablePass && observablePass ...
    && stageStateCostPass && inputCostPass && constraintPass;

fprintf('\n========== KET QUA KIEM TRA MPC ==========\n');
fprintf('prediction/control horizon       = %d / %d\n',Np,Nc);
fprintf('max spectral radius A(t)         = %.6e\n',max(spectralRadiusA));
fprintf('min PBH singular-value margin    = %.6e\n',min(minPbhSigma,[],'omitnan'));
fprintf('min tau_FB interval width [Nm]   = %.6e\n',min(constraintWidth));
fprintf('All frozen models stabilizable   : %s\n',passFail(stabilizablePass));
fprintf('Full-state model observable      : %s\n',passFail(observablePass));
fprintf('Positive state-stage weights     : %s\n',passFail(stageStateCostPass));
fprintf('Input/rate cost well posed       : %s\n',passFail(inputCostPass));
fprintf('Equilibrium inside constraints   : %s\n',passFail(constraintPass));
fprintf('Numerical preconditions          : %s\n',passFail(numericalPreconditionsPass));
fprintf('Terminal cost/set configured     : %s\n',passFail(terminalIngredientsPresent));
fprintf('Formal MPC Lyapunov certificate  : %s\n',passFail(formalCertificatePass));

if numericalPreconditionsPass
    fprintf(['KET LUAN DUNG: Mo hinh doc quy dao co cac dieu kien so co ban de MPC ' ...
        'co the dieu khien on dinh.\n']);
else
    fprintf(['KET LUAN: Co it nhat mot dieu kien so co ban khong dat; can sua truoc ' ...
        'khi danh gia on dinh.\n']);
end
fprintf(['CHUA DUOC KET LUAN: Chua the tu cac phep kiem tra tren suy ra on dinh ' ...
    'tiem can cua MPC co rang buoc. Cau hinh hien tai thieu terminal cost/set ' ...
    'va chua chung minh recursive feasibility.\n']);
fprintf(['VOI TAI NGOAI: Can kiem tra them QP status, bao hoa moment, sai so bam ' ...
    'va mien hap dan tu ket qua mo phong; stabilizability danh dinh khong thay ' ...
    'the cho cac kiem tra nay.\n']);

figure('Name','MPC stability-condition check','Color','w');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t_full,spectralRadiusA,'LineWidth',1.25); grid on;
yline(1,'k--'); xlabel('Time (s)'); ylabel('rho(A)');
title('Frozen-time open-loop spectral radius');

nexttile;
plot(t_full,minPbhSigma,'LineWidth',1.25); grid on;
yline(0,'k--'); xlabel('Time (s)'); ylabel('min PBH singular value');
title('Stabilizability margin of critical modes');

nexttile;
bar(1:3,[fbMin(:),fbMax(:)]); grid on;
yline(0,'k--'); xlabel('Joint'); ylabel('tau_{FB} (N.m)');
legend('minimum','maximum','Location','best');
title('Hard feedback-torque interval');

nexttile;
bar(1:9,stateWeights./outputScales(:)); grid on;
xlabel('MPC output/state channel'); ylabel('weight / scale');
title('Scaled state-stage penalties');

fprintf('\nDang mo MPC Design Review cua MathWorks...\n');
try
    review(mpcobj);
catch reviewError
    warning('Khong mo duoc MPC Design Review: %s',reviewError.message);
end

function [A,B] = localErrorModelForCheck(robot,q,dq,ddq,Ts,delta)
q=q(:); dq=dq(:); ddq=ddq(:);
M=massMatrix(robot,q);
assert(all(isfinite(M(:))) && rcond(M)>1e-12, ...
    'Ma tran khoi luong khong hop le.');
Kq=zeros(3); Kv=zeros(3);
for j=1:3
    h=zeros(3,1); h(j)=delta;
    Kq(:,j)=(inverseDynamics(robot,q+h,dq,ddq) ...
        -inverseDynamics(robot,q-h,dq,ddq))/(2*delta);
    Kv(:,j)=(inverseDynamics(robot,q,dq+h,ddq) ...
        -inverseDynamics(robot,q,dq-h,ddq))/(2*delta);
end
Ac=[zeros(3),eye(3);-(M\Kq),-(M\Kv)];
Bc=[zeros(3);M\eye(3)];
E=expm([Ac,Bc;zeros(3,9)]*Ts);
Ad=E(1:6,1:6);
Bd=E(1:6,7:9);
A=[eye(3),Ts*eye(3),zeros(3);zeros(6,3),Ad];
B=[zeros(3);Bd];
end

function text = passFail(condition)
if condition
    text = 'PASS';
else
    text = 'FAIL';
end
end
