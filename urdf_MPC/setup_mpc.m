% =========================================================================
% SETUP ADAPTIVE MPC CHAY CHINH - THAM SO DA DUOC VALIDATE
% =========================================================================
% Thu tu chay:
%   setup_mpc_runtime
%   out = sim('urdf');
%   plot_mpc_trajectory

clear; close all; clc;
setupDir = fileparts(mfilename('fullpath'));

%% 1. Robot va frame tool_tip
urdfFile = fullfile(setupDir,'urdf.urdf');
assert(isfile(urdfFile),'Khong tim thay file URDF: %s',urdfFile);
robot = importrobot(urdfFile);
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81];

toolTipName = 'tool_tip';
p_elbow_tool = [0.34349 0.04674 0.00607];
if ~any(strcmp(robot.BodyNames,toolTipName))
    toolTip = rigidBody(toolTipName);
    toolTipJoint = rigidBodyJoint('tool_tip_fixed','fixed');
    setFixedTransform(toolTipJoint,trvec2tform(p_elbow_tool));
    toolTip.Mass = 0;
    toolTip.CenterOfMass = [0 0 0];
    toolTip.Inertia = zeros(1,6);
    toolTip.Joint = toolTipJoint;
    addBody(robot,toolTip,robot.BodyNames{end});
end

%% 2. Quy dao
trajectoryFile = fullfile(setupDir,'trajectory_new.mat');
assert(isfile(trajectoryFile),'Khong tim thay: %s',trajectoryFile);
trajectoryData = load(trajectoryFile);
for requiredName = {'t','q','qd','qdd'}
    assert(isfield(trajectoryData,requiredName{1}), ...
        'trajectory_new.mat thieu bien %s.',requiredName{1});
end

t_full = trajectoryData.t(:);
q = trajectoryData.q;
qd = trajectoryData.qd;
qdd = trajectoryData.qdd;
N = numel(t_full);
assert(isequal(size(q),[N 3],size(qd),[N 3],size(qdd),[N 3]), ...
    'q, qd, qdd phai co kich thuoc N-by-3.');
assert(all(diff(t_full)>0),'Vector thoi gian phai tang nghiem ngat.');

% Quy uoc dau cua mo hinh Simscape.
q(:,2:3) = -q(:,2:3);
qd(:,2:3) = -qd(:,2:3);
qdd(:,2:3) = -qdd(:,2:3);
q_full = q;
qd_full = qd;
qdd_full = qdd;
Ts = round(median(diff(t_full)),12);
assert(max(abs(diff(t_full)-Ts))<1e-8*max(1,Ts), ...
    'Quy dao phai co thoi gian lay mau deu.');

% Chuan hoa luoi thoi gian de tranh sai lech so cham dong.
t_full = (0:N-1)'*Ts;
ts_q = timeseries(q_full,t_full);
ts_qd = timeseries(qd_full,t_full);
ts_qdd = timeseries(qdd_full,t_full);

%% 3. Mo hinh danh dinh MPC vat ly, khac 0
% x = [integral(e); e; e_dot], u = tau_FB, y = x.
linearizationDelta = 1e-5;
[A0,B0,C0,D0] = localErrorModel(robot,q_full(1,:)',qd_full(1,:)', ...
    qdd_full(1,:)',Ts,linearizationDelta);
assert(norm(A0,'fro')>0 && norm(B0,'fro')>0, ...
    'Mo hinh danh dinh A0/B0 khong hop le.');
sys_nominal = ss(A0,B0,C0,D0,Ts);

%% 4. Bo tham so duoc autotune va validation
% Bộ tham số MPC sau khi autotune full
Np = 30;
Nc = 10;

mpcobj = mpc(sys_nominal, Ts, Np, Nc);

Qz  = [1.3067 1.3067 1.3067];
Qe  = [1.1822 2.9554 1.1822];
Qed = [0.0200 0.0200 0.0200];

R   = [0.0695 0.0695 0.0695];
Rdu = [0.0479 0.0479 0.0479];

mpcobj.Weights.OutputVariables = [Qz Qe Qed];
mpcobj.Weights.ManipulatedVariables = R;
mpcobj.Weights.ManipulatedVariablesRate = Rdu;
% ScaleFactor la mot phan cua cau hinh da duoc tune; khong bo qua.
outputScales = [0.25 0.25 0.25 0.05 0.05 0.05 0.10 0.10 0.10];
tauMax = [5 40 5];                         % gioi han tong moment, N.m
fbSlewMax = [50 200 50];                  % toc do tau_FB, N.m/s
for i = 1:9
    mpcobj.OV(i).ScaleFactor = outputScales(i);
end

%% 5. Gioi han tau_FB de tau_FF + tau_FB nam trong gioi han tong
tau_ff = zeros(N,3);
for k = 1:N
    tau_ff(k,:) = inverseDynamics(robot,q_full(k,:)',qd_full(k,:)', ...
        qdd_full(k,:)')';
end
fbMin = -tauMax-min(tau_ff,[],1);
fbMax =  tauMax-max(tau_ff,[],1);
assert(all(fbMin<fbMax) && all(fbMin<0) && all(fbMax>0), ...
    ['Feedforward vuot gioi han moment hoac khong con mien dieu khien. ' ...
     'Peak abs tau_FF = %s N.m.'],mat2str(max(abs(tau_ff),[],1),5));

for i = 1:3
    mpcobj.MV(i).ScaleFactor = tauMax(i);
    mpcobj.MV(i).Min = fbMin(i);
    mpcobj.MV(i).Max = fbMax(i);
    mpcobj.MV(i).MinECR = 0;
    mpcobj.MV(i).MaxECR = 0;
    mpcobj.MV(i).RateMin = -fbSlewMax(i)*Ts;
    mpcobj.MV(i).RateMax =  fbSlewMax(i)*Ts;
    mpcobj.MV(i).RateMinECR = 0;
    mpcobj.MV(i).RateMaxECR = 0;
end

mpcobj.Model.Nominal.X = zeros(9,1);
mpcobj.Model.Nominal.Y = zeros(9,1);
mpcobj.Model.Nominal.U = zeros(3,1);
mpcobj.Model.Nominal.DX = zeros(9,1);

%% 6. Tai ngoai tai tool_tip
payloadMass = 1;
compensatePayload = false;
assert(isscalar(payloadMass) && isfinite(payloadMass) && payloadMass>=0, ...
    'payloadMass phai la so huu han va khong am.');

Fg_world = payloadMass*robot.Gravity;
ts_Fload = timeseries(repmat(Fg_world,N,1),t_full);

% Giu bien ts_Fext de tuong thich voi cau hinh Inverse Dynamics cu.
eeBody = toolTipName;
Fext_first = externalForce(robot,eeBody,zeros(1,6),q_full(1,:)');
Fext_array = zeros([size(Fext_first),N]);
if compensatePayload && payloadMass>0
    Fg_base = payloadMass*robot.Gravity(:);
    for k = 1:N
        q_k = q_full(k,:)';
        T_BE = getTransform(robot,q_k,eeBody);
        Fg_body = T_BE(1:3,1:3)'*Fg_base;
        wrench = [0 0 0 Fg_body'];
        Fext_array(:,:,k) = externalForce(robot,eeBody,wrench,q_k);
    end
end
ts_Fext = timeseries(Fext_array,t_full);

%% 7. Dong bo cau hinh cua urdf.slx voi lan validation
modelFile = fullfile(setupDir,'urdf.slx');
assert(isfile(modelFile),'Khong tim thay: %s',modelFile);
load_system(modelFile);
modelName = 'urdf';
set_param(modelName,'StartTime','0','StopTime',num2str(t_full(end),17), ...
    'SolverType','Variable-step','Solver','ode15s', ...
    'MaxStep',num2str(Ts/2,17),'RelTol','1e-5','AbsTol','1e-7', ...
    'ReturnWorkspaceOutputs','on');

integrator = find_system(modelName,'SearchDepth',1, ...
    'BlockType','DiscreteIntegrator');
assert(numel(integrator)==1,'Khong tim thay duy nhat mot bo tich phan sai so.');
set_param(integrator{1},'SampleTime','Ts', ...
    'IntegratorMethod','Integration: Forward Euler', ...
    'InitialCondition','[0 0 0]');

sources = find_system(modelName,'SearchDepth',1,'BlockType','FromWorkspace');
for k = 1:numel(sources)
    set_param(sources{k},'SampleTime','Ts','Interpolate','off', ...
        'OutputAfterFinalValue','Holding final value');
end

set_param([modelName '/Plant_new/MechanismConfiguration'], ...
    'GravityVector',mat2str(robot.Gravity));
set_param([modelName '/Plant_new/Rigid Transform2'], ...
    'TranslationCartesianOffset',mat2str(p_elbow_tool));

fprintf('\nAdaptive MPC runtime ready.\n');
fprintf('Qz=%s\nQe=%s\nQed=%s\nR=%s\nRdu=%s\n', ...
    mat2str(Qz),mat2str(Qe),mat2str(Qed),mat2str(R),mat2str(Rdu));
fprintf('Np=%d, Nc=%d, payload=%.1f kg\n',Np,Nc,payloadMass);
fprintf('tau total limit=%s N.m\n',mat2str(tauMax));
fprintf('tau_FB bounds: min=%s, max=%s N.m\n', ...
    mat2str(fbMin,5),mat2str(fbMax,5));
fprintf('Next: out = sim(''urdf''); then plot_mpc_trajectory\n');
open_system(modelName);

function [A,B,C,D] = localErrorModel(robot,q,dq,ddq,Ts,delta)
% Linear hoa mo hinh sai so quanh mau quy dao dau tien.
q=q(:); dq=dq(:); ddq=ddq(:);
M=massMatrix(robot,q);
assert(all(isfinite(M(:))) && rcond(M)>1e-12, ...
    'Ma tran khoi luong khong hop le.');
Kq=zeros(3); Kv=zeros(3);
for j=1:3
    h=zeros(3,1); h(j)=delta;
    Kq(:,j)=(inverseDynamics(robot,q+h,dq,ddq) ...
        - inverseDynamics(robot,q-h,dq,ddq))/(2*delta);
    Kv(:,j)=(inverseDynamics(robot,q,dq+h,ddq) ...
        - inverseDynamics(robot,q,dq-h,ddq))/(2*delta);
end
Ac=[zeros(3) eye(3); -(M\Kq) -(M\Kv)];
Bc=[zeros(3); M\eye(3)];
E=expm([Ac Bc; zeros(3,9)]*Ts);
A=[eye(3) Ts*eye(3) zeros(3); zeros(6,3) E(1:6,1:6)];
B=[zeros(3); E(1:6,7:9)];
C=eye(9);
D=zeros(9,3);
end
