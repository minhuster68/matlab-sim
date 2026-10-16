% =========================================================================
% SETUP SMC CHO TAY MAY 3-DOF
%
% Thu tu cac khop trong tat ca vector:
%   [Base; Shoulder; Elbow]
%
% File nay thuc hien 4 viec:
%   1. Import robot tu urdf.urdf.
%   2. Them frame tool_tip tai dau khau cuoi.
%   3. Nap quy dao q, qd, qdd tu trajectory_new.mat.
%   4. Tao tham so SMC va tai ngoai ts_Fload cho Plant_new.
%
% Tai ngoai chi duoc dua vao plant qua ts_Fload. Bo dieu khien SMC
% khong biet truoc tau_L va khong bu tai bang ts_Fext.
% =========================================================================

clear;
close all;
clc;

% Thu muc chua setup_smc.m. Nho do co the bam Run ngay ca khi
% Current Folder cua MATLAB dang o vi tri khac.
setupDir = fileparts(mfilename('fullpath'));

%% ========================================================================
% 0.1. KHOI TAO ROBOT TU URDF
% =========================================================================

urdfFile = fullfile(setupDir, 'urdf.urdf');

if ~isfile(urdfFile)
    error('Khong tim thay file URDF: %s', urdfFile);
end

robot = importrobot(urdfFile);
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81];  % m/s^2, truc Z cua World/Base huong len

%% ========================================================================
% 0.2. THEM FRAME TOOL_TIP VAO RIGIDBODYTREE
% =========================================================================
% Vector duoc do tu Origin_elbow_joint den Origin_tool_tip trong SolidWorks.
% Hai he truc song song, vi vay phep bien doi chi gom tinh tien.
%
% Chu y:
%   - tool_tip chi la frame dat luc.
%   - tool_tip co khoi luong bang 0.
%   - Tai trong duoc tao rieng bang ts_Fload trong Plant_new.

toolTipName = 'tool_tip';

if ~any(strcmp(robot.BodyNames, toolTipName))
    % Truoc khi them tool_tip, body cuoi cua robot la lower_arm_link.
    parentBody = robot.BodyNames{end};

    toolTip = rigidBody(toolTipName);
    toolTipJoint = rigidBodyJoint('tool_tip_fixed', 'fixed');

    % [dx dy dz] tu Origin_elbow_joint den Origin_tool_tip, don vi m.
    p_elbow_tool = [0.34349, 0.04674, 0.00607];
    setFixedTransform(toolTipJoint, trvec2tform(p_elbow_tool));

    toolTip.Mass = 0;
    toolTip.CenterOfMass = [0 0 0];
    toolTip.Inertia = zeros(1, 6);
    toolTip.Joint = toolTipJoint;

    addBody(robot, toolTip, parentBody);
    fprintf('Da them tool_tip vao body cuoi: %s\n', parentBody);
else
    disp('tool_tip da ton tai trong robot, khong them lai.');
end

%% ========================================================================
% 0.3. NAP QUY DAO THAM CHIEU
% =========================================================================
% trajectory_new.mat phai chua:
%   t   : N-by-1, thoi gian [s]
%   q   : N-by-3, vi tri mong muon [rad]
%   qd  : N-by-3, van toc mong muon [rad/s]
%   qdd : N-by-3, gia toc mong muon [rad/s^2]

trajectoryFile = fullfile(setupDir, 'trajectory_new.mat');

if ~isfile(trajectoryFile)
    error('Khong tim thay file quy dao: %s', trajectoryFile);
end

trajectoryData = load(trajectoryFile);
requiredVariables = {'t', 'q', 'qd', 'qdd'};

for iVar = 1:numel(requiredVariables)
    variableName = requiredVariables{iVar};
    if ~isfield(trajectoryData, variableName)
        error('trajectory_new.mat thieu bien %s.', variableName);
    end
end

q   = trajectoryData.q;
qd  = trajectoryData.qd;
qdd = trajectoryData.qdd;
t_full = trajectoryData.t(:);

if ~isnumeric(q) || ~isnumeric(qd) || ~isnumeric(qdd) || ~isnumeric(t_full)
    error('t, q, qd va qdd phai la du lieu so.');
end

if size(q, 2) ~= 3 || ~isequal(size(qd), size(q), size(qdd), size(q))
    error('q, qd va qdd phai co cung kich thuoc N-by-3.');
end

if numel(t_full) ~= size(q, 1)
    error('t phai co cung so mau voi q, qd va qdd.');
end

if any(~isfinite(t_full)) || any(~isfinite(q(:))) || ...
        any(~isfinite(qd(:))) || any(~isfinite(qdd(:)))
    error('Quy dao chua gia tri NaN hoac Inf.');
end

if any(diff(t_full) <= 0)
    error('Vector thoi gian t phai tang nghiem ngat.');
end

% Quy uoc dau cua robot trong Simscape.
% Giu giong cac bo PID, LQR va MPC trong repo.
q(:, 2:3)   = -q(:, 2:3);
qd(:, 2:3)  = -qd(:, 2:3);
qdd(:, 2:3) = -qdd(:, 2:3);

q_full   = q;
qd_full  = qd;
qdd_full = qdd;

N = size(q_full, 1);
Ts = median(diff(t_full));

% Cac timeseries dua vao ba block From Workspace trong Simulink.
ts_q   = timeseries(q_full,   t_full);
ts_qd  = timeseries(qd_full,  t_full);
ts_qdd = timeseries(qdd_full, t_full);

fprintf(['Da nap trajectory_new.mat: %.2f s, %d mau, ', ...
         'Ts = %.4f s.\n'], t_full(end), N, Ts);

%% ========================================================================
% 1. THAM SO BO DIEU KHIEN SMC
% =========================================================================
% Mat truot:
%   s = de + lambdaSMC .* e
%
% Luat tiep can:
%   ds = -ksSMC .* s - krSMC .* sat(s ./ phiSMC)
%
% Gia toc ao:
%   v = ddqRef ...
%       - lambdaSMC .* de ...
%       - ksSMC .* s ...
%       - krSMC .* sat(s ./ phiSMC)
%
% Mo-men dieu khien duoc tinh boi block Inverse Dynamics:
%   tauSMC = inverseDynamics(robot, qActual, dqActual, v)

% He so quyet dinh toc do suy giam sai so tren mat truot.
lambda = [15; 85; 155];          % [1/s]

% He so keo bien s ve lop bien quanh mat truot.
ks = [5; 137; 180];              % [1/s]

% He so ben vung/chong nhieu.
% Day la gia tri khoi tao de chay mo phong lan dau; se tune theo tai sau.
kr = [30; 130; 90];              % [rad/s^2]

% Do day lop bien cua ham saturation.
% Tang phi: giam chattering nhung tang sai so du.
% Giam phi: bam chinh xac hon nhung mo-men de rung hon.
phi = [0.03; 0.05; 0.15];    % [rad/s]

if numel(lambda) ~= 3 || numel(ks) ~= 3 || ...
        numel(kr) ~= 3 || numel(phi) ~= 3
    error('Moi vector tham so SMC phai co dung 3 phan tu.');
end

if any(lambda <= 0)
    error('Tat ca phan tu lambdaSMC phai lon hon 0.');
end

if any(ks <= 0)
    error('Tat ca phan tu ksSMC phai lon hon 0.');
end

if any(kr < 0)
    error('Tat ca phan tu krSMC phai lon hon hoac bang 0.');
end

if any(phi <= 0)
    error('Tat ca phan tu phiSMC phai lon hon 0.');
end

%% ========================================================================
% 2. TAI NGOAI DAT TAI TOOL_TIP
% =========================================================================
% Doi payloadMass lan luot thanh 0, 1, 2 va 5 kg de khao sat.
%
% Tai nay chi tac dung vat ly vao Plant_new. No KHONG duoc dua vao
% block Inverse Dynamics, boi vi SMC dang duoc danh gia kha nang chong nhieu.

payloadMass = 1;  % [kg]

if ~isscalar(payloadMass) || ~isfinite(payloadMass) || payloadMass < 0
    error('payloadMass phai la mot so huu han va khong am.');
end

if norm(robot.Gravity) < eps && payloadMass ~= 0
    error('robot.Gravity dang bang 0, khong the tao tai trong truong.');
end

% External Force and Torque trong Plant_new dung World frame.
% Vi robot.Gravity = [0 0 -9.81], luc tai la [0 0 -m*g] N.
Fg_world = payloadMass * robot.Gravity;       % 1-by-3 [N]
Fload_array = repmat(Fg_world, N, 1);         % N-by-3 [N]
ts_Fload = timeseries(Fload_array, t_full);


