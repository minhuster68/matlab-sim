% =========================================================================
% SETUP CASCADED PID CHO TAY MAY 3-DOF
% Dong co: GIM6010-8
% =========================================================================
clear; close all; clc;

% Thu muc chua setup_pid.m. Co the bam Run du Current Folder cua MATLAB
% dang o vi tri khac.
setupDir = fileparts(mfilename('fullpath'));

% =========================================================================
% 0.1. KHOI TAO ROBOT TRUC TIEP TU URDF
% =========================================================================
urdfFile = fullfile(setupDir, 'urdf.urdf');
if ~isfile(urdfFile)
    error('Khong tim thay file URDF: %s', urdfFile);
end

robot = importrobot(urdfFile);
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81]; % m/s^2, truc Z cua he base huong len

% =========================================================================
% 0.2. THEM FRAME TOOL_TIP VAO ROBOT IMPORT TU URDF
% Vector duoc do tu Origin_elbow_joint den Origin_tool_tip trong SolidWorks.
% Hai he truc song song nen transform chi co tinh tien.
% =========================================================================
toolTipName = 'tool_tip';

if ~any(strcmp(robot.BodyNames, toolTipName))
    parentBody = robot.BodyNames{end};

    toolTip = rigidBody(toolTipName);
    toolTipJoint = rigidBodyJoint('tool_tip_fixed', 'fixed');

    p_elbow_tool = [0.34349, 0.04674, 0.00607]; % m
    setFixedTransform(toolTipJoint, trvec2tform(p_elbow_tool));

    % tool_tip chi la frame dat luc, khong phai vat co khoi luong.
    toolTip.Mass = 0;
    toolTip.CenterOfMass = [0 0 0];
    toolTip.Inertia = zeros(1,6);
    toolTip.Joint = toolTipJoint;

    addBody(robot, toolTip, parentBody);
    fprintf('Da them tool_tip vao body khau 3: %s\n', parentBody);
else
    disp('tool_tip da ton tai trong robot, khong them lai.');
end

% =========================================================================
% 0.3. NAP QUY DAO MOI
% trajectory_new.mat da gom pha di tu q = [0 0 0], pha quet va pha quay ve,
% vi vay khong chen them approach trong setup PID.
% =========================================================================
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

if size(q,2) ~= 3 || ~isequal(size(qd), size(q), size(qdd), size(q))
    error('q, qd, qdd phai co cung kich thuoc N-by-3.');
end

if numel(t_full) ~= size(q,1) || any(diff(t_full) <= 0)
    error('t phai tang nghiem ngat va co cung so mau voi q, qd, qdd.');
end

% Quy uoc dau giong setup_lqr.m va setup_mpc.m.
q(:, 2:3)   = -q(:, 2:3);
qd(:, 2:3)  = -qd(:, 2:3);
qdd(:, 2:3) = -qdd(:, 2:3);

q_full   = q;
qd_full  = qd;
qdd_full = qdd;
Ts = median(diff(t_full));

ts_q   = timeseries(q_full,   t_full);
ts_qd  = timeseries(qd_full,  t_full);
ts_qdd = timeseries(qdd_full, t_full);

fprintf(['Da import URDF va nap trajectory_new.mat: %.2f s, ', ...
         'Ts = %.4f s.\n'], t_full(end), Ts);

% =========================================================================
% 1. THONG SO CASCADED PID
% Thu tu vector: [khop Base; khop Shoulder; khop Elbow]
% =========================================================================

% Vong ngoai - dieu khien vi tri.
K_pp = [46;
        60;
        36];

% Vong trong - dieu khien van toc.
K_vp = [8.0;
        9.5;
        10.0];

K_vi = [7.6;
        7.6;
        7.6];

% Vong dong dien/mo-men.
torque_constant = 0.47; % Nm/A, GIM6010-8

% =========================================================================
% 2. TAI NGOAI TAI TOOL_TIP
% =========================================================================

payloadMass = 5;

% false: tai chi tac dung vat ly vao plant, dung de danh gia PID.
% true : Inverse Dynamics biet tai va tao them mo-men bu feedforward.
compensatePayload = false;

eeBody = toolTipName;
N = numel(t_full);

if ~isscalar(payloadMass) || ~isfinite(payloadMass) || payloadMass < 0
    error('payloadMass phai la mot so huu han va khong am.');
end

% Luc trong truong dua vao plant Simscape, bieu dien trong World frame.
Fg_world = payloadMass * robot.Gravity;
Fload_array = repmat(Fg_world, N, 1);
ts_Fload = timeseries(Fload_array, t_full);

% Ngoai luc cap cho khoi Inverse Dynamics. Khi khong bu tai, mang bang 0.
q_first = q_full(1, :);
if strcmpi(robot.DataFormat, 'column')
    q_first = q_first.';
end

Fext_first = externalForce(robot, eeBody, zeros(1,6), q_first);
Fext_array = zeros([size(Fext_first), N]);

if compensatePayload && payloadMass > 0
    Fg_base = payloadMass * robot.Gravity(:);

    for k = 1:N
        q_k = q_full(k, :);
        if strcmpi(robot.DataFormat, 'column')
            q_k = q_k.';
        end

        T_BE = getTransform(robot, q_k, eeBody);
        R_BE = T_BE(1:3, 1:3);
        Fg_body = R_BE.' * Fg_base;

        % Wrench = [Tx Ty Tz Fx Fy Fz], luc dat tai goc tool_tip.
        wrench = [0 0 0 Fg_body.'];
        Fext_array(:, :, k) = externalForce(robot, eeBody, wrench, q_k);
    end
end

ts_Fext = timeseries(Fext_array, t_full);

fprintf(['Da tao tai %.1f kg tai tool_tip: ', ...
         'ts_Fload = luc vat ly, bu feedforward = %s.\n'], ...
        payloadMass, mat2str(compensatePayload));

disp('Da nap xong Cascaded PID. Tro lai Simulink va nhan Run.');
