% =========================================================================
% SETUP SUPER-TWISTING CHO TAY MAY 3-DOF
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

% No clear/close: preserve other open models and comparison results.

% Thu muc chua setup_sta.m. Nho do co the bam Run ngay ca khi
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

%% SUPER-TWISTING: initial trial parameters, NOT validated/tuned gains.
% e = qActual - qRef; s = de + lambdaSTA .* e.
% v = ddqRef - lambdaSTA.*de - k1STA.*sqrt(abs(s)).*sign(s) + z
% dz = -k2STA.*sign(s); z[k+1] = z[k] + TsSTA*dz[k].
% z is stored in a discrete Unit Delay; output uses z[k].
% No -ks*s term: this is the classical STA structure.
% Same lambda as baseline to isolate the robust-law change initially.
lambdaSTA = [15; 85; 155];             % 1/s
k1STA = [15; 30; 25];                 % sqrt(rad)/s^(3/2)
k2STA = [30; 100; 80];                % rad/s^3
z0STA = zeros(3,1);                   % rad/s^2
TsSTA = 1e-3;                        % controller sample time [s]
maxStepSTA = TsSTA;                    % plant solver maximum step [s]
% TsSTA sets controller timing; maxStepSTA limits only the plant solver.
assert(isscalar(TsSTA) && isfinite(TsSTA) && TsSTA > 0);
for valueSTA = {lambdaSTA, k1STA, k2STA}
    assert(isequal(size(valueSTA{1}), [3 1]) && ...
        all(isfinite(valueSTA{1})) && all(valueSTA{1} > 0), ...
        'STA gains must be finite positive 3-by-1 vectors.');
end
assert(isequal(size(z0STA),[3 1]) && all(isfinite(z0STA)));
assert(isscalar(maxStepSTA) && isfinite(maxStepSTA) && maxStepSTA > 0);

%% ========================================================================
% 2. TAI NGOAI DAT TAI TOOL_TIP
% =========================================================================
% Doi payloadMass lan luot thanh 0, 1, 2 va 5 kg de khao sat.
%
% Tai nay chi tac dung vat ly vao Plant_new. No KHONG duoc dua vao
% block Inverse Dynamics, boi vi SMC dang duoc danh gia kha nang chong nhieu.

payloadMass = 0.5;  % [kg]

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


