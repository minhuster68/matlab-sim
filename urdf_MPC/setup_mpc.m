% =========================================================================
% SETUP ADAPTIVE MPC CHO TAY MÁY 3-DOF
% =========================================================================
clear; close all; clc;

% Thư mục chứa setup_mpc.m, cho phép bấm Run từ bất kỳ Current Folder nào.
setupDir = fileparts(mfilename('fullpath'));

%% 1. Khởi tạo robot trực tiếp từ URDF
urdfFile = fullfile(setupDir, 'urdf.urdf');
if ~isfile(urdfFile)
    error('Không tìm thấy file URDF: %s', urdfFile);
end

robot = importrobot(urdfFile);
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81]; % m/s^2, trục Z của hệ base hướng lên

% Thêm frame tool_tip tại vị trí đã đo trong SolidWorks.
toolTipName = 'tool_tip';

if ~any(strcmp(robot.BodyNames, toolTipName))
    parentBody = robot.BodyNames{end};

    toolTip = rigidBody(toolTipName);
    toolTipJoint = rigidBodyJoint('tool_tip_fixed', 'fixed');

    p_elbow_tool = [0.34349, 0.04674, 0.00607]; % m
    setFixedTransform(toolTipJoint, trvec2tform(p_elbow_tool));

    toolTip.Mass = 0;
    toolTip.CenterOfMass = [0 0 0];
    toolTip.Inertia = zeros(1,6);
    toolTip.Joint = toolTipJoint;

    addBody(robot, toolTip, parentBody);
    fprintf('Đã thêm tool_tip vào body khâu 3: %s\n', parentBody);
else
    disp('tool_tip đã tồn tại trong robot, không thêm lại.');
end

%% 2. Nạp quỹ đạo mới
% trajectory_new.mat đã gồm pha từ q = [0 0 0], pha quét và pha quay về;
% không chèn thêm approach trong setup MPC.
trajectoryFile = fullfile(setupDir, 'trajectory_new.mat');
if ~isfile(trajectoryFile)
    error('Không tìm thấy file quỹ đạo: %s', trajectoryFile);
end

trajectoryData = load(trajectoryFile);
requiredVariables = {'t', 'q', 'qd', 'qdd'};
for iVar = 1:numel(requiredVariables)
    variableName = requiredVariables{iVar};
    if ~isfield(trajectoryData, variableName)
        error('trajectory_new.mat thiếu biến %s.', variableName);
    end
end

q   = trajectoryData.q;
qd  = trajectoryData.qd;
qdd = trajectoryData.qdd;
t_full = trajectoryData.t(:);

if size(q,2) ~= 3 || ~isequal(size(qd), size(q), size(qdd), size(q))
    error('q, qd, qdd phải có cùng kích thước N-by-3.');
end

if numel(t_full) ~= size(q,1) || any(diff(t_full) <= 0)
    error('t phải tăng nghiêm ngặt và có cùng số mẫu với q, qd, qdd.');
end

% Đảo chiều khớp shoulder và elbow theo quy ước mô hình
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

fprintf(['Đã import URDF và nạp trajectory_new.mat: %.2f s, ', ...
         'Ts = %.4f s.\n'], t_full(end), Ts);

%% 3. MPC nominal model
% x = [ integral(e); e; e_dot ], kích thước 9
% u = tau_FB, kích thước 3
% y = x, vì C = I9

sys_nominal = ss(zeros(9,9), zeros(9,3), eye(9), zeros(9,3), Ts);
mpcobj = mpc(sys_nominal, Ts);

%% 4. Horizons
mpcobj.PredictionHorizon = 30;   % Np = 0.30 s
mpcobj.ControlHorizon    = 5;    % Nc = 0.05 s

%% 5. Trọng số MPC
% Thứ tự output:
% [int(e1) int(e2) int(e3) e1 e2 e3 ed1 ed2 ed3]

Qz  = [40000, 20000, 40000];    % Trọng số sai số tích phân
Qe  = [15000, 20000, 30000];    % Trọng số sai số vị trí
Qed = [50, 50, 50];              % Trọng số sai số vận tốc

% PHẢI là vector 1x9, không dùng diag(...)
mpcobj.Weights.OutputVariables = [Qz, Qe, Qed];

% mv chính là tau_FB
mpcobj.Weights.ManipulatedVariables = [10, 10, 5];

% Phạt biến thiên mô-men, giúp lệnh mượt hơn
mpcobj.Weights.ManipulatedVariablesRate = [1, 1, 0.5];

%% 6. Ràng buộc mô-men phản hồi tau_FB
% Lưu ý: tau thuc = tau_FF + tau_FB
max_tau_fb = [5, 30, 5];         % N.m

%% 7. Ràng buộc tốc độ thay đổi tau_FB
% Đơn vị: N.m / sample, với Ts = 0.01 s
max_dTau_fb = [0.5, 2.0, 0.5];

for i = 1:3
    mpcobj.MV(i).Min = -max_tau_fb(i);
    mpcobj.MV(i).Max =  max_tau_fb(i);

    mpcobj.MV(i).RateMin = -max_dTau_fb(i);
    mpcobj.MV(i).RateMax =  max_dTau_fb(i);
end

%% 8. Ràng buộc trạng thái lỗi
% e = q - qd
% e_dot = q_dot - qd_dot
%
% Không ràng buộc z = integral(e), vì dễ làm bài toán khó khả thi
% khi mô-men bị bão hòa hoặc có nhiễu.

max_e  = deg2rad([5, 5, 5]);     % sai số góc tối đa: 5 độ
max_de = [1.0, 1.0, 1.0];        % sai số vận tốc tối đa: rad/s

for i = 1:3
    % Output 4:6 la e1, e2, e3
    mpcobj.OV(i+3).Min = -max_e(i);
    mpcobj.OV(i+3).Max =  max_e(i);

    % Output 7:9 la ed1, ed2, ed3
    mpcobj.OV(i+6).Min = -max_de(i);
    mpcobj.OV(i+6).Max =  max_de(i);
end

%% 9. Tải ngoài tại tool_tip
% Đổi lần lượt 0, 1, 2, 5 [kg] để khảo sát giống mô phỏng LQR và PID.
payloadMass = 0;

% false: tải chỉ tác dụng vật lý vào plant, dùng để đánh giá MPC.
% true : Inverse Dynamics biết tải và tạo thêm mô-men bù feedforward.
compensatePayload = false;

eeBody = toolTipName;
N = numel(t_full);

if ~isscalar(payloadMass) || ~isfinite(payloadMass) || payloadMass < 0
    error('payloadMass phải là một số hữu hạn và không âm.');
end

% Lực trọng trường đưa vào plant Simscape trong World frame.
Fg_world = payloadMass * robot.Gravity;
Fload_array = repmat(Fg_world, N, 1);
ts_Fload = timeseries(Fload_array, t_full);

% Ngoại lực cho Inverse Dynamics. Khi không bù tải, mảng luôn bằng 0.
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

        % Wrench = [Tx Ty Tz Fx Fy Fz], lực đặt tại gốc tool_tip.
        wrench = [0 0 0 Fg_body.'];
        Fext_array(:, :, k) = externalForce(robot, eeBody, wrench, q_k);
    end
end

ts_Fext = timeseries(Fext_array, t_full);

fprintf(['Đã tạo tải %.1f kg tại tool_tip: ', ...
         'ts_Fload = lực vật lý, bù feedforward = %s.\n'], ...
        payloadMass, mat2str(compensatePayload));

disp('Đã cấu hình Adaptive MPC: Q, mô-men, rate và state constraints.');
