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
%
% sys_nominal = ss(zeros(9,9), zeros(9,3), eye(9), zeros(9,3), Ts);
% mpcobj = mpc(sys_nominal, Ts);

%% 3. MÔ HÌNH KHỞI TẠO CHO ADAPTIVE MPC
% Trạng thái:
% x = [integral(e); e; e_dot]    kích thước 9x1
%
% Đầu vào:
% u = tau_FB                     kích thước 3x1
%
% Đầu ra:
% y = x                          kích thước 9x1

n = 3;

% Điểm làm việc ban đầu lấy từ mẫu đầu tiên của quỹ đạo
q0   = q_full(1, :).';
qd0  = qd_full(1, :).';
qdd0 = qdd_full(1, :).';

% Ma trận khối lượng tại điểm đầu quỹ đạo
M0 = massMatrix(robot, q0);

% Tuyến tính hóa inverse dynamics bằng sai phân trung tâm
delta = 1e-5;

Kq0  = zeros(n, n);
Kqd0 = zeros(n, n);

for i = 1:n
    %% Đạo hàm theo vị trí khớp q
    q_plus  = q0;
    q_minus = q0;

    q_plus(i)  = q_plus(i)  + delta;
    q_minus(i) = q_minus(i) - delta;

    tau_plus = inverseDynamics(robot, ...
                               q_plus, qd0, qdd0);

    tau_minus = inverseDynamics(robot, ...
                                q_minus, qd0, qdd0);

    Kq0(:, i) = (tau_plus - tau_minus)/(2*delta);

    %% Đạo hàm theo vận tốc khớp q_dot
    qd_plus  = qd0;
    qd_minus = qd0;

    qd_plus(i)  = qd_plus(i)  + delta;
    qd_minus(i) = qd_minus(i) - delta;

    tau_plus = inverseDynamics(robot, ...
                               q0, qd_plus, qdd0);

    tau_minus = inverseDynamics(robot, ...
                                q0, qd_minus, qdd0);

    Kqd0(:, i) = (tau_plus - tau_minus)/(2*delta);
end

%% Mô hình sai số liên tục
I3 = eye(3);
O3 = zeros(3);

% Không dùng inv(M0) để tránh sai số số học
Minv_Kq0  = M0 \ Kq0;
Minv_Kqd0 = M0 \ Kqd0;
Minv_I3   = M0 \ I3;

Ac0 = [O3, I3,          O3;
       O3, O3,          I3;
       O3, -Minv_Kq0,  -Minv_Kqd0];

Bc0 = [O3;
       O3;
       Minv_I3];

Cc0 = eye(9);
Dc0 = zeros(9,3);

%% Rời rạc hóa mô hình với Zero-Order Hold
sys_continuous = ss(Ac0, Bc0, Cc0, Dc0);
sys_nominal = c2d(sys_continuous, Ts, 'zoh');

%% Tạo MPC Controller
mpcobj = mpc(sys_nominal, Ts);

disp('Đã tạo mô hình MPC khởi tạo từ điểm đầu quỹ đạo.');

%% 4. Horizons
mpcobj.PredictionHorizon = 30;   % Np = 0.30 s
mpcobj.ControlHorizon    = 5;    % Nc = 0.05 s

%% 5. Trọng số MPC ban đầu ở mức vừa phải
% Thứ tự output:
% [int(e1) int(e2) int(e3) e1 e2 e3 ed1 ed2 ed3]
Qz  = [0, 0, 0];                 % Trọng số sai số tích phân
Qe  = [10, 35, 10];              % Trọng số sai số vị trí
Qed = [0, 0, 0];                 % Trọng số sai số vận tốc

% PHẢI là vector 1x9, không dùng diag(...)
mpcobj.Weights.OutputVariables = [Qz, Qe, Qed];

% mv chính là tau_FB
mpcobj.Weights.ManipulatedVariables = [1, 1, 1]; % Trọng số momen

% Đây là trọng số làm mượt, không phải ràng buộc cứng
mpcobj.Weights.ManipulatedVariablesRate = [1, 1, 1];

%% 6. Ràng buộc duy nhất: mô-men phản hồi tau_FB
% Lưu ý: tau thuc = tau_FF + tau_FB
max_tau_fb = [100, 100, 100];         % N.m

for i = 1:3
    mpcobj.MV(i).Min = -max_tau_fb(i);
    mpcobj.MV(i).Max =  max_tau_fb(i);
end

%% 7. Tải ngoài tại tool_tip
% Đổi lần lượt 0, 1, 2, 5 [kg] để khảo sát giống mô phỏng LQR và PID.
payloadMass = 0.5;

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
