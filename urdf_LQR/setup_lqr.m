% =========================================================================
% SETUP LQR - GAIN SCHEDULING (TIME-VARYING LQR)
% Tay máy: 3-DOF
% =========================================================================
clear; close all; clc;

% Thư mục chứa file setup_lqr.m. Nhờ đó có thể bấm Run ngay cả khi
% Current Folder của MATLAB đang ở vị trí khác.
setupDir = fileparts(mfilename('fullpath'));

% =========================================================================
% 0.1. KHỞI TẠO ROBOT TRỰC TIẾP TỪ URDF
% =========================================================================
urdfFile = fullfile(setupDir, 'urdf.urdf');
if ~isfile(urdfFile)
    error('Không tìm thấy file URDF: %s', urdfFile);
end

robot = importrobot(urdfFile);
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81]; % m/s^2, trục Z của hệ base hướng lên

% =========================================================================
% 0.2. THÊM FRAME TOOL_TIP VÀO ROBOT ĐÃ IMPORT TỪ URDF
% Vector dưới đây được đo từ Origin_elbow_joint đến Origin_tool_tip.
% Hai hệ trục đã được bố trí song song nên transform chỉ có tịnh tiến.
% =========================================================================
toolTipName = 'tool_tip';

if ~any(strcmp(robot.BodyNames, toolTipName))
    % Trước khi thêm tool_tip, body cuối của chuỗi URDF là khâu 3.
    parentBody = robot.BodyNames{end};

    toolTip = rigidBody(toolTipName);
    toolTipJoint = rigidBodyJoint('tool_tip_fixed', 'fixed');

    p_elbow_tool = [0.34349, 0.04674, 0.00607]; % m
    setFixedTransform(toolTipJoint, trvec2tform(p_elbow_tool));

    % tool_tip chỉ là frame đặt lực, không phải một vật có khối lượng.
    toolTip.Mass = 0;
    toolTip.CenterOfMass = [0 0 0];
    toolTip.Inertia = zeros(1,6);
    toolTip.Joint = toolTipJoint;

    addBody(robot, toolTip, parentBody);
    fprintf('Đã thêm tool_tip vào body khâu 3: %s\n', parentBody);
else
    disp('tool_tip đã tồn tại trong robot, không thêm lại.');
end

% =========================================================================
% 0.3. NẠP QUỸ ĐẠO MỚI
% trajectory_new.mat đã bao gồm pha chuyển từ q = [0 0 0], pha quét,
% và pha quay về; không chèn thêm approach ở đây.
% =========================================================================
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

% Quy ước dấu giống setup_mpc.m
q(:, 2:3)   = -q(:, 2:3);
qd(:, 2:3)  = -qd(:, 2:3);
qdd(:, 2:3) = -qdd(:, 2:3);

% Quỹ đạo mới đã chứa toàn bộ chuyển động, dùng trực tiếp.
q_full   = q;
qd_full  = qd;
qdd_full = qdd;
Ts = median(diff(t_full));

ts_q   = timeseries(q_full,   t_full);
ts_qd  = timeseries(qd_full,  t_full);
ts_qdd = timeseries(qdd_full, t_full);

fprintf(['Đã import URDF và nạp trajectory_new.mat: %.2f s, ', ...
         'Ts = %.4f s.\n'], t_full(end), Ts);

n = 3;
delta = 1e-5; % Bước vi phân

% =========================================================================
% 1. Chọn Q, R theo luật Bryson (TUNE RIÊNG TỪNG KHỚP)
% Thứ tự mảng: [Khớp Base, Khớp Shoulder, Khớp Elbow]
% =========================================================================

max_int_e = [10, 20.0, 10.0];   
max_e = [0.05, 0.02, 0.01];       
max_de = [10.0, 10.0, 10.0];
% Giới hạn mô-men cho từng khớp (R matrix)
max_tau_base = 5;      
max_tau_shoulder = 40; 
max_tau_elbow = 5;     
%Tạo ma trận Q (9x9) và R (3x3) bằng toán tử mảng (.^)
Q = diag([1./max_int_e.^2, 1./max_e.^2, 1./max_de.^2]);
R = diag([1/max_tau_base^2, 1/max_tau_shoulder^2, 1/max_tau_elbow^2]);

% Q1 = 50;
% Q2 = 50;
% Q3 = 50;
% Q4 = 5000;
% Q5 = 5000;
% Q6 = 15000;
% Q7 = 50;
% Q8 = 50;
% Q9 = 50;

% R1 = 500;
% R2 = 500;
% R3 = 500;

% Q = diag([Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9]);
% R = diag([R1, R2, R3]);

% =========================================================================
% 2. QUÉT QUỸ ĐẠO VÀ TÍNH MA TRẬN K LIÊN TỤC
% q_full, qd_full, qdd_full và t_full đã được tạo ở mục 0.3 phía trên.
% =========================================================================
disp('Đang tính toán Time-Varying LQR dọc theo quỹ đạo. Vui lòng chờ...');

N = length(t_full);
K_array = zeros(3, 9, N); % Mảng 3D chứa toàn bộ ma trận K

for k = 1:N
    % Lấy trạng thái tại điểm thứ k
    q_k = q_full(k, :)';
    qd_k = qd_full(k, :)';
    
    % Tính M tại điểm k
    M_k = massMatrix(robot, q_k);
    M_inv = inv(M_k);
    
    % Tính Astiff và Adamp tại điểm k bằng sai phân
    Astiff = zeros(n, n);
    Adamp = zeros(n, n);
    for i = 1:n
        % Astiff
        q_p = q_k; q_m = q_k;
        q_p(i) = q_p(i) + delta; q_m(i) = q_m(i) - delta;
        Astiff(:, i) = (gravityTorque(robot, q_p) - gravityTorque(robot, q_m)) / (2*delta);
        
        % Adamp
        qd_p = qd_k; qd_m = qd_k;
        qd_p(i) = qd_p(i) + delta; qd_m(i) = qd_m(i) - delta;
        Adamp(:, i) = (velocityProduct(robot, q_k, qd_p) - velocityProduct(robot, q_k, qd_m)) / (2*delta);
    end
    
    % Lắp ráp Không gian trạng thái
    A_k = [zeros(3,3), eye(3), zeros(3,3);
           zeros(3,3), zeros(3,3), eye(3);
           zeros(3,3), -M_inv * Astiff, -M_inv * Adamp];
           
    B_k = [zeros(3,3); 
           zeros(3,3); 
           M_inv];
    
    % Giải phương trình Riccati và lưu vào mảng
    K_array(:, :, k) = lqr(A_k, B_k, Q, R);
end

% 3. Đóng gói thành timeseries để Simulink tự động nội suy
ts_K = timeseries(K_array, t_full);

% =========================================================================
% 4. TẢI NGOÀI TẠI TOOL_TIP
% =========================================================================

% Khối lượng tải cần khảo sát. Đổi lần lượt: 0, 1, 2, 5 [kg].
payloadMass = 2;

% false: tải chỉ tác dụng vật lý vào plant; dùng để đánh giá khả năng chống
%        nhiễu/độ bền vững của LQR.
% true : Inverse Dynamics biết tải và tạo thêm mô-men bù feedforward.
compensatePayload = false;

eeBody = toolTipName;

if ~isscalar(payloadMass) || ~isfinite(payloadMass) || payloadMass < 0
    error('payloadMass phải là một số hữu hạn và không âm.');
end

if norm(robot.Gravity) < eps && payloadMass ~= 0
    error('robot.Gravity đang bằng 0. Hãy đặt gravity trước khi tạo tải.');
end

% -------------------------------------------------------------------------
% 4.1. LỰC VẬT LÝ ĐƯA VÀO PLANT_NEW
% -------------------------------------------------------------------------
% External Force and Torque được đặt:
%   Force Resolution Frame = World
% nên lực có thể dùng trực tiếp trong hệ World/Base.
% Dữ liệu timeseries có dạng N-by-3: [Fx Fy Fz], đơn vị N.
Fg_world = payloadMass * robot.Gravity;       % ví dụ [0 0 -9.81*m]
Fload_array = repmat(Fg_world, N, 1);
ts_Fload = timeseries(Fload_array, t_full);

% -------------------------------------------------------------------------
% 4.2. NGOẠI LỰC CHO KHỐI INVERSE DYNAMICS
% -------------------------------------------------------------------------
% externalForce trả về kích thước phụ thuộc robot.DataFormat:
%   row    -> nDOF-by-6
%   column -> 6-by-nDOF
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

        % Orientation của tool_tip so với hệ base.
        T_BE = getTransform(robot, q_k, eeBody);
        R_BE = T_BE(1:3, 1:3);

        % externalForce nhận wrench biểu diễn trong frame của eeBody.
        Fg_body = R_BE.' * Fg_base;

        % Wrench = [Tx Ty Tz Fx Fy Fz]. Lực đặt ngay tại gốc tool_tip
        % nên mô-men ngoại lực bằng 0.
        wrench = [0 0 0 Fg_body.'];
        Fext_array(:, :, k) = externalForce(robot, eeBody, wrench, q_k);
    end
end

% Cùng time base với q, qd, K. Khi compensatePayload = false, ts_Fext = 0.
ts_Fext = timeseries(Fext_array, t_full);

fprintf(['Đã tạo tải %.1f kg tại tool_tip: ', ...
         'ts_Fload = lực vật lý, bù feedforward = %s.\n'], ...
        payloadMass, mat2str(compensatePayload));

disp('Đã tạo xong bộ từ điển LQR (ts_K)! Sẵn sàng chạy Simulink.');
