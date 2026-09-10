% =========================================================================
% SETUP ADAPTIVE MPC CHO TAY MÁY 3-DOF
% =========================================================================
clear; close all; clc;

%% 1. Khởi tạo robot
robot = importrobot('urdf.urdf');
robot.DataFormat = 'column';

%% 2. Nạp và xử lý quỹ đạo
load('trajectory.mat');   % Yêu cầu: q, qd, qdd có kích thước N-by-3

% Đảo chiều khớp shoulder và elbow theo quy ước mô hình
q(:, 2:3)   = -q(:, 2:3);
qd(:, 2:3)  = -qd(:, 2:3);
qdd(:, 2:3) = -qdd(:, 2:3);

%% 3. Chèn pha approach 5 giây
Ts    = 0.01;
T_app = 5.0;

t_app = (0:Ts:T_app-Ts)';
s     = t_app / T_app;

% Quintic trajectory: đầu/cuối có vận tốc và gia tốc bằng 0
h   = 10*s.^3 - 15*s.^4 + 6*s.^5;
hd  = (30*s.^2 - 60*s.^3 + 30*s.^4) / T_app;
hdd = (60*s - 180*s.^2 + 120*s.^3) / T_app^2;

% Phải khớp với cấu hình ban đầu của plant Simulink
q0 = [0, 0, 0];
q1 = q(1, :);

q_app   = q0 + h   .* (q1 - q0);
qd_app  =      hd  .* (q1 - q0);
qdd_app =      hdd .* (q1 - q0);

q_full   = [q_app;   q];
qd_full  = [qd_app;  qd];
qdd_full = [qdd_app; qdd];

t_full = (0:size(q_full,1)-1)' * Ts;

ts_q   = timeseries(q_full,   t_full);
ts_qd  = timeseries(qd_full,  t_full);
ts_qdd = timeseries(qdd_full, t_full);

disp('Đã nạp quỹ đạo và chèn pha approach 5 giây.');

%% 4. MPC nominal model
% x = [ integral(e); e; e_dot ], kích thước 9
% u = tau_FB, kích thước 3
% y = x, vì C = I9

sys_nominal = ss(zeros(9,9), zeros(9,3), eye(9), zeros(9,3), Ts);
mpcobj = mpc(sys_nominal, Ts);

%% 5. Horizons
mpcobj.PredictionHorizon = 30;   % Np = 0.30 s
mpcobj.ControlHorizon    = 5;    % Nc = 0.05 s

%% 6. Trọng số MPC
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

%% 7. Ràng buộc mô-men phản hồi tau_FB
% Lưu ý: tau thuc = tau_FF + tau_FB
max_tau_fb = [5, 30, 5];         % N.m

%% 8. Ràng buộc tốc độ thay đổi tau_FB
% Đơn vị: N.m / sample, với Ts = 0.01 s
max_dTau_fb = [0.5, 2.0, 0.5];

for i = 1:3
    mpcobj.MV(i).Min = -max_tau_fb(i);
    mpcobj.MV(i).Max =  max_tau_fb(i);

    mpcobj.MV(i).RateMin = -max_dTau_fb(i);
    mpcobj.MV(i).RateMax =  max_dTau_fb(i);
end

%% 9. Ràng buộc trạng thái lỗi
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

disp('Đã cấu hình Adaptive MPC: Q, mô-men, rate và state constraints.');