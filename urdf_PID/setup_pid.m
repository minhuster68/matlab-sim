% =========================================================================
% SETUP CASCADED PID CHO TAY MÁY 3-DOF KÈM PHA APPROACH 5 GIÂY
% Động cơ: GIM6010-8
% =========================================================================
clear all; close all; clc;

% =========================================================
% 1. KHỞI TẠO MÔ HÌNH ROBOT CHO SIMULINK
% =========================================================
robot = importrobot('urdf.urdf'); 
robot.DataFormat = 'column';

% =========================================================
% 2. NẠP VÀ XỬ LÝ QUỸ ĐẠO
% =========================================================
load('trajectory.mat'); 

% --- XỬ LÝ ĐẢO CHIỀU KHỚP SHOULDER VÀ ELBOW ---
% Đảo dấu toàn bộ quỹ đạo của khớp 2 (cột 2) và khớp 3 (cột 3)
q(:, 2:3)   = -q(:, 2:3);
qd(:, 2:3)  = -qd(:, 2:3);
qdd(:, 2:3) = -qdd(:, 2:3);
% -----------------------------------------------

% =========================================================
% 3. CHÈN PHA APPROACH (5 GIÂY)
% =========================================================
% Thiết lập thời gian pha APPROACH
T_app = 5.0; 
dt = 0.01;
t_app = (0:dt:T_app-dt)';
s = t_app / T_app;

% Tính toán đa thức bậc 5 (Quintic) để robot di chuyển mượt
h = 10*s.^3 - 15*s.^4 + 6*s.^5;
hd = (30*s.^2 - 60*s.^3 + 30*s.^4) / T_app;
hdd = (60*s - 180*s.^2 + 120*s.^3) / T_app^2;

% Xác định điểm đầu (trạng thái tĩnh) và điểm nối (bắt đầu quỹ đạo gốc)
q0 = [0, 0, 0]; 
q1 = q(1, :);   

% Nội suy quỹ đạo 5 giây đầu
q_app = q0 + h .* (q1 - q0);
qd_app = hd .* (q1 - q0);
qdd_app = hdd .* (q1 - q0);

% Ghép nối quỹ đạo tiếp cận vào trước quỹ đạo gốc đã đảo chiều
q_full = [q_app; q];
qd_full = [qd_app; qd];
qdd_full = [qdd_app; qdd];

% Đóng gói thành định dạng timeseries cấp cho Simulink
t_full = (0:size(q_full,1)-1)' * dt;
ts_q = timeseries(q_full, t_full);
ts_qd = timeseries(qd_full, t_full);
ts_qdd = timeseries(qdd_full, t_full);

disp('Đã chèn thành công 5 giây Approach và đảo chiều quỹ đạo!');

% =========================================================
% 4. CẤU HÌNH THÔNG SỐ CASCADED PID (FIRMWARE GIM6010-8)
% =========================================================
% Khai báo dưới dạng Vector cột [3x1] tương ứng với 3 khớp: 
% [Khớp Đế (Base); Khớp Vai (Shoulder); Khớp Khuỷu (Elbow)]

% --- VÒNG NGOÀI (POSITION LOOP) ---
% K_pp: Hệ số Proportional cho vị trí
% Lưu ý: Gán bằng 0 trước để tune vòng vận tốc
K_pp = [46; 
        60; 
        36]; 

% --- VÒNG TRONG (VELOCITY LOOP) ---
% K_vp: Hệ số Proportional cho vận tốc
K_vp = [8.0; 
        9.5; 
        10]; 

% K_vi: Hệ số Integral cho vận tốc
K_vi = [7.6; 
        7.6; 
        7.6]; 

% =========================================================
% 5. THÔNG SỐ VÒNG DÒNG ĐIỆN / MÔ-MEN (TORQUE LOOP)
% =========================================================

torque_constant = 0.47; % Hằng số mô-men của GIM6010-8 (Nm/A)

disp('Đã nạp xong thông số Cascaded PID! Trở lại Simulink và nhấn Run.');