% =========================================================================
% SETUP ADAPTIVE MPC CHO TAY MÁY 3-DOF KÈM PHA APPROACH 5 GIÂY
% =========================================================================
clear all; close all; clc;

% 1. Khởi tạo mô hình robot cho Simulink 
robot = importrobot('urdf.urdf'); 
robot.DataFormat = 'column';

% 2. Nạp file quỹ đạo 
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

% Tính toán đa thức bậc 5 (Quintic)
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
% 4. CẤU HÌNH BỘ ĐIỀU KHIỂN ADAPTIVE MPC
% =========================================================
% Khởi tạo đối tượng MPC nền
Ts = 0.01; 
sys_nominal = ss(zeros(9,9), zeros(9,3), eye(9), zeros(9,3));
sys_nominal.Ts = Ts;
mpcobj = mpc(sys_nominal, Ts);

% Cài đặt Horizons
mpcobj.PredictionHorizon = 20; 
mpcobj.ControlHorizon = 5;     

% Kế thừa trọng số Q, R từ luật Bryson 
max_int_e = [1.0, 2.0, 1.0];   
max_e = [0.05, 0.05, 0.05];       
max_de = [2.0, 2.0, 2.0];
max_tau = [5, 40, 5]; 

% Ma trận Q và R
mpcobj.Weights.OutputVariables = [1./max_int_e.^2, 1./max_e.^2, 1./max_de.^2]; 
mpcobj.Weights.ManipulatedVariables = 1./max_tau.^2; 
    
% Cài đặt ràng buộc vật lý động cơ
mpcobj.MV(1).Min = -max_tau(1); mpcobj.MV(1).Max = max_tau(1); 
mpcobj.MV(2).Min = -max_tau(2); mpcobj.MV(2).Max = max_tau(2); 
mpcobj.MV(3).Min = -max_tau(3); mpcobj.MV(3).Max = max_tau(3); 

disp('Đã cấu hình xong thông số MPC! Trở lại Simulink và nhấn Run.');