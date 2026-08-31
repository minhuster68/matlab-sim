% =========================================================================
% SETUP LQR - GAIN SCHEDULING (TIME-VARYING LQR)
% Tay máy: 3-DOF
% =========================================================================
n = 3;
delta = 1e-5; % Bước vi phân

% =========================================================================
% 1. Chọn Q, R theo luật Bryson (TUNE RIÊNG TỪNG KHỚP)
% Thứ tự mảng: [Khớp Base, Khớp Shoulder, Khớp Elbow]
% =========================================================================

max_int_e = [100, 200.0, 100.0];   
max_e = [0.1, 0.1, 0.1];       
max_de = [2.0, 2.0, 2.0];
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
% Lưu ý: Cần chạy file tạo quỹ đạo trước để Workspace có q_full, qd_full, t_full
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
disp('Đã tạo xong bộ từ điển LQR (ts_K)! Sẵn sàng chạy Simulink.');