%% 1. TÍNH TOÁN QUỸ ĐẠO MONG MUỐN (DESIRED TRAJECTORY) BẰNG ĐỘNG HỌC THUẬN
N = length(t_full);
xyz_desired = zeros(N, 3);

% Ma trận dịch chuyển từ chốt khuỷu tay ra điểm mũi gắp (End-Effector)
% Thông số lấy từ thước đo SolidWorks của bạn
T_offset = trvec2tform([0.33226, 0.03092, 0.13012]);  

disp('Đang tính Động học thuận để lấy quỹ đạo mong muốn...');
for i = 1:N
    % Lấy ma trận biến đổi của lower_arm_link (khuỷu tay) tại thời điểm i
    % Lệnh này dùng luôn biến robot và q_full từ file setup_lqr của bạn
    T_elbow = getTransform(robot, q_full(i,:)', 'lower_arm_link');
    
    % Tịnh tiến gốc tọa độ ra điểm mũi gắp
    T_EE = T_elbow * T_offset;
    
    % Trích xuất giá trị X, Y, Z lưu vào mảng
    xyz_desired(i, :) = T_EE(1:3, 4)';
end
disp('Đã tính xong!');

%% 2. VẼ ĐỒ THỊ 3D SO SÁNH
% Ép dẹp dữ liệu về 2D (phòng trường hợp Simulink lưu dạng 3x1xN)
raw_data = squeeze(out.xyz_actual.Data);

% Nếu ma trận đang nằm ngang (3 hàng, N cột) thì lật ngược lại thành (N hàng, 3 cột)
if size(raw_data, 1) == 3
    raw_data = raw_data';
end

% Trích xuất X, Y, Z an toàn
x_act = raw_data(:, 1);
y_act = raw_data(:, 2);
z_act = raw_data(:, 3);

% Khởi tạo cửa sổ figure
figure('Name', 'So sánh Quỹ đạo End-Effector 3D', 'Color', 'w');
hold on; grid on;

% Vẽ quỹ đạo Thực tế (Màu Xanh dương, nét liền)
plot3(x_act, y_act, z_act, 'b-', 'LineWidth', 2, 'DisplayName', 'Thực tế (Actual)');

% Vẽ quỹ đạo Mong muốn (Màu Đỏ, nét đứt)
plot3(xyz_desired(:,1), xyz_desired(:,2), xyz_desired(:,3), 'r--', 'LineWidth', 2, 'DisplayName', 'Mong muốn (Desired)');

% Trang trí đồ thị
xlabel('Trục X (m)', 'FontWeight', 'bold'); 
ylabel('Trục Y (m)', 'FontWeight', 'bold'); 
zlabel('Trục Z (m)', 'FontWeight', 'bold');
title('Quỹ đạo điểm mút tay máy (End-Effector) trong không gian', 'FontSize', 14);
legend('Location', 'best');
view(3); % Bật góc nhìn 3D