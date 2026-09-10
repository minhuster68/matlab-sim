%% 1. TÍNH TOÁN QUỸ ĐẠO MONG MUỐN (DESIRED TRAJECTORY) 
N = length(t);
xyz_desired = zeros(N, 3);

% Ma trận offset từ khớp khuỷu ra EE (Thông số SolidWorks)
T_offset = trvec2tform([0.33226, 0.03092, 0.13012]); 
disp('Đang tính Động học thuận lấy quỹ đạo mong muốn...');

for i = 1:N
    % Dùng đối tượng 'robot' và quỹ đạo 'q' có sẵn từ file setup_mpc
    T_elbow = getTransform(robot, q(i,:)', 'lower_arm_link');
    T_EE = T_elbow * T_offset;
    xyz_desired(i, :) = T_EE(1:3, 4)';
end
disp('Đã tính xong!');

%% 2. VẼ ĐỒ THỊ 3D SO SÁNH (THỰC TẾ MPC vs MONG MUỐN)
% Ép dẹp dữ liệu về 2D để tránh lỗi mảng 3D của Simulink
raw_data = squeeze(out.xyz_actual.Data);

% Nếu mảng nằm ngang, lật lại thành cột (N hàng x 3 cột)
if size(raw_data, 1) == 3
    raw_data = raw_data';
end

% Trích xuất tọa độ EE thực tế do MPC điều khiển
x_act = raw_data(:, 1);
y_act = raw_data(:, 2);
z_act = raw_data(:, 3);

% Khởi tạo giao diện đồ thị
figure('Name', 'So sánh Quỹ đạo EE 3D - Adaptive MPC', 'Color', 'w');
hold on; grid on;

% Vẽ quỹ đạo Thực tế (Màu Xanh dương, nét liền)
plot3(x_act, y_act, z_act, 'b-', 'LineWidth', 2, 'DisplayName', 'Thực tế (MPC)');

% Vẽ quỹ đạo Mong muốn (Màu Đỏ, nét đứt)
plot3(xyz_desired(:,1), xyz_desired(:,2), xyz_desired(:,3), 'r--', 'LineWidth', 2, 'DisplayName', 'Mong muốn (Desired)');

% Trang trí
xlabel('Trục X (m)', 'FontWeight', 'bold'); 
ylabel('Trục Y (m)', 'FontWeight', 'bold'); 
zlabel('Trục Z (m)', 'FontWeight', 'bold');
title('Hiệu năng bám quỹ đạo 3D: Adaptive MPC', 'FontSize', 14);
legend('Location', 'best');
view(3);