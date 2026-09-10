%% 1. TÍNH QUỸ ĐẠO MONG MUỐN TẠI FRAME TOOL_TIP
% Chạy theo thứ tự:
%   setup_mpc
%   out = sim('urdf');
%   plot_mpc_trajectory

if ~exist('robot', 'var') || ~isa(robot, 'rigidBodyTree')
    error('Chưa có biến robot. Hãy chạy setup_mpc trước.');
end

if ~exist('q_full', 'var') || ~exist('t_full', 'var')
    error('Chưa có q_full hoặc t_full. Hãy chạy setup_mpc trước.');
end

if ~any(strcmp(robot.BodyNames, 'tool_tip'))
    error('Robot chưa có body tool_tip. Hãy dùng file setup_mpc mới.');
end

if ~exist('out', 'var') || ~isa(out, 'Simulink.SimulationOutput')
    error('Chưa có biến out. Hãy chạy mô hình Simulink trước.');
end

outVariables = who(out);
if ~any(strcmp(outVariables, 'xyz_actual'))
    error(['Chưa có out.xyz_actual. Hãy chạy Simulink và lưu ', ...
           'vị trí thực tế vào out.xyz_actual.']);
end

N = size(q_full, 1);
xyz_desired = zeros(N, 3);

disp('Đang tính quỹ đạo mong muốn tại tool_tip...');
for i = 1:N
    T_EE = getTransform(robot, q_full(i,:)', 'tool_tip');
    xyz_desired(i,:) = T_EE(1:3,4)';
end
disp('Đã tính xong quỹ đạo mong muốn.');

%% 2. VẼ ĐỒ THỊ 3D SO SÁNH
raw_data = squeeze(out.xyz_actual.Data);

if size(raw_data, 1) == 3
    raw_data = raw_data';
end

if size(raw_data, 2) ~= 3
    error('out.xyz_actual phải chứa dữ liệu vị trí dạng N-by-3.');
end

x_act = raw_data(:, 1);
y_act = raw_data(:, 2);
z_act = raw_data(:, 3);

figure('Name', 'So sánh quỹ đạo End-Effector 3D - Adaptive MPC', ...
       'Color', 'w');
hold on; grid on;

plot3(x_act, y_act, z_act, 'b-', 'LineWidth', 2, ...
      'DisplayName', 'Thực tế (MPC)');
plot3(xyz_desired(:,1), xyz_desired(:,2), xyz_desired(:,3), ...
      'r--', 'LineWidth', 2, 'DisplayName', 'Mong muốn (Desired)');

xlabel('Trục X (m)', 'FontWeight', 'bold');
ylabel('Trục Y (m)', 'FontWeight', 'bold');
zlabel('Trục Z (m)', 'FontWeight', 'bold');

if exist('payloadMass', 'var')
    plotTitle = sprintf('Adaptive MPC - quỹ đạo tool\_tip với tải %.1f kg', ...
                        payloadMass);
else
    plotTitle = 'Adaptive MPC - quỹ đạo tool\_tip trong không gian';
end

title(plotTitle, 'FontSize', 14);
legend('Location', 'best');
view(3);
axis equal;
