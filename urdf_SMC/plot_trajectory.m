%% 1. TINH QUY DAO MONG MUON TAI FRAME TOOL_TIP
% Chay theo thu tu:
%   setup_pid
%   out = sim('urdf');
%   plot_trajectory

if ~exist('robot', 'var') || ~isa(robot, 'rigidBodyTree')
    error('Chua co bien robot. Hay chay setup_pid truoc.');
end

if ~exist('q_full', 'var') || ~exist('t_full', 'var')
    error('Chua co q_full hoac t_full. Hay chay setup_pid truoc.');
end

if ~any(strcmp(robot.BodyNames, 'tool_tip'))
    error('Robot chua co body tool_tip. Hay dung file setup_pid moi.');
end

if ~exist('out', 'var') || ~isa(out, 'Simulink.SimulationOutput')
    error('Chua co bien out. Hay chay mo hinh Simulink truoc.');
end

outVariables = who(out);
if ~any(strcmp(outVariables, 'xyz_actual'))
    error(['Chua co out.xyz_actual. Hay chay Simulink va luu ', ...
           'vi tri thuc te vao out.xyz_actual.']);
end

N = size(q_full, 1);
xyz_desired = zeros(N, 3);

disp('Dang tinh quy dao mong muon tai tool_tip...');
for i = 1:N
    T_EE = getTransform(robot, q_full(i,:)', 'tool_tip');
    xyz_desired(i,:) = T_EE(1:3,4)';
end
disp('Da tinh xong quy dao mong muon.');

%% 2. VE DO THI 3D SO SANH
raw_data = squeeze(out.xyz_actual.Data);

if size(raw_data, 1) == 3
    raw_data = raw_data';
end

if size(raw_data, 2) ~= 3
    error('out.xyz_actual phai chua du lieu vi tri dang N-by-3.');
end

x_act = raw_data(:, 1);
y_act = raw_data(:, 2);
z_act = raw_data(:, 3);

figure('Name', 'So sanh quy dao End-Effector 3D - PID', 'Color', 'w');
hold on; grid on;

plot3(x_act, y_act, z_act, 'b-', 'LineWidth', 2, ...
      'DisplayName', 'Thuc te (Actual)');
plot3(xyz_desired(:,1), xyz_desired(:,2), xyz_desired(:,3), ...
      'r--', 'LineWidth', 2, 'DisplayName', 'Mong muon (Desired)');

xlabel('Truc X (m)', 'FontWeight', 'bold');
ylabel('Truc Y (m)', 'FontWeight', 'bold');
zlabel('Truc Z (m)', 'FontWeight', 'bold');

if exist('payloadMass', 'var')
    plotTitle = sprintf('PID - quy dao tool\_tip voi tai %.1f kg', payloadMass);
else
    plotTitle = 'PID - quy dao tool\_tip trong khong gian';
end

title(plotTitle, 'FontSize', 14);
legend('Location', 'best');
view(3);
axis equal;
