%% STEP 1 - MO PHONG 1 TURTLEBOT3 BURGER
clear;
clc;
close all;

%% 1. Thoi gian mo phong
dt = 0.05;          % Buoc thoi gian [s]
T  = 20;            % Tong thoi gian mo phong [s]

t = 0:dt:T;
N = length(t);

%% 2. Khoi tao trang thai robot
x     = zeros(1, N);
y     = zeros(1, N);
theta = zeros(1, N);

% Vi tri ban dau
x(1) = 0;
y(1) = 0;
theta(1) = 0;

%% 3. Lenh dieu khien
v = 0.15;           % Toc do tien [m/s]
omega = 0.2;          % Toc do quay [rad/s]

%% 4. Mo phong dong hoc robot
for k = 1:N-1

    x(k+1) = x(k) + v*cos(theta(k))*dt;

    y(k+1) = y(k) + v*sin(theta(k))*dt;

    theta(k+1) = theta(k) + omega*dt;

end

%% 5. Ve quy dao
figure;

plot(x, y, 'LineWidth', 2);
hold on;

% Vi tri bat dau
plot(x(1), y(1), 'o', 'MarkerSize', 8);

% Vi tri ket thuc
plot(x(end), y(end), 's', 'MarkerSize', 8);

% Huong robot tai vi tri cuoi
quiver( ...
    x(end), ...
    y(end), ...
    0.2*cos(theta(end)), ...
    0.2*sin(theta(end)), ...
    0, ...
    'LineWidth', 2);

grid on;
axis equal;

xlabel('x [m]');
ylabel('y [m]');

title('TurtleBot3 Burger - Step 1');

legend('Trajectory', 'Start', 'End', 'Heading');