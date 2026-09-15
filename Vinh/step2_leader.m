%% STEP 2 - LEADER TURTLEBOT3 BURGER
clear;
clc;
close all;

%% 1. Thoi gian mo phong
dt = 0.05;          % [s]
T  = 30;            % [s]

t = 0:dt:T;
N = length(t);

%% 2. Trang thai Leader R1
x1     = zeros(1, N);
y1     = zeros(1, N);
theta1 = zeros(1, N);

% Dieu kien ban dau
x1(1) = 0;
y1(1) = 0;
theta1(1) = 0;

%% 3. Toc do Leader
v1     = zeros(1, N);
omega1 = zeros(1, N);

%% 4. Mo phong Leader
for k = 1:N-1

    % ----- Tao chuyen dong cho Leader -----
    v1(k) = 0.15;       % toc do tien [m/s]

    if t(k) < 10

        % 0 -> 10 s: di thang
        omega1(k) = 0;

    elseif t(k) < 20

        % 10 -> 20 s: re trai
        omega1(k) = 0.15;

    else

        % 20 -> 30 s: di thang
        omega1(k) = 0;

    end

    % ----- Dong hoc TurtleBot3 -----
    x1(k+1) = x1(k) ...
        + v1(k)*cos(theta1(k))*dt;

    y1(k+1) = y1(k) ...
        + v1(k)*sin(theta1(k))*dt;

    theta1(k+1) = theta1(k) ...
        + omega1(k)*dt;

end

% Gan gia tri cuoi de ve do thi
v1(end) = v1(end-1);
omega1(end) = omega1(end-1);

%% 5. Ve quy dao Leader
figure;

plot(x1, y1, 'LineWidth', 2);
hold on;

plot(x1(1), y1(1), ...
    'o', 'MarkerSize', 8);

plot(x1(end), y1(end), ...
    's', 'MarkerSize', 8);

quiver( ...
    x1(end), ...
    y1(end), ...
    0.3*cos(theta1(end)), ...
    0.3*sin(theta1(end)), ...
    0, ...
    'LineWidth', 2);

grid on;
axis equal;

xlabel('x [m]');
ylabel('y [m]');

title('Leader R1 - TurtleBot3 Burger');

legend( ...
    'Leader trajectory', ...
    'Start', ...
    'End', ...
    'Heading');

%% 6. Ve toc do Leader
figure;

plot(t, v1, 'LineWidth', 2);

grid on;

xlabel('Time [s]');
ylabel('v_1 [m/s]');

title('Leader Linear Velocity');

%% 7. Ve toc do goc Leader
figure;

plot(t, omega1, 'LineWidth', 2);

grid on;

xlabel('Time [s]');
ylabel('\omega_1 [rad/s]');

title('Leader Angular Velocity');