%% STEP 3 - TAO VI TRI MONG MUON CHO FOLLOWER R2
clear;
clc;
close all;

%% 1. Thoi gian mo phong
dt = 0.05;
T  = 30;

t = 0:dt:T;
N = length(t);

%% 2. Trang thai Leader R1
x1     = zeros(1, N);
y1     = zeros(1, N);
theta1 = zeros(1, N);

x1(1) = 0;
y1(1) = 0;
theta1(1) = 0;

%% 3. Toc do Leader
v1     = zeros(1, N);
omega1 = zeros(1, N);

%% 4. Mo phong Leader
for k = 1:N-1

    v1(k) = 0.15;

    if t(k) < 10

        omega1(k) = 0;

    elseif t(k) < 20

        omega1(k) = 0.15;

    else

        omega1(k) = 0;

    end

    % Dong hoc Leader
    x1(k+1) = x1(k) ...
        + v1(k)*cos(theta1(k))*dt;

    y1(k+1) = y1(k) ...
        + v1(k)*sin(theta1(k))*dt;

    theta1(k+1) = theta1(k) ...
        + omega1(k)*dt;

end

v1(end) = v1(end-1);
omega1(end) = omega1(end-1);

%% 5. Khoang cach formation
d = 0.8;                 % Canh tam giac [m]

% Vi tri R2 trong he toa do gan voi Leader
r2x = -sqrt(3)/2 * d;
r2y =  1/2 * d;

%% 6. Vi tri mong muon cua R2
x2d = zeros(1, N);
y2d = zeros(1, N);

for k = 1:N

    x2d(k) = x1(k) ...
        + cos(theta1(k))*r2x ...
        - sin(theta1(k))*r2y;

    y2d(k) = y1(k) ...
        + sin(theta1(k))*r2x ...
        + cos(theta1(k))*r2y;

end

%% 7. Ve Leader va quy dao mong muon cua R2
figure;

plot(x1, y1, ...
    'LineWidth', 2);

hold on;

plot(x2d, y2d, '--', ...
    'LineWidth', 2);

% Vi tri ban dau
plot(x1(1), y1(1), ...
    'o', 'MarkerSize', 8);

plot(x2d(1), y2d(1), ...
    'o', 'MarkerSize', 8);

% Vi tri cuoi
plot(x1(end), y1(end), ...
    's', 'MarkerSize', 8);

plot(x2d(end), y2d(end), ...
    's', 'MarkerSize', 8);

grid on;
axis equal;

xlabel('x [m]');
ylabel('y [m]');

title('Leader R1 and Desired Trajectory of Follower R2');

legend( ...
    'Leader R1', ...
    'Desired trajectory R2', ...
    'R1 start', ...
    'R2 desired start', ...
    'R1 end', ...
    'R2 desired end');

%% 8. Ve mot so duong noi R1 - R2
for k = 1:100:N

    plot( ...
        [x1(k) x2d(k)], ...
        [y1(k) y2d(k)], ...
        ':');

end