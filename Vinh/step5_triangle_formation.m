%% STEP 5 - 3 TURTLEBOT3 TRIANGLE FORMATION
clear;
clc;
close all;

%% =========================================================
% 1. THOI GIAN MO PHONG
% ==========================================================

dt = 0.05;
T  = 30;

t = 0:dt:T;
N = length(t);

%% =========================================================
% 2. LEADER R1
% ==========================================================

x1     = zeros(1,N);
y1     = zeros(1,N);
theta1 = zeros(1,N);

v1     = zeros(1,N);
omega1 = zeros(1,N);

x1(1) = 0;
y1(1) = 0;
theta1(1) = 0;

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

%% =========================================================
% 3. DINH NGHIA DOI HINH TAM GIAC
% ==========================================================

d = 0.8;      % Canh tam giac [m]

% R2: sau - trai
r2x = -sqrt(3)/2*d;
r2y =  1/2*d;

% R3: sau - phai
r3x = -sqrt(3)/2*d;
r3y = -1/2*d;

%% =========================================================
% 4. QUY DAO MONG MUON R2 VA R3
% ==========================================================

x2d = zeros(1,N);
y2d = zeros(1,N);

x3d = zeros(1,N);
y3d = zeros(1,N);

for k = 1:N

    % ----- R2 -----
    x2d(k) = x1(k) ...
        + cos(theta1(k))*r2x ...
        - sin(theta1(k))*r2y;

    y2d(k) = y1(k) ...
        + sin(theta1(k))*r2x ...
        + cos(theta1(k))*r2y;

    % ----- R3 -----
    x3d(k) = x1(k) ...
        + cos(theta1(k))*r3x ...
        - sin(theta1(k))*r3y;

    y3d(k) = y1(k) ...
        + sin(theta1(k))*r3x ...
        + cos(theta1(k))*r3y;

end

%% =========================================================
% 5. VAN TOC QUY DAO MONG MUON R2
% ==========================================================

dx2d = gradient(x2d,dt);
dy2d = gradient(y2d,dt);

v2d = sqrt(dx2d.^2 + dy2d.^2);

theta2d = atan2(dy2d,dx2d);
theta2d = unwrap(theta2d);

omega2d = gradient(theta2d,dt);

%% =========================================================
% 6. VAN TOC QUY DAO MONG MUON R3
% ==========================================================

dx3d = gradient(x3d,dt);
dy3d = gradient(y3d,dt);

v3d = sqrt(dx3d.^2 + dy3d.^2);

theta3d = atan2(dy3d,dx3d);
theta3d = unwrap(theta3d);

omega3d = gradient(theta3d,dt);

%% =========================================================
% 7. TRANG THAI THUC TE R2
% ==========================================================

x2     = zeros(1,N);
y2     = zeros(1,N);
theta2 = zeros(1,N);

% Co tinh dat sai vi tri
x2(1) = -0.2;
y2(1) = 0.3;
theta2(1) = 0;

v2     = zeros(1,N);
omega2 = zeros(1,N);

%% =========================================================
% 8. TRANG THAI THUC TE R3
% ==========================================================

x3     = zeros(1,N);
y3     = zeros(1,N);
theta3 = zeros(1,N);

% Co tinh dat sai vi tri
x3(1) = -0.2;
y3(1) = -0.3;
theta3(1) = 0;

v3     = zeros(1,N);
omega3 = zeros(1,N);

%% =========================================================
% 9. THAM SO CONTROLLER
% ==========================================================

kx = 0.8;
ky = 1.5;
ktheta = 1.5;

% Gioi han TurtleBot3 Burger
vMax     = 0.22;
omegaMax = 2.84;

%% =========================================================
% 10. CONTROLLER R2 + R3
% ==========================================================

for k = 1:N-1

    %% ================= R2 =================

    % Sai so global
    px2 = x2d(k) - x2(k);
    py2 = y2d(k) - y2(k);

    % Sai so goc
    etheta2 = theta2d(k) - theta2(k);

    etheta2 = atan2( ...
        sin(etheta2), ...
        cos(etheta2));

    % Sai so trong he toa do R2
    ex2 = ...
        cos(theta2(k))*px2 ...
        + sin(theta2(k))*py2;

    ey2 = ...
        -sin(theta2(k))*px2 ...
        + cos(theta2(k))*py2;

    % Controller
    v2(k) = ...
        v2d(k)*cos(etheta2) ...
        + kx*ex2;

    omega2(k) = ...
        omega2d(k) ...
        + ky*ey2 ...
        + ktheta*sin(etheta2);

    % Saturation
    v2(k) = ...
        max(min(v2(k),vMax),-vMax);

    omega2(k) = ...
        max(min(omega2(k),omegaMax),-omegaMax);

    % Dong hoc R2
    x2(k+1) = x2(k) ...
        + v2(k)*cos(theta2(k))*dt;

    y2(k+1) = y2(k) ...
        + v2(k)*sin(theta2(k))*dt;

    theta2(k+1) = theta2(k) ...
        + omega2(k)*dt;


    %% ================= R3 =================

    % Sai so global
    px3 = x3d(k) - x3(k);
    py3 = y3d(k) - y3(k);

    % Sai so goc
    etheta3 = theta3d(k) - theta3(k);

    etheta3 = atan2( ...
        sin(etheta3), ...
        cos(etheta3));

    % Sai so trong he toa do R3
    ex3 = ...
        cos(theta3(k))*px3 ...
        + sin(theta3(k))*py3;

    ey3 = ...
        -sin(theta3(k))*px3 ...
        + cos(theta3(k))*py3;

    % Controller
    v3(k) = ...
        v3d(k)*cos(etheta3) ...
        + kx*ex3;

    omega3(k) = ...
        omega3d(k) ...
        + ky*ey3 ...
        + ktheta*sin(etheta3);

    % Saturation
    v3(k) = ...
        max(min(v3(k),vMax),-vMax);

    omega3(k) = ...
        max(min(omega3(k),omegaMax),-omegaMax);

    % Dong hoc R3
    x3(k+1) = x3(k) ...
        + v3(k)*cos(theta3(k))*dt;

    y3(k+1) = y3(k) ...
        + v3(k)*sin(theta3(k))*dt;

    theta3(k+1) = theta3(k) ...
        + omega3(k)*dt;

end

v2(end) = v2(end-1);
omega2(end) = omega2(end-1);

v3(end) = v3(end-1);
omega3(end) = omega3(end-1);

%% =========================================================
% 11. TRACKING ERROR
% ==========================================================

errorR2 = sqrt( ...
    (x2d-x2).^2 + ...
    (y2d-y2).^2 );

errorR3 = sqrt( ...
    (x3d-x3).^2 + ...
    (y3d-y3).^2 );

%% =========================================================
% 12. KHOANG CACH GIUA CAC ROBOT
% ==========================================================

d12 = sqrt( ...
    (x1-x2).^2 + ...
    (y1-y2).^2 );

d13 = sqrt( ...
    (x1-x3).^2 + ...
    (y1-y3).^2 );

d23 = sqrt( ...
    (x2-x3).^2 + ...
    (y2-y3).^2 );

%% =========================================================
% 13. FIGURE 1 - TRAJECTORY
% ==========================================================

figure;

plot(x1,y1,...
    'LineWidth',2,...
    'DisplayName','Leader R1');

hold on;

plot(x2d,y2d,'--',...
    'LineWidth',1.5,...
    'DisplayName','Desired R2');

plot(x3d,y3d,'--',...
    'LineWidth',1.5,...
    'DisplayName','Desired R3');

plot(x2,y2,...
    'LineWidth',2,...
    'DisplayName','Actual R2');

plot(x3,y3,...
    'LineWidth',2,...
    'DisplayName','Actual R3');

% Noi 3 robot tai thoi diem cuoi
plot( ...
    [x1(end) x2(end) x3(end) x1(end)], ...
    [y1(end) y2(end) y3(end) y1(end)], ...
    ':', ...
    'LineWidth',1.5,...
    'HandleVisibility','off');

grid on;
axis equal;

xlabel('x [m]');
ylabel('y [m]');

title('3 TurtleBot3 Leader-Follower Formation');

legend('Location','best');

%% =========================================================
% 14. FIGURE 2 - TRACKING ERRORS
% ==========================================================

figure;

plot(t,errorR2,...
    'LineWidth',2,...
    'DisplayName','R2 error');

hold on;

plot(t,errorR3,...
    'LineWidth',2,...
    'DisplayName','R3 error');

grid on;

xlabel('Time [s]');
ylabel('Tracking error [m]');

title('Follower Tracking Errors');

legend('Location','best');

%% =========================================================
% 15. FIGURE 3 - FORMATION DISTANCES
% ==========================================================

figure;

plot(t,d12,...
    'LineWidth',2,...
    'DisplayName','d_{12}');

hold on;

plot(t,d13,...
    'LineWidth',2,...
    'DisplayName','d_{13}');

plot(t,d23,...
    'LineWidth',2,...
    'DisplayName','d_{23}');

yline(d,'--',...
    'Desired distance',...
    'LineWidth',1.5);

grid on;

xlabel('Time [s]');
ylabel('Distance [m]');

title('Distances Between Robots');

legend('Location','best');