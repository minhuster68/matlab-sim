%% STEP 4 - FOLLOWER R2 TRACKING
clear;
clc;
close all;

%% 1. Thoi gian mo phong
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
% 3. TAO QUY DAO MONG MUON CHO R2
% ==========================================================

d = 0.8;

% R2 nam phia sau - ben trai Leader
r2x = -sqrt(3)/2*d;
r2y =  1/2*d;

x2d = zeros(1,N);
y2d = zeros(1,N);

for k = 1:N

    x2d(k) = x1(k) ...
        + cos(theta1(k))*r2x ...
        - sin(theta1(k))*r2y;

    y2d(k) = y1(k) ...
        + sin(theta1(k))*r2x ...
        + cos(theta1(k))*r2y;

end

%% =========================================================
% 4. TINH VAN TOC CUA QUY DAO MONG MUON R2
% ==========================================================

dx2d = gradient(x2d,dt);
dy2d = gradient(y2d,dt);

% Toc do tuyen tinh mong muon
v2d = sqrt(dx2d.^2 + dy2d.^2);

% Huong tiep tuyen cua quy dao mong muon
theta2d = atan2(dy2d,dx2d);

% unwrap tranh nhay tu pi sang -pi
theta2d = unwrap(theta2d);

% Toc do goc mong muon
omega2d = gradient(theta2d,dt);

%% =========================================================
% 5. TRANG THAI THUC TE CUA FOLLOWER R2
% ==========================================================

x2     = zeros(1,N);
y2     = zeros(1,N);
theta2 = zeros(1,N);

% Co tinh dat sai vi tri ban dau
x2(1) = -0.2;
y2(1) = -0.3;
theta2(1) = 0;

%% 6. Tin hieu dieu khien R2

v2     = zeros(1,N);
omega2 = zeros(1,N);

%% 7. Gain controller

kx = 0.8;
ky = 1.5;
ktheta = 1.5;

%% Gioi han TurtleBot3 Burger

vMax = 0.22;
omegaMax = 2.84;

%% =========================================================
% 8. CONTROLLER FOLLOWER R2
% ==========================================================

for k = 1:N-1

    % Sai so trong he toa do toan cuc
    px = x2d(k) - x2(k);
    py = y2d(k) - y2(k);

    % Sai so huong
    etheta = theta2d(k) - theta2(k);

    % Dua etheta ve [-pi, pi]
    etheta = atan2(sin(etheta),cos(etheta));

    % Chuyen sai so vi tri ve he toa do gan tren R2
    ex =  cos(theta2(k))*px ...
        + sin(theta2(k))*py;

    ey = -sin(theta2(k))*px ...
        + cos(theta2(k))*py;

    %% Luat dieu khien

    v2(k) = ...
        v2d(k)*cos(etheta) ...
        + kx*ex;

    omega2(k) = ...
        omega2d(k) ...
        + ky*ey ...
        + ktheta*sin(etheta);

    %% Saturation

    v2(k) = max(min(v2(k),vMax),-vMax);

    omega2(k) = ...
        max(min(omega2(k),omegaMax),-omegaMax);

    %% Dong hoc TurtleBot3 R2

    x2(k+1) = x2(k) ...
        + v2(k)*cos(theta2(k))*dt;

    y2(k+1) = y2(k) ...
        + v2(k)*sin(theta2(k))*dt;

    theta2(k+1) = theta2(k) ...
        + omega2(k)*dt;

end

v2(end) = v2(end-1);
omega2(end) = omega2(end-1);

%% =========================================================
% 9. SAI SO BAM
% ==========================================================

positionError = sqrt( ...
    (x2d-x2).^2 + ...
    (y2d-y2).^2 );

%% =========================================================
% 10. FIGURE 1 - QUY DAO
% ==========================================================

figure;

plot(x1,y1,...
    'LineWidth',2,...
    'DisplayName','Leader R1');

hold on;

plot(x2d,y2d,'--',...
    'LineWidth',2,...
    'DisplayName','Desired R2');

plot(x2,y2,...
    'LineWidth',2,...
    'DisplayName','Actual R2');

plot(x2(1),y2(1),'o',...
    'MarkerSize',8,...
    'DisplayName','R2 start');

grid on;
axis equal;

xlabel('x [m]');
ylabel('y [m]');

title('Leader R1 and Follower R2');

legend('Location','best');

%% =========================================================
% 11. FIGURE 2 - POSITION ERROR
% ==========================================================

figure;

plot(t,positionError,...
    'LineWidth',2);

grid on;

xlabel('Time [s]');
ylabel('Position error [m]');

title('R2 Tracking Error');

%% =========================================================
% 12. FIGURE 3 - LINEAR VELOCITY
% ==========================================================

figure;

plot(t,v2,...
    'LineWidth',2);

hold on;

yline(vMax,'--');
yline(-vMax,'--');

grid on;

xlabel('Time [s]');
ylabel('v_2 [m/s]');

title('R2 Linear Velocity');

%% =========================================================
% 13. FIGURE 4 - ANGULAR VELOCITY
% ==========================================================

figure;

plot(t,omega2,...
    'LineWidth',2);

grid on;

xlabel('Time [s]');
ylabel('\omega_2 [rad/s]');

title('R2 Angular Velocity');