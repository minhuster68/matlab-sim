%% STEP 7 - SMOOTH LEADER FOR 3 TURTLEBOT3 FORMATION
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
% 2. LEADER R1 - SMOOTH MOTION
% ==========================================================

x1     = zeros(1,N);
y1     = zeros(1,N);
theta1 = zeros(1,N);

v1     = zeros(1,N);
omega1 = zeros(1,N);

x1(1) = 0;
y1(1) = 0;
theta1(1) = 0;

% Toc do tien cua Leader
vLeader = 0.15;          % [m/s]

% Toc do goc lon nhat khi re
omegaTurn = 0.15;        % [rad/s]

% Khoang chuyen tiep muot
tRampUpStart   = 9;
tRampUpEnd     = 11;

tRampDownStart = 19;
tRampDownEnd   = 21;

for k = 1:N-1

    %% Toc do tien
    v1(k) = vLeader;

    %% Tao omega1 muot

    if t(k) < tRampUpStart

        % 0 -> 9 s: di thang
        omega1(k) = 0;

    elseif t(k) < tRampUpEnd

        % 9 -> 11 s: tang dan omega tu 0 den omegaTurn

        tau = ...
            (t(k) - tRampUpStart) / ...
            (tRampUpEnd - tRampUpStart);

        omega1(k) = ...
            omegaTurn * ...
            0.5 * (1 - cos(pi*tau));

    elseif t(k) < tRampDownStart

        % 11 -> 19 s: quay deu
        omega1(k) = omegaTurn;

    elseif t(k) < tRampDownEnd

        % 19 -> 21 s: giam dan omega ve 0

        tau = ...
            (t(k) - tRampDownStart) / ...
            (tRampDownEnd - tRampDownStart);

        omega1(k) = ...
            omegaTurn * ...
            0.5 * (1 + cos(pi*tau));

    else

        % 21 -> 30 s: di thang
        omega1(k) = 0;

    end

    %% Dong hoc Leader

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
% KIEM TRA OMEGA CUA LEADER
% ==========================================================

figure;

plot(t,omega1,'LineWidth',2);

grid on;

xlabel('Time [s]');
ylabel('\omega_1 [rad/s]');

title('Smooth Leader Angular Velocity');

%% =========================================================
% KIEM TRA ANGULAR VELOCITY CUA FOLLOWERS
% ==========================================================

figure;

plot(t,omega2,...
    'LineWidth',2,...
    'DisplayName','R2');

hold on;

plot(t,omega3,...
    'LineWidth',2,...
    'DisplayName','R3');

yline(omegaMax,'--',...
    'DisplayName','+ limit');

yline(-omegaMax,'--',...
    'DisplayName','- limit');

grid on;

xlabel('Time [s]');
ylabel('\omega [rad/s]');

title('Follower Angular Velocities');

legend('Location','best');

%% =========================================================
% KIEM TRA TRACKING ERROR CUA FOLLOWERS
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
% 13. ANIMATION 3 TURTLEBOT3
% ==========================================================

figure;

hold on;
grid on;
axis equal;

xlabel('x [m]');
ylabel('y [m]');
title('3 TurtleBot3 Leader-Follower Animation');

% Gioi han khung hinh
xlim([-1 3.5]);
ylim([-1 3]);

%% Ve toan bo quy dao mo
plot(x1,y1,'--','LineWidth',1);
plot(x2d,y2d,':','LineWidth',1);
plot(x3d,y3d,':','LineWidth',1);

%% Ban kinh robot chi de hien thi
robotRadius = 0.08;

%% Tao 3 robot
hR1 = rectangle( ...
    'Position',[x1(1)-robotRadius, ...
                y1(1)-robotRadius, ...
                2*robotRadius, ...
                2*robotRadius], ...
    'Curvature',[1 1], ...
    'LineWidth',2);

hR2 = rectangle( ...
    'Position',[x2(1)-robotRadius, ...
                y2(1)-robotRadius, ...
                2*robotRadius, ...
                2*robotRadius], ...
    'Curvature',[1 1], ...
    'LineWidth',2);

hR3 = rectangle( ...
    'Position',[x3(1)-robotRadius, ...
                y3(1)-robotRadius, ...
                2*robotRadius, ...
                2*robotRadius], ...
    'Curvature',[1 1], ...
    'LineWidth',2);

%% Mui ten chi huong robot
arrowLength = 0.18;

hHeading1 = quiver( ...
    x1(1),y1(1), ...
    arrowLength*cos(theta1(1)), ...
    arrowLength*sin(theta1(1)), ...
    0,'LineWidth',2);

hHeading2 = quiver( ...
    x2(1),y2(1), ...
    arrowLength*cos(theta2(1)), ...
    arrowLength*sin(theta2(1)), ...
    0,'LineWidth',2);

hHeading3 = quiver( ...
    x3(1),y3(1), ...
    arrowLength*cos(theta3(1)), ...
    arrowLength*sin(theta3(1)), ...
    0,'LineWidth',2);

%% Duong noi tao tam giac
hFormation = plot( ...
    [x1(1) x2(1) x3(1) x1(1)], ...
    [y1(1) y2(1) y3(1) y1(1)], ...
    '-','LineWidth',1.5);

%% Duong robot da di qua
hPath1 = plot(x1(1),y1(1),'LineWidth',1.5);
hPath2 = plot(x2(1),y2(1),'LineWidth',1.5);
hPath3 = plot(x3(1),y3(1),'LineWidth',1.5);

%% Ten robot
hText1 = text(x1(1),y1(1)+0.15,'R1');
hText2 = text(x2(1),y2(1)+0.15,'R2');
hText3 = text(x3(1),y3(1)+0.15,'R3');

%% =========================================================
% 14. CHAY ANIMATION
% ==========================================================

skip = 2;

for k = 1:skip:N

    %% Cap nhat vi tri robot R1
    set(hR1, ...
        'Position',[x1(k)-robotRadius, ...
                    y1(k)-robotRadius, ...
                    2*robotRadius, ...
                    2*robotRadius]);

    %% Cap nhat R2
    set(hR2, ...
        'Position',[x2(k)-robotRadius, ...
                    y2(k)-robotRadius, ...
                    2*robotRadius, ...
                    2*robotRadius]);

    %% Cap nhat R3
    set(hR3, ...
        'Position',[x3(k)-robotRadius, ...
                    y3(k)-robotRadius, ...
                    2*robotRadius, ...
                    2*robotRadius]);

    %% Cap nhat huong R1
    set(hHeading1, ...
        'XData',x1(k), ...
        'YData',y1(k), ...
        'UData',arrowLength*cos(theta1(k)), ...
        'VData',arrowLength*sin(theta1(k)));

    %% Cap nhat huong R2
    set(hHeading2, ...
        'XData',x2(k), ...
        'YData',y2(k), ...
        'UData',arrowLength*cos(theta2(k)), ...
        'VData',arrowLength*sin(theta2(k)));

    %% Cap nhat huong R3
    set(hHeading3, ...
        'XData',x3(k), ...
        'YData',y3(k), ...
        'UData',arrowLength*cos(theta3(k)), ...
        'VData',arrowLength*sin(theta3(k)));

    %% Cap nhat tam giac formation
    set(hFormation, ...
        'XData',[x1(k) x2(k) x3(k) x1(k)], ...
        'YData',[y1(k) y2(k) y3(k) y1(k)]);

    %% Cap nhat duong da di
    set(hPath1, ...
        'XData',x1(1:k), ...
        'YData',y1(1:k));

    set(hPath2, ...
        'XData',x2(1:k), ...
        'YData',y2(1:k));

    set(hPath3, ...
        'XData',x3(1:k), ...
        'YData',y3(1:k));

    %% Cap nhat label
    set(hText1, ...
        'Position',[x1(k) y1(k)+0.15 0]);

    set(hText2, ...
        'Position',[x2(k) y2(k)+0.15 0]);

    set(hText3, ...
        'Position',[x3(k) y3(k)+0.15 0]);

    %% Hien thi thoi gian
    title(sprintf( ...
        '3 TurtleBot3 Leader-Follower - Time = %.1f s', ...
        t(k)));

    drawnow;

    pause(0.01);

end