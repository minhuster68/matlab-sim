%% STEP 8 - KEYBOARD CONTROL FOR LEADER
clear;
clc;
close all;

%% Tham so mo phong
dt = 0.05;

vCmdMax = 0.15;       % [m/s]
omegaCmdMax = 0.6;    % [rad/s]

%% Trang thai Leader
x1 = 0;
y1 = 0;
theta1 = 0;

%% Tao figure
fig = figure;

hold on;
grid on;
axis equal;

xlim([-5 5]);
ylim([-5 5]);

xlabel('x [m]');
ylabel('y [m]');
title('Keyboard Control - W S A D, Space = Stop');

%% Trang thai phim
keys.w = false;
keys.s = false;
keys.a = false;
keys.d = false;

setappdata(fig,'keys',keys);

%% Callback ban phim
set(fig,...
    'WindowKeyPressFcn',@keyPress,...
    'WindowKeyReleaseFcn',@keyRelease);

%% Ve robot
robotRadius = 0.08;

hRobot = rectangle( ...
    'Position',[x1-robotRadius,...
                y1-robotRadius,...
                2*robotRadius,...
                2*robotRadius],...
    'Curvature',[1 1],...
    'LineWidth',2);

arrowLength = 0.2;

hHeading = quiver( ...
    x1,y1,...
    arrowLength*cos(theta1),...
    arrowLength*sin(theta1),...
    0,...
    'LineWidth',2);

hPath = plot(x1,y1,...
    'LineWidth',1.5);

%% Luu trajectory
xHistory = x1;
yHistory = y1;

%% Main loop
while ishandle(fig)

    %% Doc phim dang duoc giu
    keys = getappdata(fig,'keys');

    %% Toc do tien
    v1 = vCmdMax * ...
        (double(keys.w) - double(keys.s));

    %% Toc do quay
    omega1 = omegaCmdMax * ...
        (double(keys.a) - double(keys.d));

    %% Dong hoc Leader
    x1 = x1 + ...
        v1*cos(theta1)*dt;

    y1 = y1 + ...
        v1*sin(theta1)*dt;

    theta1 = theta1 + ...
        omega1*dt;

    %% Luu duong di
    xHistory(end+1) = x1;
    yHistory(end+1) = y1;

    %% Update robot
    set(hRobot,...
        'Position',[x1-robotRadius,...
                    y1-robotRadius,...
                    2*robotRadius,...
                    2*robotRadius]);

    %% Update huong
    set(hHeading,...
        'XData',x1,...
        'YData',y1,...
        'UData',arrowLength*cos(theta1),...
        'VData',arrowLength*sin(theta1));

    %% Update trajectory
    set(hPath,...
        'XData',xHistory,...
        'YData',yHistory);

    %% Hien thi command
    title(sprintf( ...
        'WASD Control | v = %.2f m/s | omega = %.2f rad/s',...
        v1,omega1));

    drawnow;

    pause(dt);

end


%% =========================================================
% CALLBACK KHI NHAN PHIM
% ==========================================================

function keyPress(src,event)

    keys = getappdata(src,'keys');

    switch event.Key

        case 'w'
            keys.w = true;

        case 's'
            keys.s = true;

        case 'a'
            keys.a = true;

        case 'd'
            keys.d = true;

        case 'space'
            keys.w = false;
            keys.s = false;
            keys.a = false;
            keys.d = false;

    end

    setappdata(src,'keys',keys);

end


%% =========================================================
% CALLBACK KHI THA PHIM
% ==========================================================

function keyRelease(src,event)

    keys = getappdata(src,'keys');

    switch event.Key

        case 'w'
            keys.w = false;

        case 's'
            keys.s = false;

        case 'a'
            keys.a = false;

        case 'd'
            keys.d = false;

    end

    setappdata(src,'keys',keys);

end