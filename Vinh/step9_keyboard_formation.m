%% STEP 9 - KEYBOARD LEADER + TRIANGLE FORMATION
clear;
clc;
close all;

%% =========================================================
% 1. THAM SO MO PHONG
% ==========================================================

dt = 0.05;

%% =========================================================
% 2. DOI HINH TAM GIAC
% ==========================================================

d = 0.8;      % canh tam giac [m]

% R2: sau - trai
r2x = -sqrt(3)/2*d;
r2y =  1/2*d;

% R3: sau - phai
r3x = -sqrt(3)/2*d;
r3y = -1/2*d;

%% =========================================================
% 3. GIOI HAN LEADER
% ==========================================================

% Chon nho hon gioi han TurtleBot3
% de ca formation co the theo kip

vLeaderMax     = 0.10;    % [m/s]
omegaLeaderMax = 0.15;    % [rad/s]

% Lam muot lenh ban phim
tauV = 0.25;
tauW = 0.25;

alphaV = dt/(tauV + dt);
alphaW = dt/(tauW + dt);

%% =========================================================
% 4. GIOI HAN FOLLOWER
% ==========================================================

vMax     = 0.22;     % [m/s]
omegaMax = 2.84;     % [rad/s]

%% =========================================================
% 5. THAM SO CONTROLLER
% ==========================================================

kx     = 0.8;
ky     = 1.5;
ktheta = 1.5;

%% =========================================================
% 6. TRANG THAI LEADER R1
% ==========================================================

x1     = 0;
y1     = 0;
theta1 = 0;

v1     = 0;
omega1 = 0;

%% =========================================================
% 7. VI TRI MONG MUON BAN DAU CUA R2, R3
% ==========================================================

c = cos(theta1);
s = sin(theta1);

offset2x = c*r2x - s*r2y;
offset2y = s*r2x + c*r2y;

offset3x = c*r3x - s*r3y;
offset3y = s*r3x + c*r3y;

x2d = x1 + offset2x;
y2d = y1 + offset2y;

x3d = x1 + offset3x;
y3d = y1 + offset3y;

%% =========================================================
% 8. TRANG THAI BAN DAU FOLLOWERS
% ==========================================================

% Dat dung formation ngay tu dau
x2 = x2d;
y2 = y2d;
theta2 = 0;

x3 = x3d;
y3 = y3d;
theta3 = 0;

v2 = 0;
omega2 = 0;

v3 = 0;
omega3 = 0;

%% Huong mong muon truoc do
theta2dPrev = 0;
theta3dPrev = 0;

%% =========================================================
% 9. TAO FIGURE
% ==========================================================

fig = figure;

hold on;
grid on;
axis equal;

xlim([-3 5]);
ylim([-3 5]);

xlabel('x [m]');
ylabel('y [m]');

title('WASD Leader Control');

%% =========================================================
% 10. TRANG THAI BAN PHIM
% ==========================================================

keys.w = false;
keys.s = false;
keys.a = false;
keys.d = false;

setappdata(fig,'keys',keys);
setappdata(fig,'stop',false);

set(fig,...
    'WindowKeyPressFcn',@keyPress,...
    'WindowKeyReleaseFcn',@keyRelease);

%% =========================================================
% 11. VE ROBOT
% ==========================================================

robotRadius = 0.08;
arrowLength = 0.18;

%% R1
hR1 = rectangle( ...
    'Position',[x1-robotRadius,...
                y1-robotRadius,...
                2*robotRadius,...
                2*robotRadius],...
    'Curvature',[1 1],...
    'LineWidth',2);

%% R2
hR2 = rectangle( ...
    'Position',[x2-robotRadius,...
                y2-robotRadius,...
                2*robotRadius,...
                2*robotRadius],...
    'Curvature',[1 1],...
    'LineWidth',2);

%% R3
hR3 = rectangle( ...
    'Position',[x3-robotRadius,...
                y3-robotRadius,...
                2*robotRadius,...
                2*robotRadius],...
    'Curvature',[1 1],...
    'LineWidth',2);

%% =========================================================
% 12. MUI TEN HUONG ROBOT
% ==========================================================

hHeading1 = quiver( ...
    x1,y1,...
    arrowLength*cos(theta1),...
    arrowLength*sin(theta1),...
    0,'LineWidth',2);

hHeading2 = quiver( ...
    x2,y2,...
    arrowLength*cos(theta2),...
    arrowLength*sin(theta2),...
    0,'LineWidth',2);

hHeading3 = quiver( ...
    x3,y3,...
    arrowLength*cos(theta3),...
    arrowLength*sin(theta3),...
    0,'LineWidth',2);

%% =========================================================
% 13. DIEM MONG MUON CUA FOLLOWERS
% ==========================================================

hDesired2 = plot( ...
    x2d,y2d,'x',...
    'MarkerSize',10,...
    'LineWidth',2);

hDesired3 = plot( ...
    x3d,y3d,'x',...
    'MarkerSize',10,...
    'LineWidth',2);

%% =========================================================
% 14. TAM GIAC FORMATION
% ==========================================================

hFormation = plot( ...
    [x1 x2 x3 x1],...
    [y1 y2 y3 y1],...
    '-',...
    'LineWidth',1.5);

%% =========================================================
% 15. QUY DAO DA DI
% ==========================================================

hPath1 = plot(x1,y1,'LineWidth',1.5);
hPath2 = plot(x2,y2,'LineWidth',1.5);
hPath3 = plot(x3,y3,'LineWidth',1.5);

%% Labels

hText1 = text(x1,y1+0.15,'R1');
hText2 = text(x2,y2+0.15,'R2');
hText3 = text(x3,y3+0.15,'R3');

%% =========================================================
% 16. BIEN LUU DU LIEU
% ==========================================================

timeHistory = [];

x1History = [];
y1History = [];

x2History = [];
y2History = [];

x3History = [];
y3History = [];

error2History = [];
error3History = [];

d12History = [];
d13History = [];
d23History = [];

v1History = [];
omega1History = [];

v2History = [];
omega2History = [];

v3History = [];
omega3History = [];

%% =========================================================
% 17. MAIN LOOP
% ==========================================================

simTime = 0;

while ishandle(fig) && ~getappdata(fig,'stop')

    %% -----------------------------------------------------
    % A. DOC BAN PHIM
    % ------------------------------------------------------

    keys = getappdata(fig,'keys');

    vTarget = ...
        vLeaderMax * ...
        (double(keys.w) - double(keys.s));

    omegaTarget = ...
        omegaLeaderMax * ...
        (double(keys.a) - double(keys.d));

    %% -----------------------------------------------------
    % B. LAM MUOT LENH LEADER
    % ------------------------------------------------------

    v1 = v1 ...
        + alphaV*(vTarget - v1);

    omega1 = omega1 ...
        + alphaW*(omegaTarget - omega1);

    %% -----------------------------------------------------
    % C. CAP NHAT LEADER
    % ------------------------------------------------------

    x1 = x1 ...
        + v1*cos(theta1)*dt;

    y1 = y1 ...
        + v1*sin(theta1)*dt;

    theta1 = theta1 ...
        + omega1*dt;

    % Dua goc ve [-pi, pi]
    theta1 = atan2( ...
        sin(theta1),...
        cos(theta1));

    %% -----------------------------------------------------
    % D. TAO FORMATION REFERENCE
    % ------------------------------------------------------

    c = cos(theta1);
    s = sin(theta1);

    %% ----- Offset R2 -----

    offset2x = ...
        c*r2x - s*r2y;

    offset2y = ...
        s*r2x + c*r2y;

    x2d = x1 + offset2x;
    y2d = y1 + offset2y;

    %% ----- Offset R3 -----

    offset3x = ...
        c*r3x - s*r3y;

    offset3y = ...
        s*r3x + c*r3y;

    x3d = x1 + offset3x;
    y3d = y1 + offset3y;

    %% -----------------------------------------------------
    % E. VAN TOC MONG MUON R2
    % ------------------------------------------------------

    dx2d = ...
        v1*cos(theta1) ...
        - omega1*offset2y;

    dy2d = ...
        v1*sin(theta1) ...
        + omega1*offset2x;

    v2d = hypot(dx2d,dy2d);

    if v2d > 1e-3

        theta2d = atan2(dy2d,dx2d);

        deltaTheta = atan2( ...
            sin(theta2d-theta2dPrev),...
            cos(theta2d-theta2dPrev));

        omega2d = deltaTheta/dt;

    else

        theta2d = theta2dPrev;
        omega2d = 0;

    end

    theta2dPrev = theta2d;

    %% -----------------------------------------------------
    % F. VAN TOC MONG MUON R3
    % ------------------------------------------------------

    dx3d = ...
        v1*cos(theta1) ...
        - omega1*offset3y;

    dy3d = ...
        v1*sin(theta1) ...
        + omega1*offset3x;

    v3d = hypot(dx3d,dy3d);

    if v3d > 1e-3

        theta3d = atan2(dy3d,dx3d);

        deltaTheta = atan2( ...
            sin(theta3d-theta3dPrev),...
            cos(theta3d-theta3dPrev));

        omega3d = deltaTheta/dt;

    else

        theta3d = theta3dPrev;
        omega3d = 0;

    end

    theta3dPrev = theta3d;

    %% Gioi han feedforward omega
    omegaRefMax = 1.5;

    omega2d = ...
        max(min(omega2d,omegaRefMax),...
        -omegaRefMax);

    omega3d = ...
        max(min(omega3d,omegaRefMax),...
        -omegaRefMax);

    %% =====================================================
    % G. CONTROLLER R2
    % ======================================================

    px2 = x2d - x2;
    py2 = y2d - y2;

    etheta2 = ...
        theta2d - theta2;

    etheta2 = atan2( ...
        sin(etheta2),...
        cos(etheta2));

    ex2 = ...
        cos(theta2)*px2 ...
        + sin(theta2)*py2;

    ey2 = ...
        -sin(theta2)*px2 ...
        + cos(theta2)*py2;

    v2 = ...
        v2d*cos(etheta2) ...
        + kx*ex2;

    omega2 = ...
        omega2d ...
        + ky*ey2 ...
        + ktheta*sin(etheta2);

    %% Saturation

    v2 = ...
        max(min(v2,vMax),-vMax);

    omega2 = ...
        max(min(omega2,omegaMax),...
        -omegaMax);

    %% Dong hoc R2

    x2 = x2 ...
        + v2*cos(theta2)*dt;

    y2 = y2 ...
        + v2*sin(theta2)*dt;

    theta2 = theta2 ...
        + omega2*dt;

    theta2 = atan2( ...
        sin(theta2),...
        cos(theta2));

    %% =====================================================
    % H. CONTROLLER R3
    % ======================================================

    px3 = x3d - x3;
    py3 = y3d - y3;

    etheta3 = ...
        theta3d - theta3;

    etheta3 = atan2( ...
        sin(etheta3),...
        cos(etheta3));

    ex3 = ...
        cos(theta3)*px3 ...
        + sin(theta3)*py3;

    ey3 = ...
        -sin(theta3)*px3 ...
        + cos(theta3)*py3;

    v3 = ...
        v3d*cos(etheta3) ...
        + kx*ex3;

    omega3 = ...
        omega3d ...
        + ky*ey3 ...
        + ktheta*sin(etheta3);

    %% Saturation

    v3 = ...
        max(min(v3,vMax),-vMax);

    omega3 = ...
        max(min(omega3,omegaMax),...
        -omegaMax);

    %% Dong hoc R3

    x3 = x3 ...
        + v3*cos(theta3)*dt;

    y3 = y3 ...
        + v3*sin(theta3)*dt;

    theta3 = theta3 ...
        + omega3*dt;

    theta3 = atan2( ...
        sin(theta3),...
        cos(theta3));

    %% =====================================================
    % I. TINH SAI SO
    % ======================================================

    error2 = hypot( ...
        x2d-x2,...
        y2d-y2);

    error3 = hypot( ...
        x3d-x3,...
        y3d-y3);

    %% Khoang cach

    d12 = hypot(x1-x2,y1-y2);
    d13 = hypot(x1-x3,y1-y3);
    d23 = hypot(x2-x3,y2-y3);

    %% =====================================================
    % J. LUU DU LIEU
    % ======================================================

    simTime = simTime + dt;

    timeHistory(end+1) = simTime;

    x1History(end+1) = x1;
    y1History(end+1) = y1;

    x2History(end+1) = x2;
    y2History(end+1) = y2;

    x3History(end+1) = x3;
    y3History(end+1) = y3;

    error2History(end+1) = error2;
    error3History(end+1) = error3;

    d12History(end+1) = d12;
    d13History(end+1) = d13;
    d23History(end+1) = d23;

    v1History(end+1) = v1;
    omega1History(end+1) = omega1;

    v2History(end+1) = v2;
    omega2History(end+1) = omega2;

    v3History(end+1) = v3;
    omega3History(end+1) = omega3;

    %% =====================================================
    % K. UPDATE ANIMATION
    % ======================================================

    set(hR1,...
        'Position',[x1-robotRadius,...
                    y1-robotRadius,...
                    2*robotRadius,...
                    2*robotRadius]);

    set(hR2,...
        'Position',[x2-robotRadius,...
                    y2-robotRadius,...
                    2*robotRadius,...
                    2*robotRadius]);

    set(hR3,...
        'Position',[x3-robotRadius,...
                    y3-robotRadius,...
                    2*robotRadius,...
                    2*robotRadius]);

    %% Heading

    set(hHeading1,...
        'XData',x1,...
        'YData',y1,...
        'UData',arrowLength*cos(theta1),...
        'VData',arrowLength*sin(theta1));

    set(hHeading2,...
        'XData',x2,...
        'YData',y2,...
        'UData',arrowLength*cos(theta2),...
        'VData',arrowLength*sin(theta2));

    set(hHeading3,...
        'XData',x3,...
        'YData',y3,...
        'UData',arrowLength*cos(theta3),...
        'VData',arrowLength*sin(theta3));

    %% Desired points

    set(hDesired2,...
        'XData',x2d,...
        'YData',y2d);

    set(hDesired3,...
        'XData',x3d,...
        'YData',y3d);

    %% Formation

    set(hFormation,...
        'XData',[x1 x2 x3 x1],...
        'YData',[y1 y2 y3 y1]);

    %% Path

    set(hPath1,...
        'XData',x1History,...
        'YData',y1History);

    set(hPath2,...
        'XData',x2History,...
        'YData',y2History);

    set(hPath3,...
        'XData',x3History,...
        'YData',y3History);

    %% Labels

    set(hText1,...
        'Position',[x1 y1+0.15 0]);

    set(hText2,...
        'Position',[x2 y2+0.15 0]);

    set(hText3,...
        'Position',[x3 y3+0.15 0]);

    %% Title

    title(sprintf( ...
        ['WASD Leader | t = %.1f s | ',...
         'v_1 = %.2f | omega_1 = %.2f'],...
        simTime,v1,omega1));

    drawnow;

    pause(dt);

end

%% =========================================================
% 18. VE KET QUA SAU KHI NHAN ESC
% ==========================================================

if ~isempty(timeHistory)

    %% Tracking error

    figure;

    plot(timeHistory,error2History,...
        'LineWidth',2,...
        'DisplayName','R2');

    hold on;

    plot(timeHistory,error3History,...
        'LineWidth',2,...
        'DisplayName','R3');

    grid on;

    xlabel('Time [s]');
    ylabel('Tracking error [m]');

    title('Follower Tracking Errors');

    legend('Location','best');

    %% Formation distance

    figure;

    plot(timeHistory,d12History,...
        'LineWidth',2,...
        'DisplayName','d_{12}');

    hold on;

    plot(timeHistory,d13History,...
        'LineWidth',2,...
        'DisplayName','d_{13}');

    plot(timeHistory,d23History,...
        'LineWidth',2,...
        'DisplayName','d_{23}');

    yline(d,'--',...
        'LineWidth',1.5,...
        'DisplayName','Desired = 0.8 m');

    grid on;

    xlabel('Time [s]');
    ylabel('Distance [m]');

    title('Formation Distances');

    legend('Location','best');

end


%% =========================================================
% CALLBACK - NHAN PHIM
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

        case 'escape'

            setappdata(src,'stop',true);

    end

    setappdata(src,'keys',keys);

end


%% =========================================================
% CALLBACK - THA PHIM
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