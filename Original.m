% kineSIM.m 

clear 
close all
clc

%% Robot geometric parameters

a = 0.21;
b = 0.3;
l = 0.38;
w = 0.2;
r = 0.12;

%% Simulation parameters

v1_max = 2;
rc_max = 1;
w1_max = v1_max/rc_max;

N = 20000;

% Simple case:
v1 = [v1_max*ones(1,3*N)];
w1 = [linspace(0,w1_max,N), w1_max*ones(1,2*N)];

% S-curve:
% v1 = [v1_max*ones(1,9*N)];
% w1 = [zeros(1,N), linspace(0,w1_max,N), w1_max*ones(1,N), linspace(w1_max,0,N), zeros(1,N), linspace(0,-w1_max,N), -w1_max*ones(1,N), linspace(-w1_max,0,N), zeros(1,N)];

% U-curve:
% v1 = [v1_max*ones(1,5*N)];
% w1 = [zeros(1,N), linspace(0,w1_max,N), w1_max*ones(1,N), linspace(w1_max,0,N), zeros(1,N)];

ts = 10;
t = linspace(0,ts,length(v1));
dt = t(2)-t(1);


V1 = [v1; zeros(1,length(t));  zeros(1,length(t))];
W1 = [zeros(1,length(t));    zeros(1,length(t));  w1];

V2 = zeros(3,length(t));
W2 = zeros(3,length(t));

V3 = zeros(3,length(t));
W3 = zeros(3,length(t));

%% Initial conditions

eta1_0  = [0,0,0]';         %Initial orientation and position [g0, x0, y0]'
eta1d_0 = [0,0,0]';         %Initial angular and linear velocity [gd0, xd0, yd0]'

eta2_0  = [0,-a-b,0]';      %Initial orientation and position [g0, x0, y0]'
eta2d_0 = [0,0,0]';         %Initial angular and linear velocity [gd0, xd0, yd0]'

eta3_0  = [0,-2*a-2*b,0]';      %Initial orientation and position [g0, x0, y0]'
eta3d_0 = [0,0,0]';         %Initial angular and linear velocity [gd0, xd0, yd0]'

eta1   = nan(3,length(t)); eta1(:,1)  = eta1_0;
eta1d  = nan(3,length(t)); eta1d(:,1) = eta1d_0;

eta2   = nan(3,length(t)); eta2(:,1)  = eta2_0;
eta2d  = nan(3,length(t)); eta2d(:,1) = eta2d_0;

eta3   = nan(3,length(t)); eta3(:,1)  = eta3_0;
eta3d  = nan(3,length(t)); eta3d(:,1) = eta3d_0;

omega1 = nan(2,length(t)); %Angular velocities of the equivalent wheels [left;right]
omega2 = nan(2,length(t));
omega3 = nan(2,length(t));

% figure(1)
% plotModule1([eta1_0(2:3)',0]',eta1_0(1),l,w,a);
% plotModule2([eta2_0(2:3)',0]',eta2_0(1),l,w,a,b);
% plotModule3([eta3_0(2:3)',0]',eta3_0(1),l,w,a,b);
% axis equal 
% grid

%% Simulation

for i = 1:length(t)-1
    th12 = eta1(1,i)-eta2(1,i);
    th23 = eta2(1,i)-eta3(1,i);

    omega1(:,i) = [(V1(1,i)-W1(3,i)*w/2)/r;
                   (V1(1,i)+W1(3,i)*w/2)/r];
    
    V2(1,i) = V1(1,i)*cos(th12) + a*W1(3,i)*sin(th12);
    W2(3,i) = (V1(1,i)*sin(th12) - a*W1(3,i)*cos(th12))/b;
    
    omega2(:,i) = [(V2(1,i)-W2(3,i)*w/2)/r;
                   (V2(1,i)+W2(3,i)*w/2)/r];

    V3(1,i) = V2(1,i)*cos(th23) + a*W2(3,i)*sin(th23);
    W3(3,i) = (V2(1,i)*sin(th23) - a*W2(3,i)*cos(th23))/b;
    
    omega3(:,i) = [(V3(1,i)-W3(3,i)*w/2)/r;
                   (V3(1,i)+W3(3,i)*w/2)/r];

    eta1d(1,i) = W1(3,i);
    eta2d(1,i) = W2(3,i);
    eta3d(1,i) = W3(3,i);

    eta1(1,i+1) = eta1(1,i) + dt*eta1d(1,i);
    eta2(1,i+1) = eta2(1,i) + dt*eta2d(1,i);
    eta3(1,i+1) = eta3(1,i) + dt*eta3d(1,i);
    
    vs1 = Rotz(eta1(1,i))*V1(:,i);
    eta1d(2:3,i) = vs1(1:2);

    vs2 = Rotz(eta2(1,i))*V2(:,i);
    eta2d(2:3,i) = vs2(1:2);

    vs3 = Rotz(eta3(1,i))*V3(:,i);
    eta3d(2:3,i) = vs3(1:2);

    eta1(2:3,i+1) = eta1(2:3,i) + dt*eta1d(2:3,i);
    eta2(2:3,i+1) = eta2(2:3,i) + dt*eta2d(2:3,i);
    eta3(2:3,i+1) = eta3(2:3,i) + dt*eta3d(2:3,i);
end

%% Plot 

figure(2)
plot(eta1(2,:),eta1(3,:),'--k',"LineWidth",0.75)
grid on 
axis equal
hold on
for ii = 1:1000:length(t)
    cla
    plotModule1([eta1(2:3,ii)',0]',eta1(1,ii),l,w,a);
    plotModule2([eta2(2:3,ii)',0]',eta2(1,ii),l,w,a,b);
    plotModule3([eta3(2:3,ii)',0]',eta3(1,ii),l,w,a,b);
    plot(eta1(2,:),eta1(3,:),'--k',"LineWidth",0.75)
    grid on 
    drawnow;
end


figure(3)
plot(t,omega1(1,:)*60/(2*pi),'--k',"LineWidth",0.75);
grid on
hold on
plot(t,omega1(2,:)*60/(2*pi),'k',"LineWidth",0.75);
plot(t,omega2(1,:)*60/(2*pi),'--r',"LineWidth",0.75);
plot(t,omega2(2,:)*60/(2*pi),'r',"LineWidth",0.75);
plot(t,omega3(1,:)*60/(2*pi),'--b',"LineWidth",0.75);
plot(t,omega3(2,:)*60/(2*pi),'b',"LineWidth",0.75);
plot(t,th12,'g',"LineWidth",0.75);
plot(t,th23,'p',"LineWidth",0.75);
xlabel("time, s")
ylabel("\omega_i, rpm")
legend("\omega_{1l}","\omega_{1r}","\omega_{2l}","\omega_{2r}","\omega_{3l}","\omega_{3r}","\theta_{1-2}","\theta_{2-3}")

%% Functions

function out = Rotz(th)

    out = [cos(th) -sin(th) 0;
           sin(th)  cos(th) 0;
           0        0       1];
end

function [] = plotModule1(p, th, l, w, a, b)

    R   = Rotz(th);
    p1  = p + R*[-l/2,-w/2,0]';
    p2  = p + R*[+l/2,-w/2,0]';
    p3  = p + R*[+l/2,+w/2,0]';
    p4  = p + R*[-l/2,+w/2,0]';
    pQ1 = p + R*[-l/2,0,0]';
    pQ  = p + R*[-a,0,0]';

    plot([p1(1) p2(1) p3(1) p4(1) p1(1)],[p1(2) p2(2) p3(2) p4(2) p1(2)], 'k', 'LineWidth',2)
    hold on
    plot([pQ1(1) pQ(1)], [pQ1(2) pQ(2)], 'k', 'LineWidth',2)
    plot(pQ(1),pQ(2),'ok', 'LineWidth',2.5)
    quiver(p(1),p(2), 0.15*l*cos(th), 0.15*l*sin(th),"Color",'red','LineWidth',1.0)
    quiver(p(1),p(2), -0.15*l*sin(th), 0.15*l*cos(th),"Color",'green','LineWidth',1.0)
end

function [] = plotModule2(p, th, l, w, a, b)

    R   = Rotz(th);
    p1  = p + R*[-l/2,-w/2,0]';
    p2  = p + R*[+l/2,-w/2,0]';
    p3  = p + R*[+l/2,+w/2,0]';
    p4  = p + R*[-l/2,+w/2,0]';
    pQ1 = p + R*[+l/2,0,0]';
    pQ  = p + R*[+b,0,0]';
    pQ2 = p + R*[-l/2,0,0]';
    pQ_2  = p + R*[-a,0,0]';

    plot([p1(1) p2(1) p3(1) p4(1) p1(1)],[p1(2) p2(2) p3(2) p4(2) p1(2)], 'r', 'LineWidth',2)
    hold on
    plot([pQ1(1) pQ(1)], [pQ1(2) pQ(2)], 'r', 'LineWidth',2)
    plot(pQ(1),pQ(2),'ok', 'LineWidth',2.5)
    plot([pQ2(1) pQ_2(1)], [pQ2(2) pQ_2(2)], 'k', 'LineWidth',2)
    plot(pQ_2(1),pQ_2(2),'ok', 'LineWidth',2.5)
    quiver(p(1),p(2), 0.15*l*cos(th), 0.15*l*sin(th),"Color",'red','LineWidth',1.0)
    quiver(p(1),p(2), -0.15*l*sin(th), 0.15*l*cos(th),"Color",'green','LineWidth',1.0)
end

function [] = plotModule3(p, th, l, w, a, b)

    R   = Rotz(th);
    p1  = p + R*[-l/2,-w/2,0]';
    p2  = p + R*[+l/2,-w/2,0]';
    p3  = p + R*[+l/2,+w/2,0]';
    p4  = p + R*[-l/2,+w/2,0]';
    pQ1 = p + R*[+l/2,0,0]';
    pQ  = p + R*[+b,0,0]';

    plot([p1(1) p2(1) p3(1) p4(1) p1(1)],[p1(2) p2(2) p3(2) p4(2) p1(2)], 'b', 'LineWidth',2)
    hold on
    plot([pQ1(1) pQ(1)], [pQ1(2) pQ(2)], 'b', 'LineWidth',2)
    plot(pQ(1),pQ(2),'ok', 'LineWidth',2.5)
    quiver(p(1),p(2), 0.15*l*cos(th), 0.15*l*sin(th),"Color",'red','LineWidth',1.0)
    quiver(p(1),p(2), -0.15*l*sin(th), 0.15*l*cos(th),"Color",'green','LineWidth',1.0)
end