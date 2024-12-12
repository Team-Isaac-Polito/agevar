clear 
close all
clc

%% Robot geometric parameters

a = 0.21;
b = 0.3;
l = 0.38;
w = 0.2;
r = 0.12;
modules = 2;

%% Simulation parameters

v1_max = 0.5;
rc_max = 0.4;
w1_max = v1_max/rc_max;

x = [12 64 24];
trajectory = 'Simple case';
%trajectory = 'Simple case smooth_w';
%trajectory = 'S-curve';
%trajectory = 'U-curve';
%trajectory = '<3 shape';

N = 20000;

switch trajectory
    case 'Simple case'
        v1 = v1_max*ones(1,N);
        w1 = w1_max*ones(1,N);
    case 'Simple case smooth_w'
        v1 = v1_max*ones(1,3*N);
        w1 = [zeros(1,N),linspace(0,w1_max,N),w1_max*ones(1,N)];
    case 'S-curve'
        v1 = v1_max*ones(1,8*N);
        w1 = [zeros(1,N), linspace(0,w1_max,N), w1_max*ones(1,N/2), linspace(w1_max,0,N), zeros(1,N), linspace(0,-w1_max,N), -w1_max*ones(1,N/2), linspace(-w1_max,0,N), zeros(1,N)];
    case 'U-curve'
        v1 = v1_max*ones(1,2*N+N);
        w1 = [zeros(1,N), w1_max*ones(1,N), zeros(1,N)];
    case '<3 shape'
        v1_max = 0.56;
        rc_min = 0.1;
        w1_max = v1_max/rc_min;
        v1 = v1_max*ones(1,37*N);
        w1 = [zeros(1,N), linspace(0,w1_max/3,N), linspace(w1_max/3,0,N), zeros(1,5*N), linspace(0,w1_max/4,N), w1_max/4*ones(1,8*N), linspace(w1_max/4,0,N), -w1_max*ones(1,N), linspace(0,w1_max/4,N), w1_max/4*ones(1,8*N), linspace(w1_max/4,0,N), zeros(1,5*N), linspace(0,w1_max/3,N), linspace(w1_max/3,0,N), zeros(1,N)];
    otherwise
        v1 = v1_max*ones(1,N);
        w1 = w1_max*ones(1,N);
end

ts = 10;
t = linspace(0,ts,length(v1));
dt = t(2)-t(1);


V{1} = [v1; zeros(1,length(t));  zeros(1,length(t))];
W{1} = [zeros(1,length(t));    zeros(1,length(t));  w1];

for m = 2:modules
    V{m} = zeros(3,length(t));
    W{m} = zeros(3,length(t));
end

%% Initial conditions
for m = 1:modules
    eta{m}   = nan(3,length(t)); 
    etad{m}  = nan(3,length(t));

    eta{m}(:,1)  = [0,(m-1)*(-a-b),0]'; %Initial orientation and position [g0, x0, y0]'
    etad{m}(:,1) = [0,0,0]';            %Initial angular and linear velocity [gd0, xd0, yd0]'
end

% plotModuleHead([eta_0{1}(2:3)',0]',eta_0{1}(1),l,w,a);
% for m= 2:modules-1
%     plotModuleMiddle([eta_0{m}(2:3)',0]',eta_0{m}(1),l,w,a,b);
% end
% if modules > 1
%     plotModuleTail([eta_0{end}(2:3)',0]',eta_0{end}(1),l,w,b);
% end
% axis equal 
% grid

%% Simulation

for i = 1:length(t)-1

    for m = 2:modules
        th{m-1} = eta{m-1}(1,i)-eta{m}(1,i);
   
        V{m}(1,i) = V{m-1}(1,i)*cos(th{m-1}) + a*W{m-1}(3,i)*sin(th{m-1});
        W{m}(3,i) = (V{m-1}(1,i)*sin(th{m-1}) - a*W{m-1}(3,i)*cos(th{m-1}))/b;
    end
    
    for m = 1:modules
        etad{m}(1,i) = W{m}(3,i);
        eta{m}(1,i+1) = eta{m}(1,i) + dt*etad{m}(1,i);

        vs{m} = Rotz(eta{m}(1,i))*V{m}(:,i);
        etad{m}(2:3,i) = vs{m}(1:2);
        eta{m}(2:3,i+1) = eta{m}(2:3,i) + dt*etad{m}(2:3,i);
    end
end

%% Plot 

figure(2)
plot(eta{1}(2,:),eta{1}(3,:),'--k',"LineWidth",0.75)
grid on 
axis equal
hold on
for ii = 1:1000:length(t)
    cla

    plotModuleHead([eta{1}(2:3,ii)',0]',eta{1}(1,ii),l,w,a);
    for m= 2:modules-1
        plotModuleMiddle([eta{m}(2:3,ii)',0]',eta{m}(1,ii),l,w,a,b);
    end
    if modules > 1
        plotModuleTail([eta{end}(2:3,ii)',0]',eta{end}(1,ii),l,w,b);
    end

    plot(eta{1}(2,:),eta{1}(3,:),'--k',"LineWidth",0.75)
    drawnow;
end

%% Functions

function out = Rotz(th)

    out = [cos(th) -sin(th) 0;
           sin(th)  cos(th) 0;
           0        0       1];
end

function [] = plotModuleHead(p, th, l, w, a)

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

function [] = plotModuleMiddle(p, th, l, w, a, b)

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

function [] = plotModuleTail(p, th, l, w, b)

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