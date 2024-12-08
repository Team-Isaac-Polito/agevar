clear
close all
clc

%% Simulation parameters

while(1)
    for iii=1:5    
        % Robot geometric parameters
        WheelSpan=0.210;
        r = 0.121/2;
        l=WheelSpan+r*2;
        a = WheelSpan/2+0.0965;
        b = WheelSpan/2+0.104;
        w_e = (0.2046-0.0843)/2;    % distance between the center of the 2 tracks
        w_max = 0.2046;    % total width of the robot
    
        %theta_max = rad2deg(atan((b-l/2)/(w_max/2))+asin((a-l/2)/sqrt((w_max/2)^2+(b-l/2)^2)))
        theta_max = 50;
    
        trajectory(1,:) = "Simple case";
        trajectory(2,:) = "Simple case smooth_w";
        trajectory(3,:) = "S-curve";
        trajectory(4,:) = "U-curve";
        trajectory(5,:) = "<3 shape";
        
        N = 20000;

        switch trajectory(iii,:)
            case "Simple case"
                v1_max = 0.5;
                rc_max = 0.4;
                w1_max = v1_max/rc_max;
                v1 = v1_max*ones(1,N);
                w1 = w1_max*ones(1,N);
            case "Simple case smooth_w"
                v1_max = 0.5;
                rc_max = 0.4;
                w1_max = v1_max/rc_max;
                v1 = v1_max*ones(1,3*N);
                w1 = [zeros(1,N),linspace(0,w1_max,N),w1_max*ones(1,N)];
            case "S-curve"
                v1_max = 0.5;
                rc_max = 0.4;
                w1_max = v1_max/rc_max;
                v1 = v1_max*ones(1,8*N);
                w1 = [zeros(1,N), linspace(0,w1_max,N), w1_max*ones(1,N/2), linspace(w1_max,0,N), zeros(1,N), linspace(0,-w1_max,N), -w1_max*ones(1,N/2), linspace(-w1_max,0,N), zeros(1,N)];
            case "U-curve"
                v1_max = 0.5;
                rc_max = 0.50;
                w1_max = v1_max/rc_max;
                v1 = v1_max*ones(1,3*N);
                w1 = [zeros(1,N), w1_max*ones(1,N), zeros(1,N)];
            case "<3 shape"
                v1_max = 0.56;
                rc_max = 0.1;
                w1_max = v1_max/rc_max;
                v1 = v1_max*ones(1,37*N);
                w1 = [zeros(1,N), linspace(0,w1_max/3,N), linspace(w1_max/3,0,N), zeros(1,5*N), linspace(0,w1_max/4,N), w1_max/4*ones(1,8*N), linspace(w1_max/4,0,N), -w1_max*ones(1,N), linspace(0,w1_max/4,N), w1_max/4*ones(1,8*N), linspace(w1_max/4,0,N), zeros(1,5*N), linspace(0,w1_max/3,N), linspace(w1_max/3,0,N), zeros(1,N)];
            otherwise
                v1_max = 0.56;
                rc_max = 0.1;
                w1_max = v1_max/rc_max;
                v1 = v1_max*ones(1,N);
                w1 = w1_max*ones(1,N);
        end
        
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
        eta2_0  = [deg2rad(theta_max),-a-b*cos(deg2rad(theta_max)),-b*sin(deg2rad(theta_max))]';
        eta2d_0 = [0,0,0]';
        eta3_0  = [0,-a-b*cos(deg2rad(theta_max))-a*cos(deg2rad(theta_max))-b,-b*sin(deg2rad(theta_max))-a*sin(deg2rad(theta_max))]';
        eta3d_0 = [0,0,0]';
        
        eta1   = nan(3,length(t)); eta1(:,1)  = eta1_0;
        eta1d  = nan(3,length(t)); eta1d(:,1) = eta1d_0;
        eta2   = nan(3,length(t)); eta2(:,1)  = eta2_0;
        eta2d  = nan(3,length(t)); eta2d(:,1) = eta2d_0;
        eta3   = nan(3,length(t)); eta3(:,1)  = eta3_0;
        eta3d  = nan(3,length(t)); eta3d(:,1) = eta3d_0;
        
        omega1 = nan(2,length(t)); %Angular velocities of the equivalent wheels [left;right]
        omega2 = nan(2,length(t));
        omega3 = nan(2,length(t));
        
        %% Simulation
        
        for i = 1:length(t)-1
        
            th12(i) = eta1(1,i)-eta2(1,i);
            th23(i) = eta2(1,i)-eta3(1,i);
        
            omega1(:,i) = [(V1(1,i)-W1(3,i)*w_e/2)/r;
                           (V1(1,i)+W1(3,i)*w_e/2)/r];
            V2(1,i) = V1(1,i)*cos(th12(i)) + a*W1(3,i)*sin(th12(i));
            W2(3,i) = (V1(1,i)*sin(th12(i)) - a*W1(3,i)*cos(th12(i)))/b;
            omega2(:,i) = [(V2(1,i)-W2(3,i)*w_e/2)/r;
                           (V2(1,i)+W2(3,i)*w_e/2)/r];
            V3(1,i) = V2(1,i)*cos(th23(i)) + a*W2(3,i)*sin(th23(i));
            W3(3,i) = (V2(1,i)*sin(th23(i)) - a*W2(3,i)*cos(th23(i)))/b;
            omega3(:,i) = [(V3(1,i)-W3(3,i)*w_e/2)/r;
                           (V3(1,i)+W3(3,i)*w_e/2)/r];
        
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
        % Initialization of the plots
        subplot(221);
        plot(eta1(2,:), eta1(3,:), '--k', "LineWidth", 0.75); % Static trajectory
        grid on;
        axis equal;
        hold on;

        subplot(222);
        plot(t, [0, rad2deg(th12)], 'k', "LineWidth", 0.75);
        hold on;
        plot(t, [0, rad2deg(th23)], 'b', "LineWidth", 0.75);
        plot([0, t(end)], [theta_max, theta_max], '--r', "LineWidth", 0.75);
        plot([0, t(end)], [-theta_max, -theta_max], '--r', "LineWidth", 0.75);
        grid on;
        xlabel("time ,s");
        ylabel("\theta_i, gradi");
        legend("\theta_{1-2}", "\theta_{2-3}", "\theta_{max}", "\theta_{min}", 'Location', 'southeast');

        subplot(223);
        plot(t, V1(1, :), '--k', "LineWidth", 0.75);
        hold on;
        plot(t, W1(3, :), 'k', "LineWidth", 0.75);
        plot(t, V2(1, :), '--r', "LineWidth", 0.75);
        plot(t, W2(3, :), 'r', "LineWidth", 0.75);
        plot(t, V3(1, :), 'b', "LineWidth", 0.75);
        plot(t, W3(3, :), 'b', "LineWidth", 0.75);
        grid on;
        xlabel("time ,s");
        ylabel("linear and angular speed, m/s rad/s");
        legend("v_1","\omega_1","v_2","\omega_2","v_3","\omega_3", 'Location','southeast');

        subplot(224);
        plot(t, omega1(1, :) * 60 / (2 * pi), '--k', "LineWidth", 0.75);
        hold on;
        plot(t, omega1(2, :) * 60 / (2 * pi), 'k', "LineWidth", 0.75);
        plot(t, omega2(1, :) * 60 / (2 * pi), '--r', "LineWidth", 0.75);
        plot(t, omega2(2, :) * 60 / (2 * pi), 'r', "LineWidth", 0.75);
        plot(t, omega3(1, :) * 60 / (2 * pi), '--b', "LineWidth", 0.75);
        plot(t, omega3(2, :) * 60 / (2 * pi), 'b', "LineWidth", 0.75);
        grid on;
        xlabel("time, s");
        ylabel("\omega_i, rpm");
        legend("\omega_{1l}", "\omega_{1r}", "\omega_{2l}", "\omega_{2r}", "\omega_{3l}", "\omega_{3r}", 'Location', 'southeast');

        % Add xline handles to all subplots
        subplot(222);
        xline_handle_222 = xline(t(1), 'k', "LineWidth", 1.5);
        subplot(223);
        xline_handle_223 = xline(t(1), 'k', "LineWidth", 1.5);
        subplot(224);
        xline_handle_224 = xline(t(1), 'k', "LineWidth", 1.5);
        
        % Dynamic Update Loop
        for ii = 1:round(length(t)/50):length(t)
            % Update xline positions
            set(xline_handle_222, 'Value', t(ii));
            set(xline_handle_223, 'Value', t(ii));
            set(xline_handle_224, 'Value', t(ii));

            if ii==1
                subplot(222)
                legend("\theta_{1-2}", "\theta_{2-3}", "\theta_{max}", "\theta_{min}" ,"Current time", 'Location', 'southeast');
                subplot(223)
                legend("v_1","\omega_1","v_2","\omega_2","v_3","\omega_3", "Current time", 'Location','southeast');
                subplot(224)
                legend("\omega_{1l}", "\omega_{1r}", "\omega_{2l}", "\omega_{2r}", "\omega_{3l}", "\omega_{3r}", "Current time", 'Location', 'southeast');
            end

            % Update the dynamic plot in subplot 221
            subplot(221);
            cla;
            plotModule1([eta1(2:3, ii)', 0]', eta1(1, ii), l, w_max, a);
            plotModule2([eta2(2:3, ii)', 0]', eta2(1, ii), l, w_max, a, b);
            plotModule3([eta3(2:3, ii)', 0]', eta3(1, ii), l, w_max, a, b);
            plot(eta1(2,:), eta1(3,:), '--k', "LineWidth", 0.75);
        
            drawnow;
        end

        clf
        clear
    end
end

%% Functions

function out = Rotz(th)

    out = [cos(th) -sin(th) 0;
           sin(th)  cos(th) 0;
           0        0       1];
end

function [] = plotModule1(p, th, l, w_max, a, b)

    R   = Rotz(th);
    p1  = p + R*[-l/2,-w_max/2,0]';
    p2  = p + R*[+l/2,-w_max/2,0]';
    p3  = p + R*[+l/2,+w_max/2,0]';
    p4  = p + R*[-l/2,+w_max/2,0]';
    pQ1 = p + R*[-l/2,0,0]';
    pQ  = p + R*[-a,0,0]';

    plot([p1(1) p2(1) p3(1) p4(1) p1(1)],[p1(2) p2(2) p3(2) p4(2) p1(2)], 'k', 'LineWidth',2)
    hold on
    plot([pQ1(1) pQ(1)], [pQ1(2) pQ(2)], 'k', 'LineWidth',2)
    plot(pQ(1),pQ(2),'ok', 'LineWidth',2.5)
    quiver(p(1),p(2), 0.15*l*cos(th), 0.15*l*sin(th),"Color",'red','LineWidth',1.0)
    quiver(p(1),p(2), -0.15*l*sin(th), 0.15*l*cos(th),"Color",'green','LineWidth',1.0)
end

function [] = plotModule2(p, th, l, w_max, a, b)

    R   = Rotz(th);
    p1  = p + R*[-l/2,-w_max/2,0]';
    p2  = p + R*[+l/2,-w_max/2,0]';
    p3  = p + R*[+l/2,+w_max/2,0]';
    p4  = p + R*[-l/2,+w_max/2,0]';
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

function [] = plotModule3(p, th, l, w_max, a, b)

    R   = Rotz(th);
    p1  = p + R*[-l/2,-w_max/2,0]';
    p2  = p + R*[+l/2,-w_max/2,0]';
    p3  = p + R*[+l/2,+w_max/2,0]';
    p4  = p + R*[-l/2,+w_max/2,0]';
    pQ1 = p + R*[+l/2,0,0]';
    pQ  = p + R*[+b,0,0]';

    plot([p1(1) p2(1) p3(1) p4(1) p1(1)],[p1(2) p2(2) p3(2) p4(2) p1(2)], 'b', 'LineWidth',2)
    hold on
    plot([pQ1(1) pQ(1)], [pQ1(2) pQ(2)], 'b', 'LineWidth',2)
    plot(pQ(1),pQ(2),'ok', 'LineWidth',2.5)
    quiver(p(1),p(2), 0.15*l*cos(th), 0.15*l*sin(th),"Color",'red','LineWidth',1.0)
    quiver(p(1),p(2), -0.15*l*sin(th), 0.15*l*cos(th),"Color",'green','LineWidth',1.0)
end