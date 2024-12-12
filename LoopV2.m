clear
close all
clc

%% Simulation parameters

while(1)
    for iii=1:5
        for modules = [2 4 6]
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
            
            N = 2000;
    
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
                    rc_max = 0.5;
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
    
                omega{m} = nan(2,length(t)); %Angular velocities of the equivalent wheels [left;right]
            end
            
            %% Simulation
            
            for i = 1:length(t)-1
                omega{1}(:,i) = [(V{1}(1,i)-W{1}(3,i)*w_e/2)/r;
                                 (V{1}(1,i)+W{1}(3,i)*w_e/2)/r];
            
                for m = 2:modules
                    th{m-1}(i) = eta{m-1}(1,i)-eta{m}(1,i);
               
                    V{m}(1,i) = V{m-1}(1,i)*cos(th{m-1}(i)) + a*W{m-1}(3,i)*sin(th{m-1}(i));
                    W{m}(3,i) = (V{m-1}(1,i)*sin(th{m-1}(i)) - a*W{m-1}(3,i)*cos(th{m-1}(i)))/b;
                
                    omega{m}(:,i) = [(V{m}(1,i)-W{m}(3,i)*w_e/2)/r;
                                     (V{m}(1,i)+W{m}(3,i)*w_e/2)/r];
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
            % Initialization of the plots
            subplot(221);
            plot(eta{1}(2,:), eta{1}(3,:), '--k', "LineWidth", 0.75); % Static trajectory
            grid on;
            axis equal;
            hold on;
    
            if modules > 1
                subplot(222);
                plot([0, t(end)], [theta_max, theta_max], '--r', "LineWidth", 0.75);
                hold on;
                plot([0, t(end)], [-theta_max, -theta_max], '--r', "LineWidth", 0.75);
                plot(t, [0, rad2deg(th{1})], 'k', "LineWidth", 0.75);
                for m = 2:modules
                    plot(t, [0, rad2deg(th{m-1})], 'b', "LineWidth", 0.75);
                end
                grid on;
                xlabel("time ,s");
                ylabel("\theta_i, gradi");
                legend("\theta_{max}", "\theta_{min}", "\theta_{1-2}", 'Location', 'southeast');
    
                xline_handle_222 = xline(t(1), 'k', "LineWidth", 1.5);
            end
    
            subplot(223);
            plot(t, V{1}(1, :), '--k', "LineWidth", 0.75);
            hold on;
            plot(t, W{1}(3, :), 'k', "LineWidth", 0.75);
            for m = 2:modules
                plot(t, V{m}(1, :), '--r', "LineWidth", 0.75);
                plot(t, W{m}(3, :), 'r', "LineWidth", 0.75);
            end
            grid on;
            xlabel("time ,s");
            ylabel("linear and angular speed, m/s rad/s");
            legend("v_1","\omega_1", 'Location','southeast');
            xline_handle_223 = xline(t(1), 'k', "LineWidth", 1.5);
    
            subplot(224);
            plot(t, omega{1}(1, :) * 60 / (2 * pi), '--k', "LineWidth", 0.75);
            hold on;
            plot(t, omega{1}(2, :) * 60 / (2 * pi), 'k', "LineWidth", 0.75);
            for m = 2:modules
                plot(t, omega{m}(1, :) * 60 / (2 * pi), '--b', "LineWidth", 0.75);
                plot(t, omega{m}(2, :) * 60 / (2 * pi), 'b', "LineWidth", 0.75);
            end
            grid on;
            xlabel("time, s");
            ylabel("\omega_i, rpm");
            legend("\omega_{1l}", "\omega_{1r}", 'Location', 'southeast');
            xline_handle_224 = xline(t(1), 'k', "LineWidth", 1.5);
            
            % Dynamic Update Loop
            for ii = 1:round(length(t)/50):length(t)
                % Update xline positions
                if modules > 1
                    set(xline_handle_222, 'Value', t(ii));
                end
                set(xline_handle_223, 'Value', t(ii));
                set(xline_handle_224, 'Value', t(ii));
    
                if ii==1
                    if modules > 1
                        subplot(222)
                        legend("\theta_{max}", "\theta_{min}", "\theta_{1-2}", 'Location', 'southeast');
                    end
                    subplot(223)
                    legend("v_1","\omega_1", 'Location','southeast');
                    subplot(224)
                    legend("\omega_{1l}", "\omega_{1r}", 'Location', 'southeast');
                end
    
                % Update the dynamic plot in subplot 221
                subplot(221);
                cla
                plotModuleHead([eta{1}(2:3,ii)',0]',eta{1}(1,ii),l,w_max,a);
                for m= 2:modules-1
                    plotModuleMiddle([eta{m}(2:3,ii)',0]',eta{m}(1,ii),l,w_max,a,b);
                end
                if modules > 1
                    plotModuleTail([eta{end}(2:3,ii)',0]',eta{end}(1,ii),l,w_max,b);
                end
                plot(eta{1}(2,:),eta{1}(3,:),'--k',"LineWidth",0.75)
                drawnow;
            end
    
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

function [] = plotModuleHead(p, th, l, w_max, a)

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

function [] = plotModuleMiddle(p, th, l, w_max, a, b)

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

function [] = plotModuleTail(p, th, l, w_max, b)

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