clear
close all
clc

%% AGEVAR - Multi-Module Robot Kinematics Simulation (DEMO VERSION)
% Continuous loop version for fairs and demonstrations
% This script simulates the kinematics of a multi-module articulated robot
% Each module has differential drive wheels and is connected via passive joints
%
% For development version, check the 'main' branch

%% Simulation parameters

while(1)
    for iii = 1:5
        for modules = [2 4 6]
            
            %% Robot geometric parameters
            WheelSpan = 0.210;              % Distance between wheels [m]
            r = 0.121/2;                    % Wheel radius [m]
            l = WheelSpan + r*2;            % Module length [m]
            a = WheelSpan/2 + 0.0965;       % Distance from module center to front connection [m]
            b = WheelSpan/2 + 0.104;        % Distance from module center to rear connection [m]
            w_e = (0.2046-0.0843)/2;        % Distance between track centers [m]
            w_max = 0.2046;                 % Total width of the robot [m]
        
            % Maximum joint angle (calculated or fixed)
            % theta_max = rad2deg(atan((b-l/2)/(w_max/2))+asin((a-l/2)/sqrt((w_max/2)^2+(b-l/2)^2)))
            theta_max = 50;                 % Maximum joint angle [degrees]
        
            %% Trajectory definitions
            trajectory_names = ["Simple case", "Simple case smooth_w", "S-curve", "U-curve", "<3 shape"];
            current_trajectory = trajectory_names(iii);
            
            fprintf('Running simulation: %s with %d modules\n', current_trajectory, modules);
            
            N = 2000;                       % Base number of time steps
    
            %% Velocity profile generation
            switch current_trajectory
                case "Simple case"
                    % Constant velocity and angular velocity
                    v1_max = 0.5;               % Maximum linear velocity [m/s]
                    rc_max = 0.4;               % Minimum turning radius [m]
                    w1_max = v1_max/rc_max;     % Maximum angular velocity [rad/s]
                    v1 = v1_max * ones(1, N);
                    w1 = w1_max * ones(1, N);
                    
                case "Simple case smooth_w"
                    % Constant velocity with smooth angular velocity ramp
                    v1_max = 0.5;
                    rc_max = 0.4;
                    w1_max = v1_max/rc_max;
                    v1 = v1_max * ones(1, 3*N);
                    w1 = [zeros(1,N), linspace(0,w1_max,N), w1_max*ones(1,N)];
                    
                case "S-curve"
                    % S-shaped trajectory with alternating turns
                    v1_max = 0.5;
                    rc_max = 0.4;
                    w1_max = v1_max/rc_max;
                    v1 = v1_max * ones(1, 8*N);
                    w1 = [zeros(1,N), ...
                          linspace(0,w1_max,N), w1_max*ones(1,N/2), linspace(w1_max,0,N), ...
                          zeros(1,N), ...
                          linspace(0,-w1_max,N), -w1_max*ones(1,N/2), linspace(-w1_max,0,N), ...
                          zeros(1,N)];
                          
                case "U-curve"
                    % U-turn maneuver
                    v1_max = 0.5;
                    rc_max = 0.5;
                    w1_max = v1_max/rc_max;
                    v1 = v1_max * ones(1, 3*N);
                    w1 = [zeros(1,N), w1_max*ones(1,N), zeros(1,N)];
                    
                case "<3 shape"
                    % Heart-shaped trajectory
                    v1_max = 0.56;
                    rc_max = 0.1;
                    w1_max = v1_max/rc_max;
                    v1 = v1_max * ones(1, 37*N);
                    w1 = [zeros(1,N), ...
                          linspace(0,w1_max/3,N), linspace(w1_max/3,0,N), zeros(1,5*N), ...
                          linspace(0,w1_max/4,N), w1_max/4*ones(1,8*N), linspace(w1_max/4,0,N), ...
                          -w1_max*ones(1,N), ...
                          linspace(0,w1_max/4,N), w1_max/4*ones(1,8*N), linspace(w1_max/4,0,N), ...
                          zeros(1,5*N), linspace(0,w1_max/3,N), linspace(w1_max/3,0,N), zeros(1,N)];
                          
                otherwise
                    % Default case
                    v1_max = 0.56;
                    rc_max = 0.1;
                    w1_max = v1_max/rc_max;
                    v1 = v1_max * ones(1, N);
                    w1 = w1_max * ones(1, N);
            end
            
            %% Time vector setup
            ts = 10;                        % Simulation time [s]
            t = linspace(0, ts, length(v1));
            dt = t(2) - t(1);
            
            %% Velocity commands initialization
            V{1} = [v1; zeros(1,length(t)); zeros(1,length(t))];
            W{1} = [zeros(1,length(t)); zeros(1,length(t)); w1];
    
            for m = 2:modules
                V{m} = zeros(3, length(t));
                W{m} = zeros(3, length(t));
            end
            
            %% Initial conditions
            for m = 1:modules
                eta{m} = nan(3, length(t));     % Position and orientation [theta, x, y]
                etad{m} = nan(3, length(t));    % Velocities [theta_dot, x_dot, y_dot]
                
                % Initial pose: modules aligned in a line
                eta{m}(:,1) = [0; (m-1)*(-a-b); 0];  % [orientation, x_position, y_position]
                etad{m}(:,1) = [0; 0; 0];            % [angular_vel, x_vel, y_vel]
    
                omega{m} = nan(2, length(t));        % Wheel angular velocities [left; right]
            end
            
            %% Plot initial configuration (optional)
            % Uncomment the following lines to see the starting position
            % figure(100);
            % clf;
            % plotModuleHead([eta{1}(2:3,1)', 0]', eta{1}(1,1), l, w_max, a);
            % for m = 2:modules-1
            %     plotModuleMiddle([eta{m}(2:3,1)', 0]', eta{m}(1,1), l, w_max, a, b);
            % end
            % if modules > 1
            %     plotModuleTail([eta{modules}(2:3,1)', 0]', eta{modules}(1,1), l, w_max, b);
            % end
            % grid on;
            % axis equal;
            % title(sprintf('Initial Configuration - %s - %d Modules', current_trajectory, modules));
            % xlabel('X [m]');
            % ylabel('Y [m]');
            % pause(2);  % Show for 2 seconds

            %% Simulation loop
            for i = 1:length(t)-1
                % Calculate wheel velocities for the first module
                omega{1}(:,i) = [(V{1}(1,i) - W{1}(3,i)*w_e/2)/r;
                                 (V{1}(1,i) + W{1}(3,i)*w_e/2)/r];
            
                % Calculate velocities for following modules using kinematic constraints
                for m = 2:modules
                    th{m-1}(i) = eta{m-1}(1,i) - eta{m}(1,i);  % Joint angle
               
                    % Kinematic coupling equations
                    V{m}(1,i) = V{m-1}(1,i)*cos(th{m-1}(i)) + a*W{m-1}(3,i)*sin(th{m-1}(i));
                    W{m}(3,i) = (V{m-1}(1,i)*sin(th{m-1}(i)) - a*W{m-1}(3,i)*cos(th{m-1}(i)))/b;
                
                    % Calculate wheel velocities
                    omega{m}(:,i) = [(V{m}(1,i) - W{m}(3,i)*w_e/2)/r;
                                     (V{m}(1,i) + W{m}(3,i)*w_e/2)/r];
                end
                
                % Update positions and orientations for all modules
                for m = 1:modules
                    etad{m}(1,i) = W{m}(3,i);
                    eta{m}(1,i+1) = eta{m}(1,i) + dt*etad{m}(1,i);
            
                    % Transform body velocities to world frame
                    vs{m} = Rotz(eta{m}(1,i)) * V{m}(:,i);
                    etad{m}(2:3,i) = vs{m}(1:2);
                    eta{m}(2:3,i+1) = eta{m}(2:3,i) + dt*etad{m}(2:3,i);
                end
            end
            
            %% Plotting and visualization
            figure(1);
            clf;
            
            % Initialize subplots
            subplot(2,2,1);
            plot(eta{1}(2,:), eta{1}(3,:), '--k', "LineWidth", 0.75);
            grid on;
            axis equal;
            hold on;
            title(sprintf('%s - %d Modules', current_trajectory, modules));
            xlabel('X [m]');
            ylabel('Y [m]');
    
            if modules > 1
                subplot(2,2,2);
                plot([0, t(end)], [theta_max, theta_max], '--r', "LineWidth", 0.75);
                hold on;
                plot([0, t(end)], [-theta_max, -theta_max], '--r', "LineWidth", 0.75);
                plot(t, [0, rad2deg(th{1})], 'k', "LineWidth", 0.75);
                for m = 2:modules-1
                    plot(t, [0, rad2deg(th{m})], 'b', "LineWidth", 0.75);
                end
                grid on;
                xlabel("Time [s]");
                ylabel("Joint angles [deg]");
                legend("\theta_{max}", "\theta_{min}", "\theta_{1-2}", 'Location', 'southeast');
                title('Joint Angles vs Time');
    
                xline_handle_222 = xline(t(1), 'k', "LineWidth", 1.5);
            end
    
            subplot(2,2,3);
            plot(t, V{1}(1, :), '--k', "LineWidth", 0.75);
            hold on;
            plot(t, W{1}(3, :), 'k', "LineWidth", 0.75);
            for m = 2:modules
                plot(t, V{m}(1, :), '--r', "LineWidth", 0.75);
                plot(t, W{m}(3, :), 'r', "LineWidth", 0.75);
            end
            grid on;
            xlabel("Time [s]");
            ylabel("Velocities [m/s, rad/s]");
            legend("v_1","\omega_1", 'Location','southeast');
            title('Linear and Angular Velocities');
            xline_handle_223 = xline(t(1), 'k', "LineWidth", 1.5);
    
            subplot(2,2,4);
            plot(t, omega{1}(1, :) * 60 / (2 * pi), '--k', "LineWidth", 0.75);
            hold on;
            plot(t, omega{1}(2, :) * 60 / (2 * pi), 'k', "LineWidth", 0.75);
            for m = 2:modules
                plot(t, omega{m}(1, :) * 60 / (2 * pi), '--b', "LineWidth", 0.75);
                plot(t, omega{m}(2, :) * 60 / (2 * pi), 'b', "LineWidth", 0.75);
            end
            grid on;
            xlabel("Time [s]");
            ylabel("Wheel speeds [rpm]");
            legend("\omega_{1l}", "\omega_{1r}", 'Location', 'southeast');
            title('Wheel Angular Velocities');
            xline_handle_224 = xline(t(1), 'k', "LineWidth", 1.5);
            
            % Dynamic animation loop
            animation_steps = round(length(t)/50);
            for ii = 1:animation_steps:length(t)
                % Update time indicators on plots
                if modules > 1
                    set(xline_handle_222, 'Value', t(ii));
                end
                set(xline_handle_223, 'Value', t(ii));
                set(xline_handle_224, 'Value', t(ii));
    
                % Update legends on first iteration
                if ii == 1
                    if modules > 1
                        subplot(2,2,2)
                        legend("\theta_{max}", "\theta_{min}", "\theta_{1-2}", 'Location', 'southeast');
                    end
                    subplot(2,2,3)
                    legend("v_1","\omega_1", 'Location','southeast');
                    subplot(2,2,4)
                    legend("\omega_{1l}", "\omega_{1r}", 'Location', 'southeast');
                end
    
                % Update robot visualization in main plot
                subplot(2,2,1);
                cla;
                % Plot trajectory
                plot(eta{1}(2,:), eta{1}(3,:), '--k', "LineWidth", 0.75);
                hold on;
                
                % Plot robot modules
                plotModuleHead([eta{1}(2:3,ii)', 0]', eta{1}(1,ii), l, w_max, a);
                for m = 2:modules-1
                    plotModuleMiddle([eta{m}(2:3,ii)', 0]', eta{m}(1,ii), l, w_max, a, b);
                end
                if modules > 1
                    plotModuleTail([eta{end}(2:3,ii)', 0]', eta{end}(1,ii), l, w_max, b);
                end
                
                grid on;
                axis equal;
                title(sprintf('%s - %d Modules (t=%.1fs)', current_trajectory, modules, t(ii)));
                xlabel('X [m]');
                ylabel('Y [m]');
                drawnow;
            end
    
        end
        clf;
        clear;
    end
end

%% Utility Functions

function out = Rotz(th)
    % Rotation matrix around Z-axis
    % Input: th - rotation angle in radians
    % Output: 3x3 rotation matrix
    out = [cos(th) -sin(th) 0;
           sin(th)  cos(th) 0;
           0        0       1];
end

function [] = plotModuleHead(p, th, l, w_max, a)
    % Plot the head module of the robot
    % Inputs:
    %   p - position [x, y, z]
    %   th - orientation angle
    %   l - module length
    %   w_max - module width
    %   a - distance to connection point
    
    R = Rotz(th);
    p1 = p + R*[-l/2, -w_max/2, 0]';
    p2 = p + R*[+l/2, -w_max/2, 0]';
    p3 = p + R*[+l/2, +w_max/2, 0]';
    p4 = p + R*[-l/2, +w_max/2, 0]';
    pQ1 = p + R*[-l/2, 0, 0]';
    pQ = p + R*[-a, 0, 0]';

    plot([p1(1) p2(1) p3(1) p4(1) p1(1)], [p1(2) p2(2) p3(2) p4(2) p1(2)], 'k', 'LineWidth', 2)
    hold on
    plot([pQ1(1) pQ(1)], [pQ1(2) pQ(2)], 'k', 'LineWidth', 2)
    plot(pQ(1), pQ(2), 'ok', 'LineWidth', 2.5)
    quiver(p(1), p(2), 0.15*l*cos(th), 0.15*l*sin(th), "Color", 'red', 'LineWidth', 1.0)
    quiver(p(1), p(2), -0.15*l*sin(th), 0.15*l*cos(th), "Color", 'green', 'LineWidth', 1.0)
end

function [] = plotModuleMiddle(p, th, l, w_max, a, b)
    % Plot a middle module of the robot
    % Inputs:
    %   p - position [x, y, z]
    %   th - orientation angle
    %   l - module length
    %   w_max - module width
    %   a - distance to rear connection point
    %   b - distance to front connection point
    
    R = Rotz(th);
    p1 = p + R*[-l/2, -w_max/2, 0]';
    p2 = p + R*[+l/2, -w_max/2, 0]';
    p3 = p + R*[+l/2, +w_max/2, 0]';
    p4 = p + R*[-l/2, +w_max/2, 0]';
    pQ1 = p + R*[+l/2, 0, 0]';
    pQ = p + R*[+b, 0, 0]';
    pQ2 = p + R*[-l/2, 0, 0]';
    pQ_2 = p + R*[-a, 0, 0]';

    plot([p1(1) p2(1) p3(1) p4(1) p1(1)], [p1(2) p2(2) p3(2) p4(2) p1(2)], 'r', 'LineWidth', 2)
    hold on
    plot([pQ1(1) pQ(1)], [pQ1(2) pQ(2)], 'r', 'LineWidth', 2)
    plot(pQ(1), pQ(2), 'ok', 'LineWidth', 2.5)
    plot([pQ2(1) pQ_2(1)], [pQ2(2) pQ_2(2)], 'k', 'LineWidth', 2)
    plot(pQ_2(1), pQ_2(2), 'ok', 'LineWidth', 2.5)
    quiver(p(1), p(2), 0.15*l*cos(th), 0.15*l*sin(th), "Color", 'red', 'LineWidth', 1.0)
    quiver(p(1), p(2), -0.15*l*sin(th), 0.15*l*cos(th), "Color", 'green', 'LineWidth', 1.0)
end

function [] = plotModuleTail(p, th, l, w_max, b)
    % Plot the tail module of the robot
    % Inputs:
    %   p - position [x, y, z]
    %   th - orientation angle
    %   l - module length
    %   w_max - module width
    %   b - distance to connection point
    
    R = Rotz(th);
    p1 = p + R*[-l/2, -w_max/2, 0]';
    p2 = p + R*[+l/2, -w_max/2, 0]';
    p3 = p + R*[+l/2, +w_max/2, 0]';
    p4 = p + R*[-l/2, +w_max/2, 0]';
    pQ1 = p + R*[+l/2, 0, 0]';
    pQ = p + R*[+b, 0, 0]';

    plot([p1(1) p2(1) p3(1) p4(1) p1(1)], [p1(2) p2(2) p3(2) p4(2) p1(2)], 'b', 'LineWidth', 2)
    hold on
    plot([pQ1(1) pQ(1)], [pQ1(2) pQ(2)], 'b', 'LineWidth', 2)
    plot(pQ(1), pQ(2), 'ok', 'LineWidth', 2.5)
    quiver(p(1), p(2), 0.15*l*cos(th), 0.15*l*sin(th), "Color", 'red', 'LineWidth', 1.0)
    quiver(p(1), p(2), -0.15*l*sin(th), 0.15*l*cos(th), "Color", 'green', 'LineWidth', 1.0)
end