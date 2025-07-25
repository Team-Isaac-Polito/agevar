clear 
close all
clc

%% RECURSIVE DIFFERENTIAL DRIVE MODULE CHAIN
% Scalable number of modules: each module follows the behavior of the previous one
% Input: x velocity and z angular velocity for first module only
% Recursive behavior: module i follows module i-1 kinematics

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

modules=3;

%% Simulation parameters
N = 1000;                       % Number of time steps
ts = 10;                        % Total simulation time [s]
t = linspace(0, ts, N);
dt = t(2) - t(1);

v_forward = 0.2;                % Forward velocity [m/s]
w_rotation = 0.5;               % Angular velocity for rotation [rad/s]

% Create velocity profiles
v_x = zeros(1, N);
w_z = zeros(1, N);

% Time indices for phases
phase1_end = round(0.3 * N);
phase2_end = round(0.6 * N);
phase3_end = N;

% Phase 1: Straight motion
v_x(1:phase1_end) = v_forward;
w_z(1:phase1_end) = 0;

% Phase 2: Rotation around rear joint + forward motion
v_x(phase1_end+1:phase2_end) = v_forward * 0.5;  % Reduced linear velocity during pivot
w_z(phase1_end+1:phase2_end) = w_rotation;

% Phase 3: Straight motion again
v_x(phase2_end+1:phase3_end) = v_forward;
w_z(phase2_end+1:phase3_end) = 0;

fprintf('%d-module iterative coupling: straight -> first pivots+moves (others calculated iteratively) -> all straight\n', modules);

% Add debug output every 100 steps during pivot
debug_output = true;

%% Initialize arrays

for m = 1:modules
    eta{m} = nan(3, N);     % Position and orientation [x, y, theta]
    yaw{m} = nan(2, N);     % Joint position [x, y] for each module
    
    % Initial pose: modules aligned in a line
    eta{m}(:,1) = [(m-1)*(-a-b); 0; 0];  % [x_position, y_position, orientation]
    
    % Initial joint position (rear joint of each module)
    yaw{m}(:,1) = [(m-1)*(-a-b) - b; 0];  % [x_joint, y_joint]
end

% Print initial positions dynamically
fprintf('Initial positions: ');
for m = 1:modules
    fprintf('Module %d: [%.2f, %.2f]', m, eta{m}(1,1), eta{m}(2,1));
    if m < modules
        fprintf(', ');
    end
end
fprintf('\n');

%% Simulation loop
for i = 1:N-1
    % Store velocities for each module [v_x, w_z]
    module_vel = zeros(2, modules);
    
    % FIRST MODULE: Uses direct input commands
    module_vel(1,1) = v_x(i);  % Linear velocity
    module_vel(2,1) = w_z(i);  % Angular velocity
    
    % Update first module
    if w_z(i) == 0
        % STRAIGHT MOTION: Use same integration method as other modules for consistency
        eta{1}(3,i+1) = eta{1}(3,i) + dt * w_z(i);  % Update orientation first
        
        % Transform body velocity to world frame
        v_world_x = v_x(i) * cos(eta{1}(3,i));
        v_world_y = v_x(i) * sin(eta{1}(3,i));
        
        eta{1}(1,i+1) = eta{1}(1,i) + dt * v_world_x;
        eta{1}(2,i+1) = eta{1}(2,i) + dt * v_world_y;
        yaw{1}(1,i+1) = eta{1}(1,i+1) - b * cos(eta{1}(3,i+1));
        yaw{1}(2,i+1) = eta{1}(2,i+1) - b * sin(eta{1}(3,i+1));
    else
        % ROTATION AROUND REAR JOINT + FORWARD MOTION OF THE JOINT
        % First, move the rear joint forward
        yaw{1}(1,i+1) = yaw{1}(1,i) + dt * v_x(i);  % Joint moves forward
        yaw{1}(2,i+1) = yaw{1}(2,i);  % Joint stays at same Y
        
        % Then, rotate the module around the moved joint
        eta{1}(3,i+1) = eta{1}(3,i) + dt * w_z(i);
        eta{1}(1,i+1) = yaw{1}(1,i+1) + b * cos(eta{1}(3,i+1));
        eta{1}(2,i+1) = yaw{1}(2,i+1) + b * sin(eta{1}(3,i+1));
    end
    
    % FOLLOWING MODULES: Each follows the previous one
    for m = 2:modules
        prev_v = module_vel(1, m-1);  % Previous module linear velocity
        prev_w = module_vel(2, m-1);  % Previous module angular velocity
        
        if w_z(i) == 0  % During straight motion phases, all modules follow
            % Calculate kinematic coupling with previous module using original equations
            th_joint = eta{m-1}(3,i) - eta{m}(3,i);  % Joint angle (orientation difference)
            
            % Original kinematic coupling equations:
            % V{m}(1,i) = V{m-1}(1,i)*cos(th{m-1}(i)) + a*W{m-1}(3,i)*sin(th{m-1}(i));
            % W{m}(3,i) = (V{m-1}(1,i)*sin(th{m-1}(i)) - a*W{m-1}(3,i)*cos(th{m-1}(i)))/b;
            v_follow = prev_v * cos(th_joint) + a * prev_w * sin(th_joint);
            w_follow = (prev_v * sin(th_joint) - a * prev_w * cos(th_joint)) / b;
            
            % Store velocities for this module
            module_vel(1,m) = v_follow;
            module_vel(2,m) = w_follow;
            
            % Update module state using the same integration method as original
            eta{m}(3,i+1) = eta{m}(3,i) + dt * w_follow;  % Update orientation first
            
            % Transform body velocities to world frame (like original vs = Rotz * V)
            v_world_x = v_follow * cos(eta{m}(3,i));
            v_world_y = v_follow * sin(eta{m}(3,i));
            
            eta{m}(1,i+1) = eta{m}(1,i) + dt * v_world_x;
            eta{m}(2,i+1) = eta{m}(2,i) + dt * v_world_y;
            yaw{m}(1,i+1) = eta{m}(1,i+1) - b * cos(eta{m}(3,i+1));
            yaw{m}(2,i+1) = eta{m}(2,i+1) - b * sin(eta{m}(3,i+1));
        else
            % During pivot phases: iterative calculation based on previous module's state
            % Consider both linear velocity and rotational contribution of previous module
            
            if m == 2  % Second module follows the rear joint of first module
                % For the second module, we need to follow the rear joint velocity of the first module
                % The rear joint moves due to:
                % 1. Direct forward movement: v_x
                % 2. Rotational movement due to module rotation around the joint
                
                % Calculate the velocity of the rear joint of the first module
                % Joint moves forward with v_x, but also moves due to rotation
                v_joint_x = v_x(i);  % Direct forward movement of joint
                v_joint_y = 0;       % Joint doesn't move in Y during this simple pivot
                
                % But we also need to consider that as the module rotates around the joint,
                % the joint effectively moves in a way that creates the proper coupling
                % For proper coupling, the second module should match the velocity component
                % that maintains the connection constraint
                
                target_v_x_global = v_joint_x;
                target_v_y_global = v_joint_y;
            else
                % For modules beyond the second, use front connection point velocity
                % Previous module's center velocity in global frame
                prev_v_center_x = prev_v * cos(eta{m-1}(3,i));
                prev_v_center_y = prev_v * sin(eta{m-1}(3,i));
                
                % Add rotational contribution: velocity of front connection point of previous module
                % The front connection point is at distance 'a' from the center
                % Rotational velocity contribution: v_rot = w × r (cross product)
                prev_w_rot_contrib_x = -prev_w * a * sin(eta{m-1}(3,i));  % -w*r*sin(theta)
                prev_w_rot_contrib_y = prev_w * a * cos(eta{m-1}(3,i));   % w*r*cos(theta)
                
                % Total velocity of the front connection point of previous module
                target_v_x_global = prev_v_center_x + prev_w_rot_contrib_x;
                target_v_y_global = prev_v_center_y + prev_w_rot_contrib_y;
            end
            
            % Calculate required velocity for this module to achieve target X-component
            if abs(cos(eta{m}(3,i))) > 1e-6  % Avoid division by zero
                v_required = target_v_x_global / cos(eta{m}(3,i));
            else
                v_required = 0;  % Module is perpendicular to X-axis
            end
            
            % Limit velocity to reasonable bounds
            v_max = v_forward * 2.0;  % Maximum allowed velocity
            v_required = max(-v_max, min(v_max, v_required));
            
            % Store velocities for this module (straight motion only during pivot)
            module_vel(1,m) = v_required;
            module_vel(2,m) = 0;  % No rotation during pivot phase
            
            % Debug output every 100 steps
            if debug_output && mod(i,100) == 0 && w_z(i) ~= 0
                fprintf('Step %d: M%d angle=%.1f°, prev_v=%.3f, prev_w=%.3f, front_vx=%.3f, req_v=%.3f\n', ...
                    i, m, rad2deg(eta{m}(3,i)), prev_v, prev_w, target_v_x_global, v_required);
            end
            
            % Update module state (straight motion)
            eta{m}(3,i+1) = eta{m}(3,i);  % No rotation
            eta{m}(1,i+1) = eta{m}(1,i) + dt * v_required * cos(eta{m}(3,i));
            eta{m}(2,i+1) = eta{m}(2,i) + dt * v_required * sin(eta{m}(3,i));
            yaw{m}(1,i+1) = eta{m}(1,i+1) - b * cos(eta{m}(3,i+1));
            yaw{m}(2,i+1) = eta{m}(2,i+1) - b * sin(eta{m}(3,i+1));
        end
    end
end

% Animation
colors = {'b', 'g', 'r', 'm', 'c', 'k', 'y'};
figure(2);
clf;
for i = 1:20:N
    clf;
    
    % Plot trajectory up to current time
    hold on;
    for m = 1:modules
        color = colors{mod(m-1, length(colors)) + 1};
        plot(eta{m}(1,1:i), eta{m}(2,1:i), [color '-'], 'LineWidth', 1.5);
        plot(yaw{m}(1,1:i), yaw{m}(2,1:i), [color '--'], 'LineWidth', 1);
    end
    
    % Plot current modules
    for m = 1:modules
        color = colors{mod(m-1, length(colors)) + 1};
        plotSimpleModule(eta{m}(:,i), l, w_e, color);
        
        % Plot rear joint (yaw)
        plot(yaw{m}(1,i), yaw{m}(2,i), [color 'o'], 'MarkerSize', 8, 'LineWidth', 2);
        
        % Plot front connection point (at distance 'a' from module center)
        front_x = eta{m}(1,i) + a * cos(eta{m}(3,i));
        front_y = eta{m}(2,i) + a * sin(eta{m}(3,i));
        plot(front_x, front_y, [color 's'], 'MarkerSize', 6, 'LineWidth', 2);
        
        % Draw line from module center to front connection (distance 'a')
        plot([eta{m}(1,i), front_x], [eta{m}(2,i), front_y], [color ':'], 'LineWidth', 1.5);
        
        % Draw line from module center to rear joint (distance 'b')  
        plot([eta{m}(1,i), yaw{m}(1,i)], [eta{m}(2,i), yaw{m}(2,i)], [color '-'], 'LineWidth', 2);
    end
    
    % No inter-module connection lines - just showing individual module geometry
    
    grid on;
    axis equal;
    title(sprintf('t = %.1f s, %d-Module System', t(i), modules));
    xlabel('X [m]');
    ylabel('Y [m]');
    
    % Add legend for visualization elements
    legend_elements = {};
    if modules > 0
        legend_elements{end+1} = 'Module body';
        legend_elements{end+1} = 'Rear joint (o)';
        legend_elements{end+1} = 'Front connection (□)';
        legend_elements{end+1} = 'Distance a (:)';
        legend_elements{end+1} = 'Distance b (-)';
    end
    
    drawnow;
    pause(0.1);
end

%% Helper Functions
function plotSimpleModule(pose, l, w, color)
    % Plot a simple rectangular module
    % pose = [x; y; theta]
    % color = 'b', 'g', 'r', etc.
    if nargin < 4
        color = 'k';  % Default black
    end
    
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    
    % Module corners
    corners = [-l/2, -w/2; l/2, -w/2; l/2, w/2; -l/2, w/2; -l/2, -w/2]';
    
    % Rotate corners
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotated_corners = R * corners;
    
    % Translate to current position
    rotated_corners(1,:) = rotated_corners(1,:) + x;
    rotated_corners(2,:) = rotated_corners(2,:) + y;
    
    % Plot module
    plot(rotated_corners(1,:), rotated_corners(2,:), color, 'LineWidth', 2);
    
    % Plot orientation arrow
    arrow_length = l/3;
    arrow_end_x = x + arrow_length * cos(theta);
    arrow_end_y = y + arrow_length * sin(theta);
    quiver(x, y, arrow_end_x - x, arrow_end_y - y, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.3);
end

function phase_name = getPhase(i, phase1_end, phase2_end)
    if i <= phase1_end
        phase_name = 'Straight';
    elseif i <= phase2_end
        phase_name = 'Rotating around joint';
    else
        phase_name = 'Straight';
    end
end