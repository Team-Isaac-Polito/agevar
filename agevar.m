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
N = 2000;                       % Number of time steps
ts = 10;                        % Total simulation time [s]
t = linspace(0, ts, N);
dt = t(2) - t(1);

v_forward = 0.2;                % Forward velocity [m/s]
w_rotation = 0.5;               % Angular velocity for rotation [rad/s]

% Create velocity profiles
v_x = zeros(1, N);
w_z = zeros(1, N);

% Time indices for phases
phase1_end = round(0.2 * N);    % Straight motion
phase2_end = round(0.4 * N);    % First module pivot
phase3_end = round(0.6 * N);    % Second module pivot
phase4_end = round(0.8 * N);    % Third module pivot (new)
phase5_end = N;                 % All straight again

% Phase 1: All straight motion
v_x(1:phase1_end) = v_forward;
w_z(1:phase1_end) = 0;

% Phase 2: First module pivots
v_x(phase1_end+1:phase2_end) = v_forward * 0.5;  % Reduced linear velocity during pivot
w_z(phase1_end+1:phase2_end) = w_rotation;

% Phase 3: Second module pivots (first goes straight, third follows)
v_x(phase2_end+1:phase3_end) = v_forward;
w_z(phase2_end+1:phase3_end) = 0;  % First module stops rotating

% Phase 4: Third module pivots (first and second go straight)
v_x(phase3_end+1:phase4_end) = v_forward;
w_z(phase3_end+1:phase4_end) = 0;  % First module stops rotating

% Phase 5: All straight motion again
v_x(phase4_end+1:phase5_end) = v_forward;
w_z(phase4_end+1:phase5_end) = 0;

fprintf('%d-module sequential pivot: straight -> M1 pivots -> M2 pivots -> M3 pivots -> all straight\n', modules);

% Add debug output every 100 steps during pivot
debug_output = true;

% Define which module is pivoting in each phase
function pivoting_module = getPivotingModule(i, phase1_end, phase2_end, phase3_end, phase4_end)
    if i <= phase1_end
        pivoting_module = 0;  % No pivot
    elseif i <= phase2_end
        pivoting_module = 1;  % First module pivots
    elseif i <= phase3_end
        pivoting_module = 2;  % Second module pivots
    elseif i <= phase4_end
        pivoting_module = 3;  % Third module pivots
    else
        pivoting_module = 0;  % No pivot
    end
end

%% Initialize arrays

for m = 1:modules
    eta{m} = nan(3, N);     % Position and orientation [x, y, theta]
    yaw{m} = nan(2, N);     % Joint position [x, y] for each module (rear joint)
    front_joint{m} = nan(2, N);  % Front joint position [x, y] for each module
    
    % Initial pose: modules aligned in a line
    eta{m}(:,1) = [(m-1)*(-a-b); 0; 0];  % [x_position, y_position, orientation]
    
    % Initial joint position (rear joint of each module)
    yaw{m}(:,1) = [(m-1)*(-a-b) - b; 0];  % [x_joint, y_joint]
    
    % Initial front joint position
    front_joint{m}(:,1) = [(m-1)*(-a-b) + a; 0];  % [x_front_joint, y_front_joint]
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
    
    % Determine which module is pivoting in this phase
    pivoting_module = getPivotingModule(i, phase1_end, phase2_end, phase3_end, phase4_end);
    
    % FIRST MODULE: Uses direct input when pivoting, calculated when following
    if pivoting_module == 1
        % First module is pivoting - use direct commands
        module_vel(1,1) = v_x(i);  % Linear velocity
        module_vel(2,1) = w_z(i);  % Angular velocity
    else
        % First module is not pivoting - will be calculated in following modules loop
        module_vel(1,1) = v_forward;  % Default forward velocity when not pivoting
        module_vel(2,1) = 0;          % No rotation when not pivoting
    end
    
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
        
        % Determine which module is pivoting in this phase
        pivoting_module = getPivotingModule(i, phase1_end, phase2_end, phase3_end, phase4_end);
        
        if m == 2 && pivoting_module == 2
            % Second module during its own pivot: use constraint-based positioning
            % The second module must satisfy both connection constraints:
            % 1. Its front connects to first module's rear
            % 2. Its rear connects to third module's front
            
            % Get positions of connection points
            % First module rear joint (where M2 front should be)
            P1_rear_x = yaw{1}(1,i+1);  % Already calculated from M1 update
            P1_rear_y = yaw{1}(2,i+1);
            
            % Third module front connection (where M2 rear should be)
            P3_front_x = eta{3}(1,i) + a * cos(eta{3}(3,i));
            P3_front_y = eta{3}(2,i) + a * sin(eta{3}(3,i));
            
            % Calculate M2 orientation from constraint geometry
            constraint_vec_x = P1_rear_x - P3_front_x;
            constraint_vec_y = P1_rear_y - P3_front_y;
            constraint_length = sqrt(constraint_vec_x^2 + constraint_vec_y^2);
            
            if constraint_length > 1e-6
                % Calculate M2 orientation (direction from rear to front)
                new_theta_2 = atan2(constraint_vec_y, constraint_vec_x);
                
                % Calculate M2 center position
                new_center_x = P3_front_x + b * cos(new_theta_2);
                new_center_y = P3_front_y + b * sin(new_theta_2);
                
                % Calculate velocities from position change
                v_center_x = (new_center_x - eta{2}(1,i)) / dt;
                v_center_y = (new_center_y - eta{2}(2,i)) / dt;
                w_angular = (new_theta_2 - eta{2}(3,i)) / dt;
                
                % Convert to body frame velocity
                cos_theta = cos(eta{2}(3,i));
                sin_theta = sin(eta{2}(3,i));
                v_body_x = v_center_x * cos_theta + v_center_y * sin_theta;
                
                module_vel(1,2) = v_body_x;    % Linear velocity in body frame
                module_vel(2,2) = w_angular;   % Angular velocity
                
                % Update module state
                eta{2}(3,i+1) = new_theta_2;
                eta{2}(1,i+1) = new_center_x;
                eta{2}(2,i+1) = new_center_y;
                yaw{2}(1,i+1) = eta{2}(1,i+1) - b * cos(eta{2}(3,i+1));
                yaw{2}(2,i+1) = eta{2}(2,i+1) - b * sin(eta{2}(3,i+1));
                
                % Debug constraint satisfaction
                if debug_output && mod(i,50) == 0
                    front_x = eta{2}(1,i+1) + a * cos(eta{2}(3,i+1));
                    front_y = eta{2}(2,i+1) + a * sin(eta{2}(3,i+1));
                    front_error = sqrt((front_x - P1_rear_x)^2 + (front_y - P1_rear_y)^2);
                    
                    rear_x = eta{2}(1,i+1) - b * cos(eta{2}(3,i+1));
                    rear_y = eta{2}(2,i+1) - b * sin(eta{2}(3,i+1));
                    rear_error = sqrt((rear_x - P3_front_x)^2 + (rear_y - P3_front_y)^2);
                    
                    fprintf('Step %d: M2 constraint errors: front=%.6f, rear=%.6f, v_body=%.3f, w=%.3f\n', ...
                        i, front_error, rear_error, v_body_x, w_angular);
                end
            else
                % Constraint is already satisfied, minimal movement
                module_vel(1,2) = 0;
                module_vel(2,2) = 0;
                eta{2}(:,i+1) = eta{2}(:,i);
                yaw{2}(:,i+1) = yaw{2}(:,i);
            end
        elseif m == 3 && pivoting_module == 3 && modules >= 3
            % Third module during its own pivot: use constraint-based positioning
            % The third module only needs to satisfy one connection constraint:
            % Its front connects to second module's rear
            
            % Get positions of connection points
            % Second module rear joint (where M3 front should be)
            P2_rear_x = yaw{2}(1,i+1);  % Already calculated from M2 update
            P2_rear_y = yaw{2}(2,i+1);
            
            % For the last module, we only need to maintain front connection
            % Position the module center so that front connection aligns with M2 rear
            
            % Calculate M3 orientation (from center toward front connection point)
            % Use current orientation as reference and adjust minimally
            constraint_vec_x = P2_rear_x - eta{3}(1,i);
            constraint_vec_y = P2_rear_y - eta{3}(2,i);
            
            % Calculate desired orientation based on front connection constraint
            new_theta_3 = atan2(constraint_vec_y, constraint_vec_x);
            
            % Calculate M3 center position
            new_center_x = P2_rear_x - a * cos(new_theta_3);
            new_center_y = P2_rear_y - a * sin(new_theta_3);
            
            % Calculate velocities from position change
            v_center_x = (new_center_x - eta{3}(1,i)) / dt;
            v_center_y = (new_center_y - eta{3}(2,i)) / dt;
            w_angular = (new_theta_3 - eta{3}(3,i)) / dt;
            
            % Convert to body frame velocity
            cos_theta = cos(eta{3}(3,i));
            sin_theta = sin(eta{3}(3,i));
            v_body_x = v_center_x * cos_theta + v_center_y * sin_theta;
            
            module_vel(1,3) = v_body_x;    % Linear velocity in body frame
            module_vel(2,3) = w_angular;   % Angular velocity
            
            % Update module state
            eta{3}(3,i+1) = new_theta_3;
            eta{3}(1,i+1) = new_center_x;
            eta{3}(2,i+1) = new_center_y;
            yaw{3}(1,i+1) = eta{3}(1,i+1) - b * cos(eta{3}(3,i+1));
            yaw{3}(2,i+1) = eta{3}(2,i+1) - b * sin(eta{3}(3,i+1));
            
            % Debug constraint satisfaction
            if debug_output && mod(i,50) == 0
                front_x = eta{3}(1,i+1) + a * cos(eta{3}(3,i+1));
                front_y = eta{3}(2,i+1) + a * sin(eta{3}(3,i+1));
                front_error = sqrt((front_x - P2_rear_x)^2 + (front_y - P2_rear_y)^2);
                
                fprintf('Step %d: M3 constraint error: front=%.6f, v_body=%.3f, w=%.3f\n', ...
                    i, front_error, v_body_x, w_angular);
            end
        elseif m == 3 && pivoting_module == 2
            % Third module during second module pivot: move straight
            module_vel(1,m) = v_forward;
            module_vel(2,m) = 0;
            
            eta{m}(3,i+1) = eta{m}(3,i);  % No rotation
            v_world_x = v_forward * cos(eta{m}(3,i));
            v_world_y = v_forward * sin(eta{m}(3,i));
            eta{m}(1,i+1) = eta{m}(1,i) + dt * v_world_x;
            eta{m}(2,i+1) = eta{m}(2,i) + dt * v_world_y;
            yaw{m}(1,i+1) = eta{m}(1,i+1) - b * cos(eta{m}(3,i+1));
            yaw{m}(2,i+1) = eta{m}(2,i+1) - b * sin(eta{m}(3,i+1));
        elseif w_z(i) == 0  % During straight motion phases, all modules follow
            % Calculate kinematic coupling with previous module using original equations
            th_joint = eta{m-1}(3,i) - eta{m}(3,i);  % Joint angle (orientation difference)
            
            % Original kinematic coupling equations:
            v_follow = prev_v * cos(th_joint) + a * prev_w * sin(th_joint);
            w_follow = (prev_v * sin(th_joint) - a * prev_w * cos(th_joint)) / b;
            
            % Store velocities for this module
            module_vel(1,m) = v_follow;
            module_vel(2,m) = w_follow;
            
            % Update module state using the same integration method as original
            eta{m}(3,i+1) = eta{m}(3,i) + dt * w_follow;  % Update orientation first
            
            % Transform body velocities to world frame
            v_world_x = v_follow * cos(eta{m}(3,i));
            v_world_y = v_follow * sin(eta{m}(3,i));
            
            eta{m}(1,i+1) = eta{m}(1,i) + dt * v_world_x;
            eta{m}(2,i+1) = eta{m}(2,i) + dt * v_world_y;
            yaw{m}(1,i+1) = eta{m}(1,i+1) - b * cos(eta{m}(3,i+1));
            yaw{m}(2,i+1) = eta{m}(2,i+1) - b * sin(eta{m}(3,i+1));
        else
            % During pivot phases: iterative calculation based on previous module's state
            
            if m == 2  % Second module follows the rear joint of first module
                % Calculate the velocity of the rear joint of the first module
                v_joint_x = v_x(i);  % Direct forward movement of joint
                v_joint_y = 0;       % Joint doesn't move in Y during this simple pivot
                
                target_v_x_global = v_joint_x;
                target_v_y_global = v_joint_y;
            else
                % For modules beyond the second, use front connection point velocity
                prev_v_center_x = prev_v * cos(eta{m-1}(3,i));
                prev_v_center_y = prev_v * sin(eta{m-1}(3,i));
                
                prev_w_rot_contrib_x = -prev_w * a * sin(eta{m-1}(3,i));
                prev_w_rot_contrib_y = prev_w * a * cos(eta{m-1}(3,i));
                
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
    
    % Update all front joints after state updates
    for m = 1:modules
        front_joint{m}(1,i+1) = eta{m}(1,i+1) + a * cos(eta{m}(3,i+1));
        front_joint{m}(2,i+1) = eta{m}(2,i+1) + a * sin(eta{m}(3,i+1));
    end
end

% Debug output
if debug_output && mod(i,100) == 0
    pivoting_module = getPivotingModule(i, phase1_end, phase2_end, phase3_end, phase4_end);
    if pivoting_module > 0
        fprintf('Step %d: M%d pivoting, velocities: ', i, pivoting_module);
        for m = 1:modules
            fprintf('M%d=%.3f ', m, module_vel(1,m));
        end
        fprintf('\n');
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
        % Plot front joint trajectory
        plot(front_joint{m}(1,1:i), front_joint{m}(2,1:i), [color '-.'], 'LineWidth', 1, 'Color', color);
    end
    
    % Plot current modules
    for m = 1:modules
        color = colors{mod(m-1, length(colors)) + 1};
        plotSimpleModule(eta{m}(:,i), l, w_e, color);
        
        % Plot rear joint (yaw)
        plot(yaw{m}(1,i), yaw{m}(2,i), [color 'o'], 'MarkerSize', 8, 'LineWidth', 2);
        
        % Plot front joint
        plot(front_joint{m}(1,i), front_joint{m}(2,i), [color 'd'], 'MarkerSize', 8, 'LineWidth', 2);
        
        % Plot front connection point (at distance 'a' from module center)
        front_x = eta{m}(1,i) + a * cos(eta{m}(3,i));
        front_y = eta{m}(2,i) + a * sin(eta{m}(3,i));
        plot(front_x, front_y, [color 's'], 'MarkerSize', 6, 'LineWidth', 2);
        
        % Draw line from module center to front connection (distance 'a')
        plot([eta{m}(1,i), front_x], [eta{m}(2,i), front_y], [color ':'], 'LineWidth', 1.5);
        
        % Draw line from module center to rear joint (distance 'b')  
        plot([eta{m}(1,i), yaw{m}(1,i)], [eta{m}(2,i), yaw{m}(2,i)], [color '-'], 'LineWidth', 2);
    end
    
    grid on;
    axis equal;
    
    % Determine current phase for title
    current_phase = getPivotingModule(i, phase1_end, phase2_end, phase3_end, phase4_end);
    phase_text = '';
    if current_phase == 0
        phase_text = 'All Straight';
    elseif current_phase == 1
        phase_text = 'M1 Pivoting';
    elseif current_phase == 2
        phase_text = 'M2 Pivoting';
    elseif current_phase == 3
        phase_text = 'M3 Pivoting';
    end
    
    title(sprintf('t = %.1f s, %d-Module Sequential Pivot - %s', t(i), modules, phase_text));
    xlabel('X [m]');
    ylabel('Y [m]');
    
    % Add legend for visualization elements
    legend_elements = {};
    if modules > 0
        legend_elements{end+1} = 'Module center trajectory (-)';
        legend_elements{end+1} = 'Rear joint trajectory (--)';
        legend_elements{end+1} = 'Front joint trajectory (-.)';
        legend_elements{end+1} = 'Rear joint (o)';
        legend_elements{end+1} = 'Front joint (◇)';
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