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

modules=4;

%% Simulation parameters
N = 1000;                       % Number of time steps
ts = 10;                        % Total simulation time [s]
t = linspace(0, ts, N);
dt = t(2) - t(1);

%% Input commands: [straight, rotate, straight]
% Phase 1: Move straight (0-3s)
% Phase 2: Rotate around rear joint (3-7s) 
% Phase 3: Move straight again (7-10s)

v_forward = 0.2;                % Forward velocity [m/s]
w_rotation = 0.5;               % Angular velocity for rotation [rad/s]

% Create velocity profiles
v_x = zeros(1, N);
w_z = zeros(1, N);

% Time indices for phases
phase1_end = round(0.1 * N);
phase2_end = round(0.4 * N);
phase3_end = round(0.5 * N); 
phase4_end = round(0.8 * N);
phase5_end = N;

% Phase 1: Straight motion
v_x(1:phase1_end) = v_forward;
w_z(1:phase1_end) = 0;

% Phase 2: Rotation around rear joint
v_x(phase1_end+1:phase2_end) = 0;
w_z(phase1_end+1:phase2_end) = w_rotation;

% Phase 3: Straight motion again
v_x(phase2_end+1:phase3_end) = v_forward;
w_z(phase2_end+1:phase3_end) = 0;

% Phase 4: Rotation around rear joint
v_x(phase3_end+1:phase4_end) = 0;
w_z(phase3_end+1:phase4_end) = w_rotation;

% Phase 5: Straight motion again
v_x(phase4_end+1:phase5_end) = v_forward;
w_z(phase4_end+1:phase5_end) = 0;

fprintf('%d-module joint rotation test: all straight -> first rotates (others stay) -> all straight\n', modules);

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
        % STRAIGHT MOTION: Normal differential drive kinematics
        eta{1}(3,i+1) = eta{1}(3,i) + dt * w_z(i);
        eta{1}(1,i+1) = eta{1}(1,i) + dt * v_x(i) * cos(eta{1}(3,i));
        eta{1}(2,i+1) = eta{1}(2,i) + dt * v_x(i) * sin(eta{1}(3,i));
        yaw{1}(1,i+1) = eta{1}(1,i+1) - b * cos(eta{1}(3,i+1));
        yaw{1}(2,i+1) = eta{1}(2,i+1) - b * sin(eta{1}(3,i+1));
    else
        % ROTATION AROUND REAR JOINT
        yaw{1}(:,i+1) = yaw{1}(:,i);
        eta{1}(3,i+1) = eta{1}(3,i) + dt * w_z(i);
        eta{1}(1,i+1) = yaw{1}(1,i) + b * cos(eta{1}(3,i+1));
        eta{1}(2,i+1) = yaw{1}(2,i) + b * sin(eta{1}(3,i+1));
    end
    
    % FOLLOWING MODULES: Each follows the previous one
    for m = 2:modules
        prev_v = module_vel(1, m-1);  % Previous module linear velocity
        prev_w = module_vel(2, m-1);  % Previous module angular velocity
        
        if w_z(i) == 0  % During straight motion phases, all modules follow
            % Calculate kinematic coupling with previous module
            th_joint = eta{m-1}(3,i) - eta{m}(3,i);  % Relative angle
            
            % Kinematic coupling equations
            v_follow = prev_v * cos(th_joint) + a * prev_w * sin(th_joint);
            w_follow = (prev_v * sin(th_joint) - a * prev_w * cos(th_joint)) / b;
            
            % Store velocities for this module
            module_vel(1,m) = v_follow;
            module_vel(2,m) = w_follow;
            
            % Update module state
            eta{m}(3,i+1) = eta{m}(3,i) + dt * w_follow;
            eta{m}(1,i+1) = eta{m}(1,i) + dt * v_follow * cos(eta{m}(3,i));
            eta{m}(2,i+1) = eta{m}(2,i) + dt * v_follow * sin(eta{m}(3,i));
            yaw{m}(1,i+1) = eta{m}(1,i+1) - b * cos(eta{m}(3,i+1));
            yaw{m}(2,i+1) = eta{m}(2,i+1) - b * sin(eta{m}(3,i+1));
        else
            % Stay still during rotation phases of first module
            module_vel(1,m) = 0;
            module_vel(2,m) = 0;
            eta{m}(:,i+1) = eta{m}(:,i);
            yaw{m}(:,i+1) = yaw{m}(:,i);
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
        plot(yaw{m}(1,i), yaw{m}(2,i), [color 'o'], 'MarkerSize', 8, 'LineWidth', 2);
    end
    
    % Draw connections between modules (through joints)
    for m = 1:modules-1
        plot([eta{m}(1,i), yaw{m}(1,i)], [eta{m}(2,i), yaw{m}(2,i)], 'k-', 'LineWidth', 2);
        plot([yaw{m}(1,i), eta{m+1}(1,i)], [yaw{m}(2,i), eta{m+1}(2,i)], 'k-', 'LineWidth', 2);
    end
    % Last module connection to its own joint
    if modules > 0
        plot([eta{modules}(1,i), yaw{modules}(1,i)], [eta{modules}(2,i), yaw{modules}(2,i)], 'k-', 'LineWidth', 2);
    end
    
    grid on;
    axis equal;
    title(sprintf('t = %.1f s, %d-Module System', t(i), modules));
    xlabel('X [m]');
    ylabel('Y [m]');
    
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