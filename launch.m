%% Real-time MPC for Mobile Robot - Project 9
clear; close all; clc;

%% === Parameter Settings ===
dt = 0.1;  % Control period
N_total = 1200;  % Total simulation time steps
N_pred = 10;     % MPC prediction horizon

% Trajectory parameters
num_laps = 1;  % Number of figure-eight laps

% Trajectory selection
% 'circle': Circular trajectory
% 'figure_eight': Figure-eight
trajectory_type = 'figure_eight';
add_noise = false;

%% === Trajectory Generation ===
[x_ref, y_ref, theta_ref] = generate_reference_trajectory(trajectory_type, N_total, dt, num_laps);

%% === Initial Setup ===
% Initial state
x_initial = [x_ref(1); y_ref(1); theta_ref(1)];
x_robot = x_initial;

% MPC weight matrices
Q = [5, 0, 0;    
     0, 5, 0;
     0, 0, 2];   
R = [0.05, 0;    
     0, 0.05];

% Constraints
cons.v_min = -0.5;   cons.v_max = 0.5;    % velocity
cons.w_min = -pi/3;  cons.w_max = pi/3; 

% Noise parameters (used when add_noise = true)
noise_params.position_std = 0.05;     % Position noise std dev: 5cm
noise_params.orientation_std = 0.02;  % Orientation noise std dev: ~1 degree

%% === Real-time MPC Loop ===
x_history = zeros(3, N_total);
u_history = zeros(2, N_total-1);
solve_time = zeros(1, N_total-1);
cost_history = zeros(1, N_total-1);

% Progress bar
fprintf('Progress: ');

for k = 1:N_total-1
    
    % Save current state
    x_history(:, k) = x_robot;
    
    % Add noise
    if add_noise
        x_estimated = add_noise_to_state(x_robot, noise_params);
    else
        x_estimated = x_robot;
    end
    
    % Extract reference trajectory within prediction horizon
    ref_horizon = extract_reference_horizon(x_ref, y_ref, theta_ref, k, N_pred, N_total);
    
    % Execute MPC optimization
    tic;
    [u_optimal, fval] = solve_mpc_optimization(x_estimated, ref_horizon, Q, R, cons, N_pred, dt);
    solve_time(k) = toc;
    cost_history(k) = fval;
    
    % Save control input (use only the first step)
    u_history(:, k) = u_optimal(1:2);
    
    % State update
    [phi_next, x_next, y_next] = compute_next_pose(x_robot(3), x_robot(1), x_robot(2), ...
                                                       u_optimal(1), u_optimal(2), dt);
    x_robot = [x_next; y_next; wrap(phi_next)];
    
    % Real-time visualization
    if mod(k, 40) == 1
        realtime_visualization(x_ref, y_ref, x_history, x_robot, ref_horizon, k);
        drawnow;
    end
end

% Save final state
x_history(:, N_total) = x_robot;

%% === Performance Evaluation ===
pos_errors = sqrt((x_history(1,:) - x_ref).^2 + (x_history(2,:) - y_ref).^2);
mean_pos_error = mean(pos_errors);
avg_solve_time = mean(solve_time)*1000;

fprintf('\n=== Performance Evaluation Results ===\n');
fprintf('Mean position error: %.3f m\n', mean_pos_error);
fprintf('Average solve time : %.3f ms\n', avg_solve_time);


%% === Results Visualization ===
plot_results(x_ref, y_ref, x_history, u_history);


function ref_horizon = extract_reference_horizon(x_ref, y_ref, theta_ref, k, N_pred, N_total)
    end_idx = min(k + N_pred - 1, N_total);
    ref_horizon = [x_ref(k:end_idx); y_ref(k:end_idx); theta_ref(k:end_idx)];
    if size(ref_horizon, 2) < N_pred
        last_col = ref_horizon(:, end);
        ref_horizon(:, end+1:N_pred) = repmat(last_col, 1, N_pred - size(ref_horizon,2));
    end
end





