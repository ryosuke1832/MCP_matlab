clear; close all; clc;

%% 1) CONFIG
% Timing / horizon
dt       = 0.1;      % control period [s]
N_total  = 300;     % total simulation steps
N_pred   = 10;       % prediction horizon length

% Trajectory
trajectory_type = 'figure_eight';  % 'circle' | 'figure_eight'
num_laps        = 1.0;
test_offsets = 0.0;  

% MPC weight matrices
Q = [5, 0, 0;    
     0, 5, 0;
     0, 0, 2];   
R = [0.05, 0;    
     0, 0.05];     

% Input constraints
cons.v_min = -0.5;  cons.v_max = 0.5;
cons.w_min = -pi/3; cons.w_max =  pi/3;

% Noise 
add_noise = false;
noise_params.position_std    = 0.05;  % [m]
noise_params.orientation_std = 0.02;  % [rad]

% Realtime visualization
realtime_stride = 40;

%% 2) Reference / init
[x_ref, y_ref, theta_ref] = generate_reference_trajectory(trajectory_type, N_total, dt, num_laps);
x_robot = [x_ref(1); y_ref(1)+test_offsets; theta_ref(1)];   % initial state

% Logs
x_history     = zeros(3, N_total);
u_history     = zeros(2, N_total-1);
solve_time_ms = zeros(1, N_total-1);
cost_history  = zeros(1, N_total-1);

%% 3) MPC loop
for k = 1:N_total-1
    x_history(:,k) = x_robot;

    % state estimate (with optional noise)
    if add_noise
        x_est = add_noise_to_state(x_robot, noise_params);
    else
        x_est = x_robot;
    end

    % fixed-length reference window
    ref_hor = extract_reference_horizon(x_ref, y_ref, theta_ref, k, N_pred, N_total);

    % solve MPC
    tic;
    [u_opt, fval] = solve_mpc_optimization(x_est, ref_hor, Q, R, cons, N_pred, dt);
    solve_time_ms(k) = toc * 1000;
    cost_history(k)  = fval;

    % apply first control and propagate
    u_history(:,k) = u_opt(1:2);
    [phi1, x1, y1] = compute_next_pose(x_robot(3), x_robot(1), x_robot(2), u_opt(1), u_opt(2), dt);
    x_robot = [x1; y1; wrap(phi1)];

    % realtime viz
    if mod(k, realtime_stride) == 1
        realtime_visualization(x_ref, y_ref, x_history, x_robot, ref_hor, k);
        drawnow;
    end
end
x_history(:,N_total) = x_robot;

%% 4) Report
pos_errors     = hypot(x_history(1,:) - x_ref, x_history(2,:) - y_ref);
mean_pos_error = mean(pos_errors);
avg_solve_time = mean(solve_time_ms);

fprintf('\n=== Performance ===\n');
fprintf('Mean position error: %.3f m\n',  mean_pos_error);
fprintf('Average solve time : %.3f ms\n', avg_solve_time);

%　Plots
plot_results(x_ref, y_ref, x_history, u_history, cost_history, N_pred);

