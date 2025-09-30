%% Real-time MPC for Mobile Robot - Project 9
clc; clf; clear all;

%% === Parameter Settings ===
dt = 0.1;  % Control period
N_total = 1200;  % Total simulation time steps
N_pred = 10;     % MPC prediction horizon

% Trajectory parameters
num_laps = 2;  % Number of figure-eight laps

% Trajectory selection
% 'circle': Circular trajectory
% 'figure_eight': Figure-eight
trajectory_type = 'figure_eight';
add_noise = true;

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
constraints = setup_constraints();

% Noise parameters (used when add_noise = true)
noise_params.position_std = 0.05;     % Position noise std dev: 5cm
noise_params.orientation_std = 0.02;  % Orientation noise std dev: ~1 degree

%% === Real-time MPC Loop ===
x_history = zeros(3, N_total);
u_history = zeros(2, N_total-1);
solve_time = zeros(1, N_total-1);
cost_history = zeros(1, N_total-1);

% Previous solution for warm start
X_prev = [];

% Progress bar
fprintf('Progress: ');

for k = 1:N_total-1
    % Progress display
    if mod(k, 40) == 0
        fprintf('%d%% ', round(100*k/N_total));
    end
    
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
    [u_optimal, fval, X_prev] = solve_mpc_optimization(x_estimated, ref_horizon, Q, R, constraints, N_pred, X_prev);
    solve_time(k) = toc;
    cost_history(k) = fval;
    
    % Save control input (use only the first step)
    u_history(:, k) = u_optimal(1:2);
    
    % State update
    [phi_next, x_next, y_next] = compute_next_pose(x_robot(3), x_robot(1), x_robot(2), ...
                                                       u_optimal(1), u_optimal(2), dt);
    x_robot = [x_next; y_next; wrap(phi_next)];
    
    % Real-time visualization (every 40 steps)
    if mod(k, 40) == 1
        visualize_current_state(x_ref, y_ref, x_history, x_robot, ref_horizon, k);
        drawnow;
    end
end

% Save final state
x_history(:, N_total) = x_robot;

%% === Performance Evaluation ===
performance = evaluate_mpc_performance(x_history, [x_ref; y_ref; theta_ref], ...
                                       u_history, solve_time, cost_history);

% Display results
fprintf('\n=== Performance Evaluation Results ===\n');
fprintf('Mean position error: %.3f m\n', performance.mean_pos_error);
fprintf('Max position error: %.3f m\n', performance.max_pos_error);
fprintf('RMSE position: %.3f m\n', performance.rmse_pos);
fprintf('Mean orientation error: %.3f rad\n', performance.mean_orient_error);
fprintf('Average solve time: %.3f ms\n', mean(solve_time)*1000);
fprintf('Total control effort: %.3f\n', performance.total_control);

%% === Comprehensive Results Visualization ===
plot_comprehensive_results(x_ref, y_ref, theta_ref, x_history, u_history, ...
                           solve_time, cost_history, performance);



function constraints = setup_constraints()
    % Constraint settings
    constraints.x_min = -10; constraints.x_max = 10;
    constraints.y_min = -10; constraints.y_max = 10;
    constraints.theta_min = -pi; constraints.theta_max = pi;
    constraints.v_min = -0.5; constraints.v_max = 0.5;  % Relaxed velocity constraints
    constraints.w_min = -pi/3; constraints.w_max = pi/3;  % Relaxed angular velocity constraints
end

function ref_horizon = extract_reference_horizon(x_ref, y_ref, theta_ref, k, N_pred, N_total)
    % Extract reference trajectory within prediction horizon
    end_idx = min(k + N_pred - 1, N_total);
    ref_horizon = [x_ref(k:end_idx); y_ref(k:end_idx); theta_ref(k:end_idx)];
    
    % Extend with last value if horizon is short
    if size(ref_horizon, 2) < N_pred
        last_ref = ref_horizon(:, end);
        for i = size(ref_horizon, 2)+1:N_pred
            ref_horizon(:, i) = last_ref;
        end
    end
end


function visualize_current_state(x_ref, y_ref, x_history, x_current, ref_horizon, k)
    % Real-time visualization
    subplot(1,2,1);
    plot(x_ref, y_ref, 'r--', 'LineWidth', 1.5); hold on;
    plot(x_history(1,1:k), x_history(2,1:k), 'b-', 'LineWidth', 2);
    plot(x_current(1), x_current(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'blue');
    plot(ref_horizon(1,:), ref_horizon(2,:), 'g*', 'MarkerSize', 6);
    
    axis equal; grid on;
    title(sprintf('MPC Trajectory Tracking (Step %d)', k));
    legend('Reference', 'Actual', 'Current Position', 'Prediction Horizon', 'Location', 'best');
    hold off;
end

function x_noisy = add_noise_to_state(x_true, noise_params)
    % Add Gaussian noise to state
    % x_true: [x; y; theta]
    % noise_params: .position_std, .orientation_std
    
    position_noise = noise_params.position_std * randn(2, 1);
    orientation_noise = noise_params.orientation_std * randn(1, 1);
    
    x_noisy = x_true + [position_noise; orientation_noise];
    
    % Normalize angle to [-pi, pi] range
    x_noisy(3) = wrap(x_noisy(3));
end

function performance = evaluate_mpc_performance(x_history, x_ref_all, u_history, solve_time, cost_history)
    % Performance evaluation
    pos_errors = sqrt((x_history(1,:) - x_ref_all(1,:)).^2 + (x_history(2,:) - x_ref_all(2,:)).^2);
    orient_errors = abs(arrayfun(@wrap, x_history(3,:) - x_ref_all(3,:)));
    
    performance.mean_pos_error = mean(pos_errors);
    performance.max_pos_error = max(pos_errors);
    performance.rmse_pos = sqrt(mean(pos_errors.^2));
    performance.mean_orient_error = mean(orient_errors);
    performance.total_control = sum(sum(u_history.^2));
    performance.avg_solve_time = mean(solve_time);
    performance.avg_cost = mean(cost_history);
end

function plot_comprehensive_results(x_ref, y_ref, theta_ref, x_history, u_history, solve_time, cost_history, perf)
    % Comprehensive results visualization
    figure('Position', [100, 100, 1400, 900]);
    
    % Trajectory comparison
    subplot(2,4,1);
    plot(x_ref, y_ref, 'r--', 'LineWidth', 2); hold on;
    plot(x_history(1,:), x_history(2,:), 'b-', 'LineWidth', 2);
    axis equal; grid on; title('Trajectory Tracking Results');
    legend('Reference', 'Actual');
    
    % Position error
    subplot(2,4,2);
    pos_error = sqrt((x_history(1,:) - x_ref).^2 + (x_history(2,:) - y_ref).^2);
    plot(pos_error, 'LineWidth', 2); grid on; title('Position Error');
    ylabel('Error [m]'); xlabel('Step');
    
    % Orientation error
    subplot(2,4,3);
    orient_error = abs(arrayfun(@wrap, x_history(3,:) - theta_ref));
    plot(orient_error, 'LineWidth', 2); grid on; title('Orientation Error');
    ylabel('Error [rad]'); xlabel('Step');
    
    % Control inputs
    subplot(2,4,4);
    plot(u_history(1,:), 'b-', 'LineWidth', 2); hold on;
    plot(u_history(2,:), 'r-', 'LineWidth', 2);
    grid on; title('Control Inputs'); legend('Velocity', 'Angular Velocity');
    
    % Solve time
    subplot(2,4,5);
    plot(solve_time*1000, 'LineWidth', 2); grid on; title('Solve Time');
    ylabel('Time [ms]'); xlabel('Step');
    
    % Cost history
    subplot(2,4,6);
    plot(cost_history, 'LineWidth', 2); grid on; title('Cost History');
    ylabel('Cost'); xlabel('Step');
    
    % Performance summary
    subplot(2,4,7);
    metrics = [perf.mean_pos_error, perf.max_pos_error, perf.rmse_pos, perf.mean_orient_error];
    bar(metrics); grid on; title('Performance Metrics');
    set(gca, 'XTickLabel', {'Mean Pos', 'Max Pos', 'RMSE Pos', 'Mean Orient'});
    
    % Statistics
    subplot(2,4,8);
    text(0.1, 0.8, sprintf('Mean position error: %.3f m', perf.mean_pos_error), 'FontSize', 10);
    text(0.1, 0.7, sprintf('Max position error: %.3f m', perf.max_pos_error), 'FontSize', 10);
    text(0.1, 0.6, sprintf('RMSE: %.3f m', perf.rmse_pos), 'FontSize', 10);
    text(0.1, 0.5, sprintf('Mean orientation error: %.3f rad', perf.mean_orient_error), 'FontSize', 10);
    text(0.1, 0.4, sprintf('Average solve time: %.1f ms', perf.avg_solve_time*1000), 'FontSize', 10);
    text(0.1, 0.3, sprintf('Total control effort: %.3f', perf.total_control), 'FontSize', 10);
    axis off; title('Statistics Summary');
end