% ===== N_pred Cross validation  =====
do_npred_sweep = true;

if do_npred_sweep
    Qfix = diag([5,5,2]);
    Rfix = diag([0.05,0.05]);

    Npred_grid = [3 5 7 8 9 10 12 15 18 22 26 30];
    noise_grid = [false true];

    % Storage
    err_mean  = nan(numel(noise_grid), numel(Npred_grid));
    time_mean = nan(numel(noise_grid), numel(Npred_grid));
    time_std  = nan(numel(noise_grid), numel(Npred_grid));

    fprintf('\n=== N_pred-only Sweep (%d x %d cases) ===\n', ...
            numel(noise_grid), numel(Npred_grid));

    % Run all cases
    for iNoise = 1:numel(noise_grid)
        noise = noise_grid(iNoise);
        for j = 1:numel(Npred_grid)
            Np = Npred_grid(j);
            [mean_err, avg_ms, allTimes] = run_once(Qfix, Rfix, Np, noise);

            err_mean(iNoise,j)  = mean_err;
            time_mean(iNoise,j) = mean(allTimes);
            time_std(iNoise,j)  = std(allTimes);

            fprintf('noise=%d | Np=%2d | Err=%.3f m | Time=%.2f ms\n', ...
                     noise, Np, mean_err, avg_ms);
        end
    end

    % ---- Plot results ----
    figure('Position',[100,100,1000,400]);

    % (1) Error vs N_pred (mean only)
    subplot(1,2,1);
    plot(Npred_grid, err_mean(1,:), '-o','LineWidth',1.5); hold on;
    plot(Npred_grid, err_mean(2,:), '-s','LineWidth',1.5);
    grid on; xlabel('N_{pred}'); ylabel('Mean position error [m]');
    title('Error vs N_{pred}');
    legend('Noise off','Noise on','Location','best');

    % (2) Solve time vs N_pred (mean ± std)
    subplot(1,2,2);
    errorbar(Npred_grid, time_mean(1,:), time_std(1,:), '-o','LineWidth',1.5); hold on;
    errorbar(Npred_grid, time_mean(2,:), time_std(2,:), '-s','LineWidth',1.5);
    yline(100, '--', 'dt = 100 ms', 'LineWidth',1.2); % control period line
    grid on; xlabel('N_{pred}'); ylabel('Solve time [ms]');
    title('Solve time vs N_{pred}');
    legend('Noise off','Noise on', 'Location','northwest');
end


% ===== run once =====
function [mean_err, avg_ms, allTimes] = run_once(Q_in, R_in, Npred_in, add_noise_in)
    % Fixed settings
    dt = 0.1; N_total = 300;
    trajectory_type = 'figure_eight'; num_laps = 1;
    cons.v_min = -0.5; cons.v_max = 0.5;
    cons.w_min = -pi/3; cons.w_max = pi/3;
    noise_params.position_std = 0.05;
    noise_params.orientation_std = 0.02;

    % Variable
    Q = Q_in; R = R_in; N_pred = Npred_in; add_noise = add_noise_in;
    if add_noise, rng(0); end  % reproducibility

    % Reference & init
    [x_ref, y_ref, theta_ref] = generate_reference_trajectory(trajectory_type, N_total, dt, num_laps);
    x_robot = [x_ref(1); y_ref(1); theta_ref(1)];
    x_history = zeros(3, N_total);
    allTimes  = zeros(1, N_total-1);

    % Loop
    for k = 1:N_total-1
        x_history(:,k) = x_robot;
        if add_noise
            x_est = add_noise_to_state(x_robot, noise_params);
        else
            x_est = x_robot;
        end
        ref_hor = extract_reference_horizon(x_ref, y_ref, theta_ref, k, N_pred, N_total);

        tStart = tic;
        [u_opt, ~] = solve_mpc_optimization(x_est, ref_hor, Q, R, cons, N_pred, dt);
        allTimes(k) = toc(tStart)*1000;

        u = u_opt(1:2);
        [phi1, x1, y1] = compute_next_pose(x_robot(3), x_robot(1), x_robot(2), u(1), u(2), dt);
        x_robot = [x1; y1; wrap(phi1)];
    end
    x_history(:,N_total) = x_robot;

    % Metrics
    pos_errors = hypot(x_history(1,:) - x_ref, x_history(2,:) - y_ref);
    mean_err   = mean(pos_errors);
    avg_ms     = mean(allTimes);
end
