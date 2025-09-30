%% === Minimal Results Visualization ===

function plot_results(x_ref, y_ref, x_history, u_history)
    figure('Position', [100,100,1100,650]);

    % (1) Trajectory: reference vs actual
    subplot(2,2,[1 3]); % 大きめに
    plot(x_ref, y_ref, 'r--', 'LineWidth', 2); hold on;
    plot(x_history(1,:), x_history(2,:), 'b-', 'LineWidth', 2);
    axis equal; grid on;
    title('Trajectory Tracking'); xlabel('x [m]'); ylabel('y [m]');
    legend('Reference','Actual','Location','best');

    % (2) Position error (single curve)
    subplot(2,2,2);
    pos_err = sqrt((x_history(1,:) - x_ref).^2 + (x_history(2,:) - y_ref).^2);
    plot(pos_err, 'LineWidth', 2);
    grid on; title('Position Error'); ylabel('Error [m]'); xlabel('Step');

    % (3) Control inputs v, w
    subplot(2,2,4);
    plot(u_history(1,:), 'LineWidth', 2); hold on;
    plot(u_history(2,:), 'LineWidth', 2);
    grid on; title('Control Inputs'); xlabel('Step');
    legend('v [m/s]','w [rad/s]','Location','best');
end
