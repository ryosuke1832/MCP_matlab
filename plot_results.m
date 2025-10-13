%% === Results Visualization ===
function plot_results(x_ref, y_ref, x_history, u_history, cost_history, N_pred)
    figure('Position', [100,100,1300,750]);

    % (1) Trajectory: reference vs actual
    subplot(2,3,[1 4]); 
    plot(x_ref, y_ref, 'r--', 'LineWidth', 2); hold on;
    plot(x_history(1,:), x_history(2,:), 'b-', 'LineWidth', 2);
    axis equal; grid on;
    title('Trajectory Tracking'); xlabel('x [m]'); ylabel('y [m]');
    legend('Reference','Actual','Location','best');

    % (2) Position error
    subplot(2,3,2);
    pos_err = sqrt((x_history(1,:) - x_ref).^2 + (x_history(2,:) - y_ref).^2);
    plot(pos_err, 'LineWidth', 2);
    grid on; title('Position Error'); ylabel('Error [m]'); xlabel('Step');

    % (3) Control inputs
    subplot(2,3,5);
    plot(u_history(1,:), 'LineWidth', 2); hold on;
    plot(u_history(2,:), 'LineWidth', 2);
    grid on; title('Control Inputs'); xlabel('Step');
    legend('v [m/s]','w [rad/s]','Location','best');

    % (4) Predicted horizon cost
    subplot(2,3,[3 6]);
    plot(cost_history, 'LineWidth', 2); hold on;
    plot(cost_history/N_pred, '--', 'LineWidth', 1.5);
    grid on; 
    title('Predicted Horizon Cost (fmincon fval)'); 
    xlabel('Step'); ylabel('Cost J');
    legend('Total cost over horizon','Cost per step (normalized)','Location','best');
end
