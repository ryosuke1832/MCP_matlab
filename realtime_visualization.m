function realtime_visualization(x_ref, y_ref, x_history, x_current, ref_horizon, k)
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


