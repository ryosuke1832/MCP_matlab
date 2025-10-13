function ref_horizon = extract_reference_horizon(x_ref, y_ref, theta_ref, k, N_pred, N_total)
    end_idx = min(k + N_pred - 1, N_total);
    ref_horizon = [x_ref(k:end_idx); y_ref(k:end_idx); theta_ref(k:end_idx)];
    if size(ref_horizon, 2) < N_pred
        last_col = ref_horizon(:, end);
        ref_horizon(:, end+1:N_pred) = repmat(last_col, 1, N_pred - size(ref_horizon,2));
    end
end