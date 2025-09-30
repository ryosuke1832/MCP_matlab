
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
