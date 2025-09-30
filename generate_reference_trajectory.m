function [x_ref, y_ref, theta_ref] = generate_reference_trajectory(type, N_steps, dt, num_laps)
    % Generate practical reference trajectories for mobile robot control
    % Inputs:
    %   type - trajectory type: 'circle', 'figure_eight'
    %   N_steps - total number of time steps
    %   dt - time step size (sampling period)
    %   num_laps - number of cycles/laps to complete
    % Outputs:
    %   x_ref - reference x-coordinates [1 x N_steps]
    %   y_ref - reference y-coordinates [1 x N_steps]
    %   theta_ref - reference orientation angles [1 x N_steps]
    
    t = 0:dt:(N_steps-1)*dt;
    total_time = (N_steps-1)*dt;
    
    switch type
        case 'circle'
            % Circular trajectory
            R = 1.5;  % radius [m]
            omega = 2*pi*num_laps/total_time;  % angular velocity [rad/s]
            x_ref = R * cos(omega * t);
            y_ref = R * sin(omega * t);
            theta_ref = omega * t + pi/2;  % tangent direction
            
        case 'figure_eight'
            % Figure-eight trajectory (Lissajous curve)
            A = 1.0;  % amplitude [m]
            omega = 2*pi*num_laps/total_time;
            x_ref = A * sin(omega * t);
            y_ref = A * sin(2 * omega * t) / 2;
            
            % Calculate heading angle from velocity
            dx_dt = A * omega * cos(omega * t);
            dy_dt = A * omega * cos(2 * omega * t);
            theta_ref = compute_smooth_angles(dx_dt, dy_dt, N_steps);
            
        
        otherwise
            error('Unknown trajectory type: %s', type);
    end
    
    % Normalize all angles to [-pi, pi]
    theta_ref = arrayfun(@wrap, theta_ref);
end

function theta_ref = compute_smooth_angles(dx_dt, dy_dt, N_steps)
    % Compute smooth heading angles with rate limiting
    % Prevents abrupt heading changes by limiting max angular change
    % Inputs:
    %   dx_dt - x-velocity components
    %   dy_dt - y-velocity components
    %   N_steps - number of time steps
    % Output:
    %   theta_ref - smoothed heading angles [rad]
    
    theta_ref = zeros(1, N_steps);
    theta_ref(1) = atan2(dy_dt(1), dx_dt(1));
    
    for i = 2:N_steps
        theta_new = atan2(dy_dt(i), dx_dt(i));
        diff = wrap(theta_new - theta_ref(i-1));
        max_change = pi/9;  % maximum angular change: 20 degrees per step
        
        if abs(diff) > max_change
            % Limit angular rate of change
            theta_ref(i) = theta_ref(i-1) + sign(diff) * max_change;
        else
            theta_ref(i) = theta_new;
        end
    end
end