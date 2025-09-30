function [u_optimal, fval, X_opt] = solve_mpc_optimization( ...
    x_current, ref_horizon, Q, R, constraints, N_pred, X_prev, dt, opts)
%  - Terminal cost Qf
%  - Input rate penalty R_delta
%  - Warm start stabilization
%
% opts (optional):
%   .Qf_scale   : Terminal weight coefficient (default: 5) → Qf = Qf_scale * Q
%   .Rdelta_scale: Δu weight coefficient (default: 0.1) → RΔ = Rdelta_scale * R
%   .u_init      : u_{-1} for initial Δu (default: [0;0])
%   .maxIters    : fmincon MaxIterations (default: 50)
%   .maxFEs      : fmincon MaxFunctionEvaluations (default: 1000)

    if nargin < 9 || isempty(dt), dt = 0.1; end
    if nargin < 10, opts = struct(); end
    if ~isfield(opts,'Qf_scale'),      opts.Qf_scale = 5;    end
    if ~isfield(opts,'Rdelta_scale'),  opts.Rdelta_scale = 0.1; end
    if ~isfield(opts,'u_init'),        opts.u_init = [0;0];  end
    if ~isfield(opts,'maxIters'),      opts.maxIters = 50;   end
    if ~isfield(opts,'maxFEs'),        opts.maxFEs = 1000;   end

    Qf     = opts.Qf_scale * Q;
    Rdelta = opts.Rdelta_scale * R;

    % Variable dimensions
    n_states   = 3 * (N_pred + 1);
    n_controls = 2 * N_pred;
    n_vars     = n_states + n_controls;

    % ===== Initial guess (warm start) =====
    X0 = build_initial_guess(X_prev, x_current, ref_horizon, N_pred, n_states, n_controls);

    % ===== Bound constraints =====
    [LB, UB] = setup_bounds(constraints, N_pred);

    % ===== fmincon options =====
    options = optimoptions('fmincon', ...
        'Algorithm','sqp', 'Display','off', ...
        'MaxIterations', opts.maxIters, ...
        'MaxFunctionEvaluations', opts.maxFEs, ...
        'ConstraintTolerance',5e-3, 'OptimalityTolerance',5e-3, 'StepTolerance',1e-5);

    % ===== Execute optimization =====
    [X_opt, fval] = fmincon( ...
        @(X) mpc_objective_function(X, ref_horizon, Q, R, Qf, Rdelta, N_pred), ...
        X0, [],[],[],[], LB,UB, ...
        @(X) mpc_constraint_function(X, x_current, N_pred, dt, constraints), ...
        options);

    % Extract first control input
    control_start_idx = n_states + 1;
    u_optimal = X_opt(control_start_idx:control_start_idx+1);
end


function X0 = build_initial_guess(X_prev, x_current, ref_horizon, N_pred, n_states, n_controls)
    % Warm start using reference and previous solution (stabilized version)
    if isempty(X_prev) || numel(X_prev) ~= (n_states + n_controls)
        X0 = zeros(n_states + n_controls,1);
        % x0
        X0(1:3) = x_current;
        % Fill x1..xN with reference
        for i = 1:N_pred
            si = 3*i+1 : 3*i+3;
            if i <= size(ref_horizon,2)
                X0(si) = ref_horizon(:,i);
            else
                X0(si) = ref_horizon(:,end);
            end
        end
        % Initialize u with 0.1
        X0(n_states+1 : n_states+n_controls) = 0.1;
    else
        X0 = zeros(n_states + n_controls,1);
        % x0 is current value
        X0(1:3) = x_current;
        % Shift states left by one step: prev(x1..xN) -> X0(x0..xN-1)
        for i = 1:N_pred
            prev_idx = 3*(i)+1 : 3*(i+1);     % Previous x_i (i=1..N_pred) ※x0 treated separately
            curr_idx = 3*(i-1)+1 : 3*i;       % Current x_{i-1}
            X0(curr_idx) = X_prev(prev_idx);
        end
        % Terminal state x_N copies from previous x_N
        X0(3*N_pred+1 : 3*(N_pred+1)) = X_prev(3*N_pred+1 : 3*(N_pred+1));
        % Shift inputs left: prev(u1..uN-1) -> X0(u0..uN-2), copy last
        control_start = n_states + 1;
        for i = 1:N_pred-1
            prev_idx = control_start + 2*i   : control_start + 2*i + 1;     % Previous u_i  (i=1..N_pred-1)
            curr_idx = control_start + 2*(i-1) : control_start + 2*(i-1) + 1; % Current u_{i-1}
            X0(curr_idx) = X_prev(prev_idx);
        end
        last_control_idx = control_start + 2*(N_pred-1) : control_start + 2*(N_pred-1) + 1;
        X0(last_control_idx) = X_prev(last_control_idx);
    end
end


function [LB, UB] = setup_bounds(constraints, N_pred)
    % Setup bound constraints (states are for reference; actual enforcement can be done in NONLCON)
    n_states   = 3 * (N_pred + 1);
    n_controls = 2 * N_pred;

    LB_states = repmat([constraints.x_min; constraints.y_min; constraints.theta_min], N_pred+1, 1);
    UB_states = repmat([constraints.x_max; constraints.y_max; constraints.theta_max], N_pred+1, 1);

    LB_controls = repmat([constraints.v_min; constraints.w_min], N_pred, 1);
    UB_controls = repmat([constraints.v_max; constraints.w_max], N_pred, 1);

    LB = [LB_states; LB_controls];
    UB = [UB_states; UB_controls];
end


function f = mpc_objective_function(X, ref_horizon, Q, R, Qf, Rdelta, N_pred)
    % Objective function: tracking error + input + input rate + terminal error
    n_states = 3 * (N_pred + 1);
    u0 = X(n_states+1 : n_states+2); % First input
    f  = 0;

    % Stage cost (1..N_pred)
    for i = 1:N_pred
        si = 3*i + 1:3*i + 3;    % x_i
        xi = X(si);
        if i <= size(ref_horizon,2), ri = ref_horizon(:,i); else, ri = ref_horizon(:,end); end
        e  = xi - ri; e(3) = wrap(e(3));
        ui = X(n_states + 2*(i-1)+1 : n_states + 2*i);
        f = f + e.'*Q*e + ui.'*R*ui;

        % Δu penalty (for i>=2, u_i - u_{i-1})
        if i >= 2
            up = X(n_states + 2*(i-2)+1 : n_states + 2*(i-1));
            du = ui - up;
            f  = f + du.'*Rdelta*du;
        end
    end

    % Terminal cost (x_N vs final reference)
    sN = 3*N_pred + 1 : 3*(N_pred+1);
    xN = X(sN);
    rN = ref_horizon(:, min(N_pred, size(ref_horizon,2)));
    eN = xN - rN; eN(3) = wrap(eN(3));
    f  = f + eN.'*Qf*eN;
end


function [c, ceq] = mpc_constraint_function(X, x_initial, N_pred, dt, constraints)
    % Dynamic equality constraints + (optional) obstacle/workspace inequality constraints
    n_states = 3 * (N_pred + 1);
    control_start_idx = n_states + 1;

    c   = [];     % Inequality
    ceq = [];     % Equality

    % Initial state constraint
    ceq = [ceq; X(1:3) - x_initial];

    % Discrete kinematics (using compute_next_pose)
    for i = 1:N_pred
        si0 = 3*(i-1)+1 : 3*i;         % x_{i-1}
        si1 = 3*i+1       : 3*(i+1);   % x_i
        ui  = control_start_idx + 2*(i-1) : control_start_idx + 2*i - 1;

        xk  = X(si0);
        uk  = X(ui);
        [phi_pred, x_pred, y_pred] = compute_next_pose(xk(3), xk(1), xk(2), uk(1), uk(2), dt);
        x_pred_vec = [x_pred; y_pred; wrap(phi_pred)];

        ceq = [ceq; X(si1) - x_pred_vec];
    end

end