function [u0, fval, U_opt] = solve_mpc_optimization( ...
    x_now, ref_horizon, Q, R, cons, N_pred, dt)
% MPC

    if nargin < 7 || isempty(dt), dt = 0.1; end

    % ----- bounds (controls only) -----
    LB = repmat([cons.v_min; cons.w_min], N_pred, 1);
    UB = repmat([cons.v_max; cons.w_max], N_pred, 1);

    % ----- trivial initial guess -----
    U0 = zeros(2*N_pred,1);

    % ----- solver options (quiet & small) -----
    options = optimoptions('fmincon', ...
        'Algorithm','sqp', 'Display','off', ...
        'MaxIterations', 50, 'MaxFunctionEvaluations', 800);

    % ----- solve (no nonlinear constraints) -----
    [U_opt, fval] = fmincon(@(U)obj(U, x_now, ref_horizon, Q, R, N_pred, dt), ...
                            U0, [],[],[],[], LB, UB, [], options);

    u0 = U_opt(1:2);  % first control to apply
end

% ===== objective =====
function J = obj(U, x0, Ref, Q, R, N, dt)
    X = rollout(x0, U, N, dt);    % [x0;y0;th0; ... ; xN;yN;thN]
    J = 0;
    for k = 1:N
        xi = X(3*k+1 : 3*k+3);
        rk = Ref(:, min(k, size(Ref,2)));    % use last ref if short
        e  = xi - rk; e(3) = wrap(e(3));
        uk = U(2*k-1 : 2*k);
        J  = J + e.'*Q*e + uk.'*R*uk;
    end
end

% ===== forward rollout =====
function X = rollout(x0, U, N, dt)
    X = zeros(3*(N+1),1); X(1:3) = x0;
    for k = 1:N
        i0 = 3*(k-1)+1 : 3*k;
        i1 = 3*k+1     : 3*(k+1);
        uk = U(2*k-1:2*k);        % [v; w]
        xk = X(i0);
        [phi1, x1, y1] = compute_next_pose(xk(3), xk(1), xk(2), uk(1), uk(2), dt);
        X(i1) = [x1; y1; wrap(phi1)];
    end
end
