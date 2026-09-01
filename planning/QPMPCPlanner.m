classdef QPMPCPlanner < handle
    % QPMPCPLANNER Linearized Model Predictive Controller (Stage 3 Baseline)
    %
    % Purpose:
    %   Solves a linearized condensed quadratic program (QP) at each timestep using
    %   the discrete kinematic bicycle model linearized around current operating state.
    %   Uses a Condensed MPC formulation (state equality elimination S_u, S_x, S_c)
    %   where r_eq = 0 is guaranteed by construction, solved via Hildreth Dual Coordinate
    %   Descent with strict 4-condition KKT residual auditing (r_primal, r_dual, r_comp, r_stat < 1e-3).
    %
    % Mathematical Formulation:
    %   State:   x = [p_x, p_y, theta, v]^T \in R^4
    %   Control: u = [delta, a]^T \in R^2
    %   Condensed Decision Vector: U = [u_0; u_1; ...; u_{Np-1}] \in R^{2 N_p}
    %   State Trajectory: X = S_x * x_0 + S_u * U + S_c
    %
    % Linearization:
    %   Single-point successive Forward-Euler discretization per MPC update step around (x_0, u_op).
    
    properties
        N_p             int32 = 20           % Prediction horizon (20 steps = 2.0s lookahead)
        dt              double = 0.10        % Sample time step (s)
        L               double = 2.70        % Wheelbase (m)
        vehicle_width   double = 1.80        % Vehicle width (m)
        
        % Cost Weights (Q_diag: [x, y, theta, v], R_diag: [delta, a])
        Q_diag          double = [1.0, 20.0, 10.0, 20.0]
        R_diag          double = [10.0, 0.1]
        R_rate_diag     double = [50.0, 1.0]
        
        % Control Limits
        max_steering    double = 0.6109      % deg2rad(35)
        max_accel       double = 3.0
        max_decel       double = -6.0
        max_steer_rate  double = 0.5236      % deg2rad(30) rad/s
        
        % Memory & Diagnostics
        last_u          double = [0; 0]      % Control memory for rate penalties
        last_solve_time double = 0.0         % Pure QP solver time (ms)
        last_total_time double = 0.0         % Total planner execution time (ms)
        last_residuals  struct = struct('r_primal', 0, 'r_dual', 0, 'r_comp', 0, 'r_stat', 0)
        last_iterations int32 = 0            % Iteration count
    end
    
    methods
        function obj = QPMPCPlanner(varargin)
            % Constructor
            if nargin > 0 && isa(varargin{1}, 'SimulationConfig')
                cfg = varargin{1};
                obj.dt = cfg.dt;
                obj.L = cfg.wheelbase;
                obj.vehicle_width = cfg.vehicle_width;
                obj.max_accel = cfg.a_max;
                obj.max_decel = cfg.a_min;
                obj.max_steering = cfg.delta_max;
                obj.max_steer_rate = cfg.delta_rate_max;
            end
        end
        
        function reset(obj)
            % RESET Restores control memory and diagnostic history
            obj.last_u = [0; 0];
            obj.last_solve_time = 0.0;
            obj.last_total_time = 0.0;
            obj.last_residuals = struct('r_primal', 0, 'r_dual', 0, 'r_comp', 0, 'r_stat', 0);
            obj.last_iterations = 0;
        end
        
        function [u_opt, pred_states, status, info] = plan(obj, world, reference_path, target_speed)
            % PLAN Solves condensed QP-MPC optimization problem for current ego state.
            
            t_plan_start = tic;
            ego = world.ego;
            x0 = [ego.x; ego.y; ego.theta; ego.v];
            
            % 1. Reference Trajectory Extraction along N_p
            ref_xy = reference_path(:, 1:2);
            dists = (ref_xy(:, 1) - ego.x).^2 + (ref_xy(:, 2) - ego.y).^2;
            [~, nearest_idx] = min(dists);
            
            num_ref = size(reference_path, 1);
            x_ref_traj = zeros(obj.N_p, 4);
            
            for k = 1:obj.N_p
                idx = min(nearest_idx + k - 1, num_ref);
                ref_pt = reference_path(idx, :);
                x_ref_traj(k, :) = [ref_pt(1), ref_pt(2), ref_pt(3), target_speed];
            end
            
            % Stacked reference trajectory vector X_ref \in R^{4 Np}
            X_ref = reshape(x_ref_traj', [], 1);
            
            % 2. Successive Single-Point Discrete Linearization
            u_op = obj.last_u;
            [A_d, B_d, c_d] = obj.linearize_kinematics(x0, u_op);
            
            % 3. Build Condensed Matrices: X = S_x * x0 + S_u * U + S_c
            Np = double(obj.N_p);
            nx = 4; nu = 2;
            
            S_x = zeros(nx * Np, nx);
            S_u = zeros(nx * Np, nu * Np);
            S_c = zeros(nx * Np, 1);
            
            A_pow = eye(nx);
            c_accum = zeros(nx, 1);
            
            for k = 1:Np
                c_accum = c_accum + A_pow * c_d;
                A_pow = A_pow * A_d;
                
                idx_row = (k - 1) * nx + (1:nx);
                S_x(idx_row, :) = A_pow;
                S_c(idx_row) = c_accum;
                
                % Fill S_u columns
                A_block = eye(nx);
                for j = k:-1:1
                    idx_col = (j - 1) * nu + (1:nu);
                    S_u(idx_row, idx_col) = A_block * B_d;
                    A_block = A_block * A_d;
                end
            end
            
            % 4. Build Condensed Objective Matrix H_u and Vector f_u
            Q_single = diag(obj.Q_diag);
            Q_bar = kron(eye(Np), Q_single);
            
            R_single = diag(obj.R_diag);
            R_bar = kron(eye(Np), R_single);
            
            R_rate_single = diag(obj.R_rate_diag);
            D_rate = eye(nu * Np);
            for k = 2:Np
                idx_curr = (k - 1) * nu + (1:nu);
                idx_prev = (k - 2) * nu + (1:nu);
                D_rate(idx_curr, idx_prev) = -eye(nu);
            end
            R_rate_bar = D_rate' * kron(eye(Np), R_rate_single) * D_rate;
            
            H_u = S_u' * Q_bar * S_u + R_bar + R_rate_bar;
            H_u = (H_u + H_u') / 2;
            
            X_free = S_x * x0 + S_c;
            f_u = S_u' * Q_bar * (X_free - X_ref);
            
            rate_offset = zeros(nu * Np, 1);
            rate_offset(1:nu) = -R_rate_single * u_op;
            f_u = f_u + rate_offset;
            
            % 5. Build Control Bounds (lb_u <= U <= ub_u)
            lb_u = zeros(nu * Np, 1);
            ub_u = zeros(nu * Np, 1);
            for k = 1:Np
                idx_u = (k - 1) * nu + (1:nu);
                lb_u(idx_u(1)) = -obj.max_steering;
                ub_u(idx_u(1)) =  obj.max_steering;
                lb_u(idx_u(2)) =  obj.max_decel;
                ub_u(idx_u(2)) =  obj.max_accel;
            end
            
            % 6. Build Linear Inequalities A_u * U <= b_u for Road Bounds & Obstacles
            A_u = [];
            b_u = [];
            
            % Lateral Road Bounds (y_min <= y_k <= y_max)
            bounds = world.getRoadBounds();
            half_W_bound = obj.vehicle_width / 2 + 0.20; % 1.10m
            y_min_safe = bounds(3) + half_W_bound;     % 1.10m
            y_max_safe = bounds(4) - half_W_bound;     % 4.90m
            
            for k = 1:Np
                row_y_S_u = S_u((k - 1) * nx + 2, :);
                y_free_k = X_free((k - 1) * nx + 2);
                
                A_u = [A_u; row_y_S_u];
                b_u = [b_u; y_max_safe - y_free_k];
                
                A_u = [A_u; -row_y_S_u];
                b_u = [b_u; -y_min_safe + y_free_k];
            end
            
            % Fixed-Side Lateral Corridor Obstacle Constraints
            % NOTE: The passing side (left vs right) is fixed based on current vehicle lateral position (x0(2) < obs_y).
            % This is a Fixed-Side Lateral Corridor MPC baseline formulation.
            has_corridor_conflict = false;
            if isprop(world, 'n_static_obs') && world.n_static_obs > 0
                n_obs = world.n_static_obs;
                r_ego = obj.vehicle_width / 2; % 0.9m
                
                x_pred_0 = zeros(Np, 4);
                x_curr_0 = x0;
                for k = 1:Np
                    x_curr_0 = A_d * x_curr_0 + B_d * u_op + c_d;
                    x_pred_0(k, :) = x_curr_0';
                end
                
                for obs_i = 1:n_obs
                    obs_x = world.static_obs(obs_i, 1);
                    obs_y = world.static_obs(obs_i, 2);
                    r_obs = world.static_obs(obs_i, 4) / 2;
                    d_req = r_ego + r_obs + 0.10;
                    
                    % Check for geometric road-bound conflict
                    if (obs_y - d_req < y_min_safe) && (obs_y + d_req > y_max_safe)
                        has_corridor_conflict = true;
                    end
                    
                    for k = 1:Np
                        px0 = x_pred_0(k, 1);
                        dx = px0 - obs_x;
                        
                        if abs(dx) < 6.0
                            row_y_S_u = S_u((k - 1) * nx + 2, :);
                            y_free_k = X_free((k - 1) * nx + 2);
                            
                            if x0(2) < obs_y
                                A_u = [A_u; row_y_S_u];
                                b_u = [b_u; (obs_y - d_req) - y_free_k];
                            else
                                A_u = [A_u; -row_y_S_u];
                                b_u = [b_u; -(obs_y + d_req) + y_free_k];
                            end
                        end
                    end
                end
            end
            
            % 7. Solve Condensed QP via Hildreth Dual Coordinate Descent
            t_qp_start = tic;
            [U_opt, iter, residuals, solver_ok] = obj.hildreth_dual_qp(H_u, f_u, A_u, b_u, lb_u, ub_u);
            obj.last_solve_time = toc(t_qp_start) * 1000; % ms
            obj.last_total_time = toc(t_plan_start) * 1000; % ms
            obj.last_residuals = residuals;
            obj.last_iterations = iter;
            
            info = struct();
            info.solve_time_ms = obj.last_solve_time;
            info.total_time_ms = obj.last_total_time;
            info.iterations = iter;
            info.residuals = residuals;
            
            if solver_ok
                status = 1;
                u_opt = U_opt(1:nu);
                info.infeasibility_reason = 'none';
                
                X_pred = S_x * x0 + S_u * U_opt + S_c;
                pred_states = reshape(X_pred, nx, Np)';
            else
                status = 0;
                u_opt = u_op;
                pred_states = repmat(x0', Np, 1);
                
                if has_corridor_conflict
                    info.infeasibility_reason = 'inherent_corridor_conflict';
                else
                    info.infeasibility_reason = 'steering_or_road_bound_conflict';
                end
            end
            
            obj.last_u = u_opt;
        end
        
        function [A_d, B_d, c_d] = linearize_kinematics(obj, x_op, u_op)
            % LINEARIZE_KINEMATICS Discrete Taylor expansion around (x_op, u_op) via Forward Euler
            v = max(0.50, x_op(4)); % Minimum velocity floor for kinematic controllability at standstill
            theta = x_op(3);
            delta = u_op(1);
            
            A_c = zeros(4, 4);
            A_c(1, 3) = -v * sin(theta);
            A_c(1, 4) =  cos(theta);
            A_c(2, 3) =  v * cos(theta);
            A_c(2, 4) =  sin(theta);
            A_c(3, 4) =  tan(delta) / obj.L;
            
            B_c = zeros(4, 2);
            B_c(3, 1) = (v / obj.L) * (sec(delta)^2);
            B_c(4, 2) = 1.0;
            
            f_op = [v * cos(theta);
                    v * sin(theta);
                    (v / obj.L) * tan(delta);
                    u_op(2)];
                
            c_c = f_op - A_c * x_op - B_c * u_op;
            
            A_d = eye(4) + A_c * obj.dt;
            B_d = B_c * obj.dt;
            c_d = c_c * obj.dt;
        end
        
        function [U_opt, iter, residuals, solver_ok] = hildreth_dual_qp(~, H, f, A_ineq, b_ineq, lb, ub)
            % HILDRETH_DUAL_QP Hildreth-Type Dual Coordinate-Descent QP Solver
            % Solves min 0.5 U' H U + f' U s.t. A_ineq U <= b_ineq, lb <= U <= ub
            
            n = length(f);
            H = (H + H') / 2 + 1e-4 * eye(n);
            
            M = [];
            d = [];
            
            if ~isempty(A_ineq)
                M = [M; A_ineq];
                d = [d; b_ineq];
            end
            
            idx_ub = find(~isinf(ub));
            if ~isempty(idx_ub)
                M_ub = zeros(length(idx_ub), n);
                for i = 1:length(idx_ub)
                    M_ub(i, idx_ub(i)) = 1.0;
                end
                M = [M; M_ub];
                d = [d; ub(idx_ub)];
            end
            
            idx_lb = find(~isinf(lb));
            if ~isempty(idx_lb)
                M_lb = zeros(length(idx_lb), n);
                for i = 1:length(idx_lb)
                    M_lb(i, idx_lb(i)) = -1.0;
                end
                M = [M; M_lb];
                d = [d; -lb(idx_lb)];
            end
            
            % Unconstrained solution via backslash solve (no inv(H))
            U0 = - (H \ f);
            
            m_total = length(d);
            lambda = zeros(m_total, 1);
            
            if m_total == 0 || max(M * U0 - d) <= 1e-4
                U_opt = U0;
                iter = 0;
            else
                % Dual matrix P = M * (H \ M^T) via backslash solve
                P = M * (H \ M');
                q = d - M * U0;
                
                max_iter = 600;
                iter = max_iter;
                
                for k = 1:max_iter
                    lambda_prev = lambda;
                    for i = 1:m_total
                        w = P(i, :) * lambda - P(i, i) * lambda(i) + q(i);
                        if P(i, i) > 1e-8
                            lambda(i) = max(0.0, -w / P(i, i));
                        end
                    end
                    if norm(lambda - lambda_prev) < 1e-6
                        iter = k;
                        break;
                    end
                end
                
                % Primal recovery: U_opt = -H \ (f + M' * lambda)
                U_opt = - (H \ (f + M' * lambda));
            end
            
            % NO post-solve clipping! The output U_opt must satisfy constraints natively.
            
            % Complete 4-Condition KKT Residual Audit
            residuals = struct();
            
            % 1. Primal Feasibility Residual: r_primal = max(0, max(M * U_opt - d))
            if ~isempty(M)
                residuals.r_primal = max(0.0, max(M * U_opt - d));
            else
                residuals.r_primal = 0.0;
            end
            
            % 2. Dual Feasibility Residual: r_dual = max(0, -min(lambda))
            if ~isempty(lambda)
                residuals.r_dual = max(0.0, -min(lambda));
            else
                residuals.r_dual = 0.0;
            end
            
            % 3. Complementarity Residual: r_comp = max(|lambda_i * (M_i * U_opt - d_i)|)
            if ~isempty(M)
                residuals.r_comp = max(abs(lambda .* (M * U_opt - d)));
            else
                residuals.r_comp = 0.0;
            end
            
            % 4. Stationarity Residual: r_stat = ||H * U_opt + f + M' * lambda||_\infty
            grad_L = H * U_opt + f;
            if ~isempty(M)
                grad_L = grad_L + M' * lambda;
            end
            residuals.r_stat = max(abs(grad_L));
            
            % Strict Gate on ALL 4 KKT Conditions
            solver_ok = (residuals.r_primal < 1e-3) && ...
                        (residuals.r_dual < 1e-3) && ...
                        (residuals.r_comp < 1e-3) && ...
                        (residuals.r_stat < 1e-3);
        end
    end
end
