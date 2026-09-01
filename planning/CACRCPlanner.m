classdef CACRCPlanner < handle
    % CACRCPLANNER Context-Adaptive Risk-Sensitive Multi-Topology Planner (Stage 4)
    %
    % Purpose:
    %   Evaluates both topological passing corridors (J_left vs J_right) explicitly,
    %   incorporates time-varying risk uncertainty margins beta * sigma_y(k),
    %   and implements Layer 1 Slack-Softened QP fallback to resolve topological infeasibilities.
    
    properties
        N_p             int32 = 20           % Prediction horizon (20 steps = 2.0s)
        dt              double = 0.10        % Sample time step (s)
        L               double = 2.70        % Wheelbase (m)
        vehicle_width   double = 1.80        % Vehicle width (m)
        
        % Cost Weights
        Q_diag          double = [1.0, 20.0, 80.0, 50.0]
        R_diag          double = [50.0, 0.1]
        R_rate_diag     double = [100.0, 0.5]
        W_slack         double = 1000.0      % Soft slack penalty weight (conditioned for Hildreth QP)
        
        % Control Limits
        max_steering    double = 0.6109      % deg2rad(35)
        max_accel       double = 3.0
        max_decel       double = -6.0
        
        % Memory & Diagnostics
        last_u          double = [0; 0]
        locked_side     char   = 'none'      % Persistent topological corridor memory ('left', 'right', or 'none')
        qp_base_planner QPMPCPlanner        % Underlying Hildreth QP engine
        predictor       PredictorModule     % Obstacle predictor
        bound_provider                      % BoundProvider object (CorridorBoundProvider or FreeSpaceBoundProvider)
    end
    
    methods
        function obj = CACRCPlanner(varargin)
            if nargin > 0 && isa(varargin{1}, 'SimulationConfig')
                cfg = varargin{1};
                obj.dt = cfg.dt;
                obj.L = cfg.wheelbase;
                obj.vehicle_width = cfg.vehicle_width;
                obj.max_accel = cfg.a_max;
                obj.max_decel = cfg.a_min;
                obj.max_steering = cfg.delta_max;
                obj.qp_base_planner = QPMPCPlanner(cfg);
                obj.predictor = PredictorModule(cfg);
            else
                obj.qp_base_planner = QPMPCPlanner();
                obj.predictor = PredictorModule();
            end
        end
        
        function reset(obj)
            obj.last_u = [0; 0];
            obj.qp_base_planner.reset();
        end
        
        function [u_opt, pred_states, status, info] = plan(obj, world, reference_path, target_speed)
            % PLAN Evaluates Left vs Right topologies and returns optimal safe plan.
            
            t_plan_start = tic;
            ego = world.ego;
            x0 = [ego.x; ego.y; ego.theta; ego.v];
            
            % Standstill Hold Gate: If vehicle is stopped and target speed is ~0, return nominal zero control
            if ego.v <= 0.05 && target_speed <= 0.10
                u_opt = [0.0; 0.0];
                obj.last_u = u_opt;
                obj.qp_base_planner.last_u = u_opt;
                pred_states = repmat(x0', double(obj.N_p), 1);
                status = 1;
                info = struct();
                info.selected_topology = 'standstill_hold';
                info.is_soft = false;
                info.failure_reason = 'none';
                info.solve_time_ms = toc(t_plan_start) * 1000;
                info.ok_geom_left = true; info.ok_geom_right = true;
                info.ok_hard_left = true; info.ok_hard_right = true;
                info.ok_soft_left = true; info.ok_soft_right = true;
                info.J_left = 0; info.J_right = 0;
                return;
            end
            
            % 1. Predict Obstacle Trajectories & Uncertainty Ellipses
            preds = obj.predictor.predict(world);
            
            % 2. Extract Reference Trajectory
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
            X_ref = reshape(x_ref_traj', [], 1);
            
            % 3. Discrete Linearization (Reset last_u if vehicle stopped to prevent negative velocity linearization trap)
            if ego.v <= 0.5 && obj.last_u(2) < 0
                u_op = [0; 0];
            else
                u_op = obj.last_u;
            end
            [A_d, B_d, c_d] = obj.qp_base_planner.linearize_kinematics(x0, u_op);
            
            % 4. Build Condensed Matrices
            Np = double(obj.N_p); nx = 4; nu = 2;
            S_x = zeros(nx * Np, nx); S_u = zeros(nx * Np, nu * Np); S_c = zeros(nx * Np, 1);
            A_pow = eye(nx); c_accum = zeros(nx, 1);
            
            for k = 1:Np
                c_accum = c_accum + A_pow * c_d; A_pow = A_pow * A_d;
                idx_row = (k - 1) * nx + (1:nx);
                S_x(idx_row, :) = A_pow; S_c(idx_row) = c_accum;
                A_block = eye(nx);
                for j = k:-1:1
                    idx_col = (j - 1) * nu + (1:nu);
                    S_u(idx_row, idx_col) = A_block * B_d;
                    A_block = A_block * A_d;
                end
            end
            
            % Cost Function H_u, f_u
            Q_single = diag(obj.Q_diag); Q_bar = kron(eye(Np), Q_single);
            R_single = diag(obj.R_diag); R_bar = kron(eye(Np), R_single);
            R_rate_single = diag(obj.R_rate_diag);
            D_rate = eye(nu * Np);
            for k = 2:Np
                idx_curr = (k - 1) * nu + (1:nu); idx_prev = (k - 2) * nu + (1:nu);
                D_rate(idx_curr, idx_prev) = -eye(nu);
            end
            R_rate_bar = D_rate' * kron(eye(Np), R_rate_single) * D_rate;
            
            H_u = S_u' * Q_bar * S_u + R_bar + R_rate_bar;
            H_u = (H_u + H_u') / 2;
            
            X_free = S_x * x0 + S_c;
            
            % Smooth Gaussian reference corridor shaping for topological overtaking & re-centering
            r_ego = obj.vehicle_width / 2; beta = 2.0;
            X_ref_left = X_ref; X_ref_right = X_ref;
            
            % Base Road Bounds (via BoundProvider abstraction)
            if isempty(obj.bound_provider)
                obj.bound_provider = CorridorBoundProvider();
            end
            [y_min_vec, y_max_vec] = obj.bound_provider.getBounds(world, X_ref, Np, obj.vehicle_width);
            
            % Build dual topology reference targets
            X_ref_left = X_ref;
            X_ref_right = X_ref;
            
            r_ego = obj.vehicle_width / 2;
            beta = 2.0;
            if world.n_static_obs > 0
                for obs_i = 1:world.n_static_obs
                    obs_x = world.static_obs(obs_i, 1);
                    if (obs_x - ego.x) > -3.0 && (obs_x - ego.x) < 25.0
                        obs_y = world.static_obs(obs_i, 2);
                        if obs_x < -10.0, continue; end % Skip disabled/out-of-bounds obstacles
                        r_obs = preds.radius(obs_i);
                        sigma_y_max = preds.sigma_y(obs_i, Np);
                        d_safe_max = r_ego + r_obs + beta * sigma_y_max + 0.10;
                        
                        for k_ref = 1:Np
                            px_ref = X_ref((k_ref - 1) * nx + 1);
                            dx_ref = px_ref - obs_x;
                            if dx_ref <= 0
                                smooth_weight = exp(- (dx_ref / 12.0)^2); % 12m pre-obstacle smooth transition scale
                            elseif dx_ref <= 5.0
                                smooth_weight = 1.0; % Hold flat target level while passing obstacle
                            else
                                smooth_weight = exp(- ((dx_ref - 5.0) / 6.0)^2); % 6m return decay scale
                            end
                            idx_y = (k_ref - 1) * nx + 2;
                            
                            y_target_left = X_ref(idx_y) + (obs_y + d_safe_max + 0.15 - X_ref(idx_y)) * smooth_weight;
                            y_target_right = X_ref(idx_y) + (obs_y - d_safe_max - 0.15 - X_ref(idx_y)) * smooth_weight;
                            
                            % Clamp topology reference targets to safe drivable bounds
                            y_target_left = min(y_max_vec(k_ref) - 0.05, y_target_left);
                            y_target_right = max(y_min_vec(k_ref) + 0.05, y_target_right);
                            
                            if y_target_left > X_ref_left(idx_y)
                                X_ref_left(idx_y) = y_target_left;
                            end
                            if y_target_right < X_ref_right(idx_y)
                                X_ref_right(idx_y) = y_target_right;
                            end
                        end
                    end
                end
            end
            
            
            f_u_left = S_u' * Q_bar * (X_free - X_ref_left);
            f_u_right = S_u' * Q_bar * (X_free - X_ref_right);
            rate_offset = zeros(nu * Np, 1); rate_offset(1:nu) = -R_rate_single * u_op;
            f_u_left = f_u_left + rate_offset;
            f_u_right = f_u_right + rate_offset;
            
            lb_u = zeros(nu * Np, 1); ub_u = zeros(nu * Np, 1);
            for k = 1:Np
                idx_u = (k - 1) * nu + (1:nu);
                lb_u(idx_u(1)) = -obj.max_steering; ub_u(idx_u(1)) = obj.max_steering;
                lb_u(idx_u(2)) = obj.max_decel;     ub_u(idx_u(2)) = obj.max_accel;
            end
            
            y_min_safe = min(y_min_vec);
            y_max_safe = max(y_max_vec);
            
            A_base = []; b_base = [];
            if ego.v > 0.5
                theta_slack = max(0.15, min(0.25, abs(x0(3)) + 0.05)); % Bound heading deviation to <=0.25 rad to maintain bounding box compliance
            else
                theta_slack = max(1.57, abs(x0(3)) + 0.50); % Relaxed heading slack at standstill
            end
            for k = 1:Np
                row_y_S_u = S_u((k - 1) * nx + 2, :);
                y_free_k = X_free((k - 1) * nx + 2);
                A_base = [A_base; row_y_S_u]; b_base = [b_base; y_max_vec(k) - y_free_k];
                A_base = [A_base; -row_y_S_u]; b_base = [b_base; -y_min_vec(k) + y_free_k];
                
                % Heading constraint: |theta(k) - theta_ref(k)| <= theta_slack
                % Expressed as: row_th * U <= theta_slack - (th_free_k - theta_ref_k)
                theta_ref_k = X_ref((k - 1) * nx + 3);
                row_th_S_u = S_u((k - 1) * nx + 3, :);
                th_free_k = X_free((k - 1) * nx + 3);
                th_err_free = th_free_k - theta_ref_k;
                A_base = [A_base; row_th_S_u]; b_base = [b_base; theta_slack - th_err_free];
                A_base = [A_base; -row_th_S_u]; b_base = [b_base; theta_slack + th_err_free];
            end
            
            % 5. Analytical Pre-QP Geometric Feasibility Classification Check
            r_ego = obj.vehicle_width / 2;
            beta = 2.0;
            ok_geom_left = true;
            ok_geom_right = true;
            
            % If perception map detects genuine dynamic blockage (corridor collapsed: y_min > y_max),
            % hard-gate geometric feasibility to prevent soft-QP from bypassing blockage.
            if any(y_min_vec > y_max_vec)
                ok_geom_left = false;
                ok_geom_right = false;
            end
            
            if world.n_static_obs > 0
                for obs_i = 1:world.n_static_obs
                    obs_x = world.static_obs(obs_i, 1);
                    if obs_x < -10.0 || obs_x < (ego.x - 3.0) || (obs_x - ego.x) > 25.0
                        continue;
                    end
                    obs_y = world.static_obs(obs_i, 2);
                    r_obs = preds.radius(obs_i);
                    sigma_y_max = preds.sigma_y(obs_i, Np);
                    d_safe_max = r_ego + r_obs + beta * sigma_y_max + 0.10;
                    
                    % Check if passing on right is off-road
                    if (obs_y - d_safe_max - 0.15) < (y_min_safe + 0.10)
                        ok_geom_right = false;
                    end
                    % Check if passing on left is off-road
                    if (obs_y + d_safe_max + 0.15) > (y_max_safe - 0.10)
                        ok_geom_left = false;
                    end
                end
            end
            
            info.ok_geom_left = ok_geom_left;
            info.ok_geom_right = ok_geom_right;
            info.y_min_vec = y_min_vec;
            info.y_max_vec = y_max_vec;
            info.min_delta_y = min(y_max_vec - y_min_vec);
            info.crossovers = sum(y_max_vec < y_min_vec);
            info.geom_rejected = (~ok_geom_left && ~ok_geom_right);
            
            % Evaluate Topology 1: LEFT PASSING (y >= obs_y + d_safe)
            if ok_geom_left
                [A_left, b_left] = obj.build_obstacle_constraints(world, preds, S_u, X_free, x0, 'left', y_min_safe, y_max_safe);
                if isempty(A_left)
                    % No obstacle constraints — pure road-bound QP
                    [U_left, ~, res_left, ok_left] = obj.qp_base_planner.hildreth_dual_qp(H_u, f_u_left, A_base, b_base, lb_u, ub_u);
                else
                    [U_left, ok_left] = obj.solve_softened_qp(H_u, f_u_left, A_base, b_base, A_left, b_left, lb_u, ub_u);
                end
                J_left = 0.5 * U_left' * H_u * U_left + f_u_left' * U_left;
            else
                A_left = []; b_left = []; U_left = zeros(nu * Np, 1); ok_left = false; J_left = inf;
            end
            
            % Evaluate Topology 2: RIGHT PASSING (y <= obs_y - d_safe)
            if ok_geom_right
                [A_right, b_right] = obj.build_obstacle_constraints(world, preds, S_u, X_free, x0, 'right', y_min_safe, y_max_safe);
                if isempty(A_right)
                    [U_right, ~, res_right, ok_right] = obj.qp_base_planner.hildreth_dual_qp(H_u, f_u_right, A_base, b_base, lb_u, ub_u);
                else
                    [U_right, ok_right] = obj.solve_softened_qp(H_u, f_u_right, A_base, b_base, A_right, b_right, lb_u, ub_u);
                end
                J_right = 0.5 * U_right' * H_u * U_right + f_u_right' * U_right;
            else
                A_right = []; b_right = []; U_right = zeros(nu * Np, 1); ok_right = false; J_right = inf;
            end
            
            info.ok_hard_left = ok_left; info.ok_hard_right = ok_right;
            info.J_left = J_left; info.J_right = J_right;
            
            % 6. Topological Selection with Nearest Obstacle Memory & Clearance Reset
            nearest_obs_i = -1;
            min_dist_ahead = inf;
            if world.n_static_obs > 0
                for obs_i = 1:world.n_static_obs
                    obs_x = world.static_obs(obs_i, 1);
                    if obs_x < -10.0 || obs_x < (ego.x - 5.0), continue; end % Skip cleared obstacles (x < ego.x - 5m)
                    dist_ahead = obs_x - ego.x;
                    if dist_ahead < min_dist_ahead
                        min_dist_ahead = dist_ahead;
                        nearest_obs_i = obs_i;
                    end
                end
            end
            if nearest_obs_i > 0 && min_dist_ahead <= 20.0
                obs_y = world.static_obs(nearest_obs_i, 2);
                if strcmp(obj.locked_side, 'none')
                    if ok_geom_left && ok_geom_right
                        if J_left <= J_right
                            obj.locked_side = 'left';
                        else
                            obj.locked_side = 'right';
                        end
                    elseif ok_geom_left
                        obj.locked_side = 'left';
                    elseif ok_geom_right
                        obj.locked_side = 'right';
                    else
                        if ego.y >= obs_y
                            obj.locked_side = 'left';
                        else
                            obj.locked_side = 'right';
                        end
                    end
                end
            else
                obj.locked_side = 'none';
            end
            
            status = 0; U_opt = zeros(nu * Np, 1); selected_topology = 'none'; is_soft = false; failure_reason = 'mpc_infeasible';
            if strcmp(obj.locked_side, 'left')
                if ok_left
                    selected_topology = 'hard_left'; U_opt = U_left; status = 1; is_soft = false; failure_reason = 'none';
                else
                    [U_soft_left, ok_soft_left] = obj.solve_softened_qp(H_u, f_u_left, A_base, b_base, A_left, b_left, lb_u, ub_u);
                    if ok_soft_left
                        selected_topology = 'soft_left'; U_opt = U_soft_left; status = 1; is_soft = true; failure_reason = 'none';
                    end
                end
            elseif strcmp(obj.locked_side, 'right')
                if ok_right
                    selected_topology = 'hard_right'; U_opt = U_right; status = 1; is_soft = false; failure_reason = 'none';
                else
                    [U_soft_right, ok_soft_right] = obj.solve_softened_qp(H_u, f_u_right, A_base, b_base, A_right, b_right, lb_u, ub_u);
                    if ok_soft_right
                        selected_topology = 'soft_right'; U_opt = U_soft_right; status = 1; is_soft = true; failure_reason = 'none';
                    end
                end
            end
            

            
            if status == 0
                if strcmp(obj.locked_side, 'none')
                    if ok_left && ok_right
                        if J_left <= J_right
                            selected_topology = 'hard_left'; U_opt = U_left; status = 1; is_soft = false; failure_reason = 'none';
                        else
                            selected_topology = 'hard_right'; U_opt = U_right; status = 1; is_soft = false; failure_reason = 'none';
                        end
                    elseif ok_left
                        selected_topology = 'hard_left'; U_opt = U_left; status = 1; is_soft = false; failure_reason = 'none';
                    elseif ok_right
                        selected_topology = 'hard_right'; U_opt = U_right; status = 1; is_soft = false; failure_reason = 'none';
                    end
                end
            end
            
            if status == 0
                % LAYER 1 FALLBACK: Dual Softened Slack QP (evaluate unlocked or soft side)
                if ok_geom_left && (strcmp(obj.locked_side, 'none') || strcmp(obj.locked_side, 'left'))
                    [U_soft_left, ok_soft_left] = obj.solve_softened_qp(H_u, f_u_left, A_base, b_base, A_left, b_left, lb_u, ub_u);
                    J_soft_left = 0.5 * U_soft_left' * H_u * U_soft_left + f_u_left' * U_soft_left;
                else
                    U_soft_left = zeros(nu * Np, 1); ok_soft_left = false; J_soft_left = inf;
                end
                
                if ok_geom_right && (strcmp(obj.locked_side, 'none') || strcmp(obj.locked_side, 'right'))
                    [U_soft_right, ok_soft_right] = obj.solve_softened_qp(H_u, f_u_right, A_base, b_base, A_right, b_right, lb_u, ub_u);
                    J_soft_right = 0.5 * U_soft_right' * H_u * U_soft_right + f_u_right' * U_soft_right;
                else
                    U_soft_right = zeros(nu * Np, 1); ok_soft_right = false; J_soft_right = inf;
                end
                
                info.ok_soft_left = ok_soft_left; info.ok_soft_right = ok_soft_right;
                
                if ok_soft_left && ok_soft_right
                    if J_soft_left <= J_soft_right
                        selected_topology = 'soft_left'; U_opt = U_soft_left; status = 1; is_soft = true; failure_reason = 'none';
                    else
                        selected_topology = 'soft_right'; U_opt = U_soft_right; status = 1; is_soft = true; failure_reason = 'none';
                    end
                elseif ok_soft_left
                    selected_topology = 'soft_left'; U_opt = U_soft_left; status = 1; is_soft = true; failure_reason = 'none';
                elseif ok_soft_right
                    selected_topology = 'soft_right'; U_opt = U_soft_right; status = 1; is_soft = true; failure_reason = 'none';
                else
                    selected_topology = 'none'; status = 0; U_opt = zeros(nu * Np, 1); is_soft = false;
                    
                    % 3-Level Failure Reason Classification
                    if ~ok_geom_left && ~ok_geom_right
                        failure_reason = 'geometric_infeasibility';
                    elseif (ok_geom_left || ok_geom_right) && (~ok_soft_left && ~ok_soft_right)
                        failure_reason = 'reachability_infeasibility';
                    else
                        failure_reason = 'optimization_infeasibility';
                    end
                end
            end
            
            info.selected_topology = selected_topology;
            info.is_soft = is_soft;
            info.failure_reason = failure_reason;
            info.solve_time_ms = toc(t_plan_start) * 1000;
            
            if status == 1
                u_opt = U_opt(1:nu);
                X_pred = S_x * x0 + S_u * U_opt + S_c;
                pred_states = reshape(X_pred, nx, Np)';
                obj.last_u = u_opt;
            else
                % Emergency road-bound recovery fallback:
                % Rebuild A_base with relaxed lower bound = min(y_min_vec(k), y_free_k - 0.10)
                % to let QP recover from a state already outside the safety inset.
                A_base_relax = []; b_base_relax = [];
                for k_r = 1:Np
                    row_y = S_u((k_r - 1) * nx + 2, :);
                    y_free_k_r = X_free((k_r - 1) * nx + 2);
                    % Upper bound always hard
                    A_base_relax = [A_base_relax; row_y];
                    b_base_relax = [b_base_relax; y_max_vec(k_r) - y_free_k_r];
                    % Lower bound: relax to free position if below inset
                    y_lo_relax = min(y_min_vec(k_r), y_free_k_r - 0.10);
                    A_base_relax = [A_base_relax; -row_y];
                    b_base_relax = [b_base_relax; -y_lo_relax + y_free_k_r];
                    row_th = S_u((k_r - 1) * nx + 3, :);
                    th_free_k_r = X_free((k_r - 1) * nx + 3);
                    th_ref_k_r = X_ref((k_r - 1) * nx + 3);
                    th_err = th_free_k_r - th_ref_k_r;
                    A_base_relax = [A_base_relax; row_th];
                    b_base_relax = [b_base_relax; theta_slack - th_err];
                    A_base_relax = [A_base_relax; -row_th];
                    b_base_relax = [b_base_relax; theta_slack + th_err];
                end
                % Use left-pass cost (steer toward road center)
                [U_recover, ~, ~, ok_recover] = obj.qp_base_planner.hildreth_dual_qp(H_u, f_u_left, A_base_relax, b_base_relax, lb_u, ub_u);
                if ok_recover
                    u_opt = U_recover(1:nu);
                    obj.last_u = u_opt;
                else
                    u_opt = [0; obj.max_decel];
                end
                pred_states = repmat(x0', Np, 1);
            end
        end
        
        function [A_obs, b_obs] = build_obstacle_constraints(obj, world, preds, S_u, X_free, x0, pass_side, y_min, y_max)
            A_obs = []; b_obs = [];
            if world.n_static_obs == 0, return; end
            
            Np = double(obj.N_p); nx = 4;
            r_ego = obj.vehicle_width / 2;
            beta = 2.0; % 95.4% confidence multiplier
            
            for i = 1:world.n_static_obs
                r_obs = preds.radius(i);
                obs_x = world.static_obs(i, 1);
                obs_y = world.static_obs(i, 2);
                if obs_x < -10.0, continue; end % Skip disabled/out-of-bounds obstacles
                
                for k = 1:Np
                    sigma_y_k = preds.sigma_y(i, k);
                    d_safe_k = r_ego + r_obs + beta * sigma_y_k + 0.10;
                    
                    row_y_S_u = S_u((k - 1) * nx + 2, :);
                    y_free_k = X_free((k - 1) * nx + 2);
                    
                    % Geometry-derived longitudinal overlap distance: (L_ego + L_obs)/2 + d_x_margin
                    L_ego = obj.L + 1.8; % 4.5m total length
                    L_obs = 2.0 * r_obs;
                    d_x_margin = 0.50;
                    d_x_overlap = (L_ego + L_obs) / 2 + d_x_margin;
                    
                    % Prediction longitudinal overlap check: active strictly for prediction step k when |px_k - obs_x| <= d_x_overlap
                    px_k = X_free((k - 1) * nx + 1);
                    if abs(px_k - obs_x) <= d_x_overlap
                        if strcmp(pass_side, 'left')
                            % Pass Left: y_k >= obs_y + d_safe_k  =>  -y_k <= -(obs_y + d_safe_k)
                            A_obs = [A_obs; -row_y_S_u];
                            b_obs = [b_obs; -(obs_y + d_safe_k) + y_free_k];
                        else
                            % Pass Right: y_k <= obs_y - d_safe_k
                            A_obs = [A_obs; row_y_S_u];
                            b_obs = [b_obs; (obs_y - d_safe_k) - y_free_k];
                        end
                    end
                end
            end
        end
        
        function [U_soft, ok_soft] = solve_softened_qp(obj, H, f, A_base, b_base, A_obs, b_obs, lb, ub)
            % SOLVE_SOFTENED_QP Layer 1 Slack-Softened QP Fallback
            % Uses dual slack variables:
            %   eps_obs  : slack for obstacle clearance (penalty W_slack = 1000)
            %   eps_base : slack for road bounds (penalty W_bound = 10000.0)
            n = length(f);
            m_base = length(b_base);
            m_obs = length(b_obs);
            
            W_bound = 10000.0;
            H_aug = blkdiag(H, obj.W_slack, W_bound);
            f_aug = [f; 0.0; 0.0];
            
            A_aug = [A_base, zeros(m_base, 1), -ones(m_base, 1); ...
                     A_obs,  -ones(m_obs, 1),  zeros(m_obs, 1)];
            b_aug = [b_base; b_obs];
            
            lb_aug = [lb; 0.0; 0.0];
            ub_aug = [ub; 50.0; 50.0];
            
            [U_aug, ~, ~, ok_aug] = obj.qp_base_planner.hildreth_dual_qp(H_aug, f_aug, A_aug, b_aug, lb_aug, ub_aug);
            if ok_aug
                U_soft = U_aug(1:n);
                ok_soft = true;
            else
                U_soft = zeros(n, 1);
                ok_soft = false;
            end
        end
    end
end
