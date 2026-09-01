function [passed, metrics, history] = stage5_multivehicle_coordination(varargin)
    % STAGE5_MULTIVEHICLE_COORDINATION Stage 5 Multi-Vehicle Interaction Audit
    %
    % Evaluates Stage 5 Coordination Controller (Hierarchical Decision Layer + Frozen Stage 4 CA-CRC)
    % across dynamic multi-vehicle interaction scenarios.
    
    p = inputParser;
    addParameter(p, 'scenario', 'multi_vehicle_yield_overtake', @ischar);
    addParameter(p, 'verbose', true, @islogical);
    addParameter(p, 'max_steps', 150, @isnumeric);
    addParameter(p, 'ego_v', 5.0, @isnumeric);
    addParameter(p, 'gap_offset', 0.0, @isnumeric);
    addParameter(p, 'obs_model', [], @(x) isempty(x) || isa(x, 'ObservationModel'));
    addParameter(p, 'actuator_model', [], @(x) isempty(x) || isa(x, 'ActuatorUncertaintyModel'));
    addParameter(p, 'uncertainty_mode', '', @ischar);
    addParameter(p, 'seed', 42, @isnumeric);
    parse(p, varargin{:});
    
    scenario_name = p.Results.scenario;
    verbose = p.Results.verbose;
    N_steps = p.Results.max_steps;
    v_init_ego = p.Results.ego_v;
    gap_offset = p.Results.gap_offset;
    obs_model = p.Results.obs_model;
    actuator_model = p.Results.actuator_model;
    unc_mode = lower(p.Results.uncertainty_mode);
    sim_seed = p.Results.seed;
    
    if ~isempty(unc_mode)
        switch unc_mode
            case 'ideal'
                obs_model = ObservationModel('ideal', sim_seed);
                actuator_model = ActuatorUncertaintyModel('ideal');
            case {'nominal_perception', 'nominal'}
                obs_model = ObservationModel('nominal', sim_seed);
                actuator_model = ActuatorUncertaintyModel('ideal');
            case {'delayed_perception', 'delayed'}
                obs_model = ObservationModel('delayed', sim_seed);
                actuator_model = ActuatorUncertaintyModel('ideal');
            case 'steering_bias'
                obs_model = ObservationModel('ideal', sim_seed);
                actuator_model = ActuatorUncertaintyModel('steering_bias');
            case {'combined_realistic', 'combined'}
                obs_model = ObservationModel('combined_realistic', sim_seed);
                actuator_model = ActuatorUncertaintyModel('steering_bias');
            otherwise
                error('Unknown uncertainty_mode: %s', unc_mode);
        end
    end
    
    if isempty(obs_model)
        obs_model = ObservationModel('ideal', sim_seed);
    end
    if isempty(actuator_model)
        actuator_model = ActuatorUncertaintyModel('ideal');
    end
    obs_model.reset();
    actuator_model.reset();
    
    addpath('planning', 'config', 'vehicle', 'core', 'environment', 'stages');
    
    % Initialize Config & Environment
    config = SimulationConfig();
    world = ScenarioDefinitions(scenario_name, config);
    vehicle = BicycleModel(config);
    
    % Apply gap perturbation if requested
    if gap_offset ~= 0.0
        if isprop(world, 'static_obs') && ~isempty(world.static_obs)
            world.static_obs(:, 1) = world.static_obs(:, 1) + gap_offset;
        end
        if isprop(world, 'dynamic_agents') && ~isempty(world.dynamic_agents)
            for a_i = 1:length(world.dynamic_agents)
                world.dynamic_agents(a_i).x = world.dynamic_agents(a_i).x + gap_offset;
            end
        end
    end
    
    % Create Stage 5 Controller
    ctrl5 = Stage5CoordinationController(config);
    ctrl5.reset();
    % Following is a longitudinal-car-following validation, not a passing
    % benchmark. Disable only the Stage 5 macro-intent authority here.
    ctrl5.setOvertakeEnabled(~strcmp(scenario_name, 'multi_vehicle_following'));
    
    world.ego.v = v_init_ego;
    dt = config.dt;
    v_target_nominal = 8.0;
    
    % Generate Nominal Center Reference Path
    N_path = 500;
    ref_path = zeros(N_path, 5);
    ref_path(:, 1) = linspace(0, 150, N_path)';
    ref_path(:, 2) = world.ego.y;
    ref_path(:, 4) = 0.0;
    ref_path(:, 5) = v_target_nominal;
    
    if verbose
        fprintf('\n');
        fprintf('╔════════════════════════════════════════════════════════╗\n');
        fprintf('║  STAGE 5: MULTI-VEHICLE INTERACTION & COORDINATION    ║\n');
        fprintf('║                                                        ║\n');
        fprintf('║  Testing: Intent Selection + Frozen Stage 4 CA-CRC     ║\n');
        fprintf('╚════════════════════════════════════════════════════════╝\n\n');
        fprintf('[SCENARIO] Running: %s (Perception Mode: %s, Actuator Mode: %s)\n', ...
            scenario_name, obs_model.mode, actuator_model.mode);
        fprintf('[SIMULATION] Starting %d steps (%.1f seconds at dt=%.2f s)\n\n', ...
            N_steps, N_steps * dt, dt);
    end
    
    % Data Tracking Arrays
    history.t = zeros(N_steps, 1);
    history.ego_x = zeros(N_steps, 1); history.ego_y = zeros(N_steps, 1);
    history.ego_v = zeros(N_steps, 1); history.ego_theta = zeros(N_steps, 1);
    history.min_clearance = zeros(N_steps, 1); history.is_collision = false(N_steps, 1);
    history.inside_bounds = true(N_steps, 1); history.solver_status = zeros(N_steps, 1);
    history.solve_time_ms = zeros(N_steps, 1); history.filter_active = false(N_steps, 1);
    history.macro_intent = cell(N_steps, 1);
    history.min_ttc = zeros(N_steps, 1);
    history.filter_reason = cell(N_steps, 1);
    history.failure_reason = cell(N_steps, 1);
    history.selected_topology = cell(N_steps, 1);
    
    % Perception Telemetry Arrays
    history.perception_e_pos = zeros(N_steps, 1);
    history.perception_e_vel = zeros(N_steps, 1);
    history.perception_e_heading = zeros(N_steps, 1);
    history.perception_age = zeros(N_steps, 1);
    
    % Actuator Telemetry Arrays
    history.delta_cmd = zeros(N_steps, 1);
    history.delta_actual_cmd = zeros(N_steps, 1);
    history.delta_plant = zeros(N_steps, 1);
    history.a_cmd = zeros(N_steps, 1);
    history.a_actual_cmd = zeros(N_steps, 1);
    history.a_plant = zeros(N_steps, 1);
    history.steering_error = zeros(N_steps, 1);
    history.acceleration_error = zeros(N_steps, 1);
    
    hard_qp_count = 0; soft_qp_count = 0; safety_rejected_count = 0; emergency_braking_count = 0;
    
    intent_counts.MAINTAIN = 0;
    intent_counts.FOLLOW = 0;
    intent_counts.YIELD = 0;
    intent_counts.OVERTAKE = 0;
    
    for k = 1:N_steps
        t = (k - 1) * dt;
        
        % Dynamic scenario braking event at t = 3.0s for multi_vehicle_following
        if strcmp(scenario_name, 'multi_vehicle_following') && t >= 3.0 && ~isempty(world.dynamic_agents)
            world.dynamic_agents(1).v_target = 2.0;
        end
        
        % Update Dynamic Agents in Ground-Truth World
        world = world.stepAgents(dt);
        
        % Pass Ground-Truth World through Perception Observation Pipeline
        [obs_world, obs_structs] = obs_model.observe(world, dt);
        
        % Execute Stage 5 Control Step using Observed World State
        [u_cmd, pred_states, status, info] = ctrl5.step(obs_world, ref_path, v_target_nominal);
        
        % Track Perception Telemetry
        if world.n_agents > 0 && ~isempty(obs_structs) && obs_structs(1).agent_id > 0
            true_ag = world.agents(1);
            obs_s = obs_structs(1);
            history.perception_e_pos(k) = hypot(true_ag.x - obs_s.x, true_ag.y - obs_s.y);
            history.perception_e_vel(k) = abs(hypot(true_ag.vx, true_ag.vy) - obs_s.velocity);
            true_th = atan2(true_ag.vy, true_ag.vx + 1e-6);
            history.perception_e_heading(k) = abs(atan2(sin(true_th - obs_s.heading), cos(true_th - obs_s.heading)));
            history.perception_age(k) = obs_s.age;
        end
        
        % Track Control Outcome
        if status == 1
            if info.filter_active
                safety_rejected_count = safety_rejected_count + 1;
            elseif info.is_soft
                soft_qp_count = soft_qp_count + 1;
            else
                hard_qp_count = hard_qp_count + 1;
            end
        else
            emergency_braking_count = emergency_braking_count + 1;
        end
        
        % Process Command through Actuator Uncertainty Layer
        [u_actual, act_info] = actuator_model.process(u_cmd, config);
        delta_actual_cmd = u_actual(1);
        a_actual_cmd = u_actual(2);
        
        % Record Actuator Telemetry
        history.delta_cmd(k) = u_cmd(1);
        history.delta_actual_cmd(k) = delta_actual_cmd;
        history.a_cmd(k) = u_cmd(2);
        history.a_actual_cmd(k) = a_actual_cmd;
        history.steering_error(k) = act_info.steering_error;
        history.acceleration_error(k) = act_info.acceleration_error;
        
        % Closed-Loop Dynamics Step for Ego Vehicle Plant
        world.ego = vehicle.stepKinematic(world.ego, a_actual_cmd, delta_actual_cmd, dt);
        history.delta_plant(k) = world.ego.delta;
        history.a_plant(k) = world.ego.a;
        
        % Record Ground-Truth Telemetry & TTC Computation
        clearance = world.getMinClearance(config);
        is_coll = world.checkCollision(config);
        in_bounds = world.isEgoInBounds(config);
        
        % Ground-truth Vector TTC computation considering velocity direction
        min_ttc_val = Inf;
        if world.n_agents > 0
            for a_i = 1:world.n_agents
                ag = world.agents(a_i);
                if ag.x < -50 || ag.y < -50, continue; end
                dx_center = ag.x - world.ego.x;
                dy_center = abs(ag.y - world.ego.y);
                
                % Check lateral conflict zone (width span + 1.0m)
                if dy_center <= (config.vehicle_width / 2 + ag.width / 2 + 1.0)
                    dx_gap = abs(dx_center) - (config.vehicle_length / 2 + ag.length / 2);
                    if dx_gap <= 0
                        min_ttc_val = 0.0;
                    else
                        v_ego_x = world.ego.v * cos(world.ego.theta);
                        v_ag_x = ag.vx;
                        if dx_center > 0
                            v_closing = v_ego_x - v_ag_x;
                        else
                            v_closing = v_ag_x - v_ego_x;
                        end
                        if v_closing > 1e-3
                            ttc_i = dx_gap / v_closing;
                            if ttc_i < min_ttc_val, min_ttc_val = ttc_i; end
                        end
                    end
                end
            end
        end
        
        % Compute Active Reference Lateral Target for Tracking RMSE
        y_nominal = ref_path(1, 2);
        ref_y_active = y_nominal;
        if strcmp(info.macro_intent, 'OVERTAKE') && ctrl5.x_overtake_start > 0
            L_lc = 12.0;
            y_start_ov = ctrl5.y_overtake_start;
            if y_start_ov < 0, y_start_ov = y_nominal; end
            target_y_ov = y_nominal + 1.55;
            dy_ov = target_y_ov - y_start_ov;
            s_norm = min(1.0, max(0.0, (world.ego.x - ctrl5.x_overtake_start) / L_lc));
            smooth_s = 3.0 * s_norm^2 - 2.0 * s_norm^3;
            ref_y_active = y_start_ov + dy_ov * smooth_s;
        elseif ctrl5.x_recenter_start > 0
            L_lc_recenter = max(25.0, 3.5 * world.ego.v);
            y_start = ctrl5.y_recenter_start;
            if y_start < 0, y_start = y_nominal + 1.55; end
            dy_total = y_nominal - y_start;
            s_norm = min(1.0, max(0.0, (world.ego.x - ctrl5.x_recenter_start) / L_lc_recenter));
            smooth_s = 3.0 * s_norm^2 - 2.0 * s_norm^3;
            ref_y_active = y_start + dy_total * smooth_s;
        end
        
        history.t(k) = t;
        history.ego_x(k) = world.ego.x; history.ego_y(k) = world.ego.y;
        history.ref_y(k) = ref_y_active;
        history.ego_v(k) = world.ego.v; history.ego_theta(k) = world.ego.theta;
        history.min_clearance(k) = clearance;
        history.is_collision(k) = is_coll;
        history.inside_bounds(k) = in_bounds;
        history.solver_status(k) = status;
        history.solve_time_ms(k) = info.solve_time_ms;
        history.filter_active(k) = info.filter_active;
        history.macro_intent{k} = info.macro_intent;
        history.min_ttc(k) = min_ttc_val;
        history.ttc(k) = min_ttc_val;
        history.filter_reason{k} = info.filter_reason;
        history.failure_reason{k} = info.failure_reason;
        history.selected_topology{k} = info.selected_topology;
        
        % Legacy radial collision check for forensic comparison
        r_ego_old = config.vehicle_width / 2;
        old_rad_clr = inf;
        for i_s = 1:world.n_static_obs
            obs_s = world.static_obs(i_s, :);
            if obs_s(1) > -50 && obs_s(3) > 0
                d_c = norm([world.ego.x - obs_s(1), world.ego.y - obs_s(2)]);
                r_c = d_c - r_ego_old - (obs_s(4)/2);
                if r_c < old_rad_clr, old_rad_clr = r_c; end
            end
        end
        for i_a = 1:world.n_agents
            ag_a = world.agents(i_a);
            if ag_a.x > -50
                d_c = norm([world.ego.x - ag_a.x, world.ego.y - ag_a.y]);
                r_c = d_c - r_ego_old - (ag_a.width/2 + 0.1);
                if r_c < old_rad_clr, old_rad_clr = r_c; end
            end
        end
        history.is_collision_old(k) = (old_rad_clr <= 0.0);
        
        % Increment macro-intent counters
        if isfield(intent_counts, info.macro_intent)
            intent_counts.(info.macro_intent) = intent_counts.(info.macro_intent) + 1;
        end
    end
    
    % Summary Metrics Calculation
    dist_traveled = world.ego.x - 10.0;
    final_v = world.ego.v;
    collision_steps = sum(history.is_collision);
    bounds_steps = sum(history.inside_bounds);
    min_clr = min(history.min_clearance);
    mean_solve_time = mean(history.solve_time_ms);
    ego_y_vec = history.ego_y(:);
    ref_y_vec = history.ref_y(:);
    lat_rmse = sqrt(mean((ego_y_vec - ref_y_vec).^2));
    
    metrics.dist_traveled = dist_traveled;
    metrics.final_v = final_v;
    metrics.collision_steps = collision_steps;
    metrics.bounds_steps = bounds_steps;
    metrics.min_clr = min_clr;
    metrics.hard_qp_count = hard_qp_count;
    metrics.soft_qp_count = soft_qp_count;
    metrics.safety_rejected_count = safety_rejected_count;
    metrics.emergency_braking_count = emergency_braking_count;
    metrics.mean_solve_time = mean_solve_time;
    metrics.intent_counts = intent_counts;
    metrics.lat_rmse = lat_rmse;
    
    % Perception Metrics
    metrics.pos_rmse = sqrt(mean(history.perception_e_pos.^2));
    metrics.vel_rmse = sqrt(mean(history.perception_e_vel.^2));
    metrics.heading_rmse = sqrt(mean(history.perception_e_heading.^2));
    metrics.max_pos_err = max(history.perception_e_pos);
    metrics.max_vel_err = max(history.perception_e_vel);
    metrics.max_heading_err = max(history.perception_e_heading);
    metrics.mean_obs_age = mean(history.perception_age);
    
    % Actuator Metrics
    metrics.rms_steering_error = sqrt(mean(history.steering_error.^2));
    metrics.max_steering_error = max(abs(history.steering_error));
    metrics.rms_acceleration_error = sqrt(mean(history.acceleration_error.^2));
    metrics.max_acceleration_error = max(abs(history.acceleration_error));
    
    % Refined Outcome Classification Precedence Rules for Phase 12E
    qp_infeas_steps = sum(history.solver_status == 0);
    if collision_steps > 0
        outcome = 'COLLISION';
    elseif bounds_steps < N_steps
        outcome = 'UNSAFE_FAILURE';
    elseif qp_infeas_steps > 5 && final_v >= 0.10
        outcome = 'PLANNER_INFEASIBLE';
    elseif final_v < 0.10 && min_clr > 0.0
        outcome = 'SAFE_STOP';
    elseif lat_rmse > 0.50 || emergency_braking_count > 0 || safety_rejected_count > 0
        outcome = 'DEGRADED_SAFE';
    else
        outcome = 'SUCCESS';
    end
    metrics.outcome = outcome;
    
    passed = (collision_steps == 0) && ...
             (bounds_steps == N_steps) && ...
             (min_clr > 0.0) && ...
             (emergency_braking_count == 0);
    
    if verbose
        fprintf('════════════════════════════════════════════════════════════════════════════════════════\n');
        fprintf('                           STAGE 5 AUDIT RESULTS SUMMARY                                \n');
        fprintf('════════════════════════════════════════════════════════════════════════════════════════\n');
        fprintf('  Distance Traveled:      %6.2f m\n', dist_traveled);
        fprintf('  Final Speed:           %6.2f m/s\n', final_v);
        fprintf('  Min Clearance:         %6.2f m\n', min_clr);
        fprintf('  Collision Steps:       %6d\n', collision_steps);
        fprintf('  Road Bounds Compliance:%6d / %d steps\n', bounds_steps, N_steps);
        fprintf('  Macro-Intent Breakdown:\n');
        fprintf('     - MAINTAIN:         %6d steps\n', intent_counts.MAINTAIN);
        fprintf('     - FOLLOW:           %6d steps\n', intent_counts.FOLLOW);
        fprintf('     - YIELD:            %6d steps\n', intent_counts.YIELD);
        fprintf('     - OVERTAKE:         %6d steps\n', intent_counts.OVERTAKE);
        fprintf('  Control Execution State:\n');
        fprintf('     - Hard QP Feasible: %6d steps\n', hard_qp_count);
        fprintf('     - Soft QP Fallback: %6d steps\n', soft_qp_count);
        fprintf('     - Safety Rejections:%6d steps\n', safety_rejected_count);
        fprintf('     - Emergency Braking:%6d steps\n', emergency_braking_count);
        fprintf('  Avg Solve Time:        %6.2f ms/step\n', mean_solve_time);
        if emergency_braking_count > 0
            reasons = unique(history.failure_reason(history.solver_status == 0));
            fprintf('  MPC Failure Reasons:   %s\n', strjoin(reasons, ', '));
        end
        if safety_rejected_count > 0
            reasons = unique(history.filter_reason(history.filter_active));
            fprintf('  Filter Reasons:        %s\n', strjoin(reasons, ', '));
        end
        fprintf('────────────────────────────────────────────────────────────────────────────────────────\n');
        if passed
            fprintf('  STAGE 5 AUDIT STATUS:  [ PASS ]\n');
        else
            fprintf('  STAGE 5 AUDIT STATUS:  [ FAIL ]\n');
        end
        fprintf('════════════════════════════════════════════════════════════════════════════════════════\n\n');
    end
end
