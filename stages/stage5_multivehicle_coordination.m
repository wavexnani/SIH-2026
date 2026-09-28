function [passed, metrics, history, simulationLog] = stage5_multivehicle_coordination(varargin)
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
    addParameter(p, 'traffic_mode', '', @ischar);
    addParameter(p, 'traffic_seed', [], @isnumeric);
    addParameter(p, 'road_type', '', @ischar);
    addParameter(p, 'density', 'MEDIUM', @ischar);
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
    param_traffic_mode = p.Results.traffic_mode;
    param_traffic_seed = p.Results.traffic_seed;
    param_road_type = p.Results.road_type;
    param_density = p.Results.density;
    
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
    
    if exist('setup_paths.m', 'file')
        setup_paths;
    end
    
    % Initialize Config & Environment
    config = SimulationConfig();
    world = ScenarioDefinitions(scenario_name, config, 'seed', sim_seed, 'density', param_density);
    vehicle = BicycleModel(config);
    
    % Override traffic mode, traffic seed, or road type if passed as parameters
    t_seed = sim_seed;
    if ~isempty(param_traffic_seed)
        t_seed = param_traffic_seed;
    end
    if ~isempty(param_traffic_mode)
        world.traffic_mode = param_traffic_mode;
        if strcmpi(param_traffic_mode, 'stochastic') && isempty(world.traffic_generator)
            world.traffic_generator = StochasticTrafficGenerator(t_seed, 'density', param_density);
        end
    end
    if ~isempty(world.traffic_generator)
        world.traffic_generator.seed = t_seed;
        world.traffic_generator.configureDensity(param_density);
        world.traffic_generator.reset();
        world.traffic_generator.populateInitialScene(world.road_geometry, world.ego);
        world.agents = world.traffic_generator.getLegacyAgentsArray();
        world.n_agents = length(world.agents);
    end
    if ~isempty(param_road_type)
        world.road_geometry = RoadGeometry(param_road_type, 'road_length', config.road_length, ...
                                           'road_width', config.road_width, 'y_center', config.road_center_y);
    end
    
    % Enforce local perception window in stochastic village traffic
    if strcmpi(world.traffic_mode, 'stochastic')
        obs_model.enforce_sensor_range = true;
    end
    
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
    has_curvature = isprop(world, 'road_geometry') && ~isempty(world.road_geometry) && world.road_geometry.curve_amp > 0;
    is_village = contains(scenario_name, 'village') || contains(scenario_name, 'stochastic');
    ctrl5.setOvertakeEnabled(~strcmp(scenario_name, 'multi_vehicle_following') && (~has_curvature || is_village));
    
    world.ego.v = v_init_ego;
    dt = config.dt;
    v_target_nominal = 8.0;
    
    % Generate Nominal Center Reference Path (conforming to road geometry curvature)
    N_path = 500;
    ref_path = zeros(N_path, 5);
    ref_path(:, 1) = linspace(0, 150, N_path)';
    if isprop(world, 'road_geometry') && ~isempty(world.road_geometry) && world.road_geometry.curve_amp > 0
        y_offset_lane = world.ego.y - world.road_geometry.y_center_base;
        for r = 1:N_path
            [y_c_r, th_r, ~] = world.road_geometry.getCenterline(ref_path(r, 1));
            ref_path(r, 2) = y_c_r + y_offset_lane;
            ref_path(r, 3) = th_r; % Heading angle theta_ref
            ref_path(r, 4) = 0.0;
        end
    else
        ref_path(:, 2) = world.ego.y;
        ref_path(:, 3) = 0.0;
        ref_path(:, 4) = 0.0;
    end
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
    
    % Traffic Ecosystem & Multi-Agent Processing Telemetry Arrays
    history.n_world_agents = zeros(N_steps, 1);
    history.n_observed_agents = zeros(N_steps, 1);
    history.n_detected_agents = zeros(N_steps, 1);
    history.n_predicted_agents = zeros(N_steps, 1);
    history.n_interacting_agents = zeros(N_steps, 1);
    history.n_planner_relevant_agents = zeros(N_steps, 1);
    history.observed_classes = cell(N_steps, 1);
    history.n_observed_classes = zeros(N_steps, 1);
    history.macro_intent = cell(N_steps, 1);
    
    hard_qp_count = 0; soft_qp_count = 0; safety_rejected_count = 0; emergency_braking_count = 0;
    
    intent_counts.MAINTAIN = 0;
    intent_counts.FOLLOW = 0;
    intent_counts.YIELD = 0;
    intent_counts.OVERTAKE = 0;
    
    % --- Structured Pipeline Logging (instrumentation only) ---
    simulationLog = cell(N_steps, 1);
    % Store scenario metadata once
    simMeta.scenario_name = scenario_name;
    simMeta.seed = sim_seed;
    simMeta.dt = dt;
    simMeta.max_steps = N_steps;
    simMeta.duration = N_steps * dt;
    simMeta.perception_mode = obs_model.mode;
    simMeta.actuator_mode = actuator_model.mode;
    simMeta.v_target_nominal = v_target_nominal;
    simMeta.traffic_mode = world.traffic_mode;
    if isprop(world, 'road_geometry') && ~isempty(world.road_geometry)
        simMeta.road_type = world.road_geometry.road_type;
        simMeta.potholes = world.road_geometry.potholes;
        simMeta.curve_amp = world.road_geometry.curve_amp;
        simMeta.curve_lambda = world.road_geometry.curve_lambda;
        simMeta.curve_x_start = world.road_geometry.curve_x_start;
        simMeta.grade_slope = world.road_geometry.grade_slope;
        simMeta.boundary_noise_amp = world.road_geometry.boundary_noise_amp;
        if isprop(world.road_geometry, 'bridge_canal')
            simMeta.bridge_canal = world.road_geometry.bridge_canal;
        else
            simMeta.bridge_canal = [];
        end
        if isprop(world.road_geometry, 'speed_breakers')
            simMeta.speed_breakers = world.road_geometry.speed_breakers;
        else
            simMeta.speed_breakers = [];
        end
    else
        simMeta.road_type = 'straight';
        simMeta.potholes = [];
        simMeta.bridge_canal = [];
        simMeta.speed_breakers = [];
        simMeta.curve_amp = 0.0;
        simMeta.curve_lambda = 80.0;
        simMeta.curve_x_start = 20.0;
        simMeta.grade_slope = 0.0;
        simMeta.boundary_noise_amp = 0.0;
    end
    simMeta.road_bounds = [0, config.road_length, config.road_center_y - config.road_width/2, config.road_center_y + config.road_width/2];
    simMeta.road_length = config.road_length;
    simMeta.road_width = config.road_width;
    simMeta.vehicle_length = config.vehicle_length;
    simMeta.vehicle_width = config.vehicle_width;
    simMeta.wheelbase = config.wheelbase;
    % Store initial static obstacles
    simMeta.static_obstacles = [];
    for s_i = 1:world.n_static_obs
        obs_s = world.static_obs(s_i, :);
        if obs_s(1) > -50
            simMeta.static_obstacles = [simMeta.static_obstacles; struct('id', s_i, 'x', obs_s(1), 'y', obs_s(2), 'L', obs_s(3), 'W', obs_s(4))];
        end
    end
    % Store initial agent info
    simMeta.initial_agents = [];
    for a_i = 1:world.n_agents
        ag = world.agents(a_i);
        if ag.x > -50
            simMeta.initial_agents = [simMeta.initial_agents; struct('id', ag.id, 'x', ag.x, 'y', ag.y, 'vx', ag.vx, 'vy', ag.vy, 'length', ag.length, 'width', ag.width)];
        end
    end
    
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
        
        history.n_world_agents(k) = world.n_agents;
        history.n_observed_agents(k) = obs_world.n_agents;
        
        % Track unique observed heterogeneous classes
        if obs_world.n_agents > 0
            obs_cls_list = {};
            for o_i = 1:obs_world.n_agents
                ag_o = obs_world.agents(o_i);
                if isprop(ag_o, 'type') && ~isempty(ag_o.type)
                    obs_cls_list{end+1} = ag_o.type;
                elseif isprop(ag_o, 'class_type') && ~isempty(ag_o.class_type)
                    obs_cls_list{end+1} = ag_o.class_type;
                end
            end
            u_cls = unique(obs_cls_list);
            history.observed_classes{k} = u_cls;
            history.n_observed_classes(k) = length(u_cls);
        else
            history.observed_classes{k} = {};
            history.n_observed_classes(k) = 0;
        end
        
        % Execute Stage 5 Control Step using Observed World State
        [u_cmd, pred_states, status, info] = ctrl5.step(obs_world, ref_path, v_target_nominal);
        
        history.n_detected_agents(k) = info.num_detected_agents;
        history.n_predicted_agents(k) = info.num_predicted_agents;
        history.n_interacting_agents(k) = info.num_interacting_agents;
        history.n_planner_relevant_agents(k) = info.num_planner_relevant;
        history.macro_intent{k} = info.macro_intent;
        
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
        
        % Closed-Loop Dynamics Step for Ego Vehicle Plant (accounting for road grade gravity effect)
        grade_ang = 0.0;
        if isprop(world, 'road_geometry') && ~isempty(world.road_geometry)
            [grade_ang, ~] = world.road_geometry.getGrade(world.ego.x);
        end
        a_applied_plant = a_actual_cmd - 9.81 * sin(grade_ang);
        world.ego = vehicle.stepKinematic(world.ego, a_applied_plant, delta_actual_cmd, dt);
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
        [~, nearest_ref_idx] = min(abs(ref_path(:, 1) - world.ego.x));
        y_nominal = ref_path(nearest_ref_idx, 2);
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
        
        % --- Populate simulationLog(k) from actual pipeline state ---
        slog.time = t;
        slog.step = k;
        
        % Ground Truth
        slog.groundTruth.ego = struct('x', world.ego.x, 'y', world.ego.y, 'theta', world.ego.theta, 'v', world.ego.v, 'delta', world.ego.delta, 'a', world.ego.a);
        gt_agents = [];
        for gt_i = 1:world.n_agents
            ag = world.agents(gt_i);
            if ag.x > -50
                ag_id_str = sprintf('A%d', ag.id);
                if isprop(ag, 'id_str') && ~isempty(ag.id_str), ag_id_str = ag.id_str; end
                ag_type = 'car';
                if isprop(ag, 'type') && ~isempty(ag.type), ag_type = ag.type; end
                ag_state = 'CRUISING';
                if isprop(ag, 'behavior_state') && ~isempty(ag.behavior_state), ag_state = ag.behavior_state; end
                ag_dir = 1;
                if isprop(ag, 'direction') && ~isempty(ag.direction), ag_dir = ag.direction; end
                
                gt_agents = [gt_agents; struct('id', ag.id, 'id_str', ag_id_str, 'type', ag_type, ...
                                               'behavior_state', ag_state, 'direction', ag_dir, ...
                                               'x', ag.x, 'y', ag.y, 'vx', ag.vx, 'vy', ag.vy, ...
                                               'length', ag.length, 'width', ag.width)];
            end
        end
        slog.groundTruth.agents = gt_agents;
        slog.groundTruth.min_clearance = clearance;
        slog.groundTruth.is_collision = is_coll;
        slog.groundTruth.in_bounds = in_bounds;
        
        % Road Geometry Telemetry
        if isprop(world, 'road_geometry') && ~isempty(world.road_geometry)
            [y_c_k, ~, ~] = world.road_geometry.getCenterline(world.ego.x);
            [y_min_k, y_max_k] = world.road_geometry.getBounds(world.ego.x);
            slog.road = struct('type', world.road_geometry.road_type, 'y_center', y_c_k, ...
                               'y_min', y_min_k, 'y_max', y_max_k, 'grade', grade_ang);
        end
        
        % Observation
        slog.observation.mode = obs_model.mode;
        obs_agents_log = [];
        if ~isempty(obs_structs)
            for obs_i = 1:length(obs_structs)
                if obs_structs(obs_i).agent_id > 0
                    obs_s = obs_structs(obs_i);
                    s_id_str = sprintf('A%d', obs_s.agent_id);
                    if isfield(obs_s, 'id_str') && ~isempty(obs_s.id_str), s_id_str = obs_s.id_str; end
                    s_type = 'car';
                    if isfield(obs_s, 'type') && ~isempty(obs_s.type), s_type = obs_s.type; end
                    s_bstate = 'CRUISING';
                    if isfield(obs_s, 'behavior_state') && ~isempty(obs_s.behavior_state), s_bstate = obs_s.behavior_state; end
                    s_conf = 1.0;
                    if isfield(obs_s, 'confidence') && ~isempty(obs_s.confidence), s_conf = obs_s.confidence; end
                    
                    obs_agents_log = [obs_agents_log; struct('id', obs_s.agent_id, 'id_str', s_id_str, ...
                                                            'type', s_type, 'behavior_state', s_bstate, ...
                                                            'x', obs_s.x, 'y', obs_s.y, ...
                                                            'velocity', obs_s.velocity, 'heading', obs_s.heading, ...
                                                            'vx', obs_s.vx, 'vy', obs_s.vy, ...
                                                            'age', obs_s.age, 'confidence', s_conf)];
                end
            end
        end
        slog.observation.agents = obs_agents_log;
        
        % Detections
        det_log = [];
        if isfield(info, 'detections') && ~isempty(info.detections)
            for d_i = 1:length(info.detections)
                d = info.detections(d_i);
                d_type = 'car';
                if isfield(d, 'type') && ~isempty(d.type), d_type = d.type; end
                d_id_str = sprintf('A%d', d.id);
                if isfield(d, 'id_str') && ~isempty(d.id_str), d_id_str = d.id_str; end
                d_len = 4.7; if isfield(d, 'length') && ~isempty(d.length), d_len = d.length; end
                d_wid = 1.8; if isfield(d, 'width') && ~isempty(d.width), d_wid = d.width; end
                
                det_log = [det_log; struct('id', d.id, 'id_str', d_id_str, 'type', d_type, ...
                                           'length', d_len, 'width', d_wid, ...
                                           'x', d.x, 'y', d.y, 'vx', d.vx, 'vy', d.vy, 'v', d.v, ...
                                           'dx', d.dx, 'dy', d.dy, 'd_rel', d.d_rel, ...
                                           'is_ahead', d.is_ahead, 'is_same_lane', d.is_same_lane, 'is_oncoming', d.is_oncoming)];
            end
        end
        slog.detections = det_log;
        
        % Prediction
        pred_log = [];
        if isfield(info, 'predictions') && ~isempty(info.predictions)
            for p_i = 1:length(info.predictions)
                p = info.predictions(p_i);
                pred_log = [pred_log; struct('id', p.id, 'x_traj', p.x_traj, 'y_traj', p.y_traj, 'vx', p.vx, 'vy', p.vy)];
            end
        end
        slog.prediction.agents = pred_log;
        slog.prediction.horizon_steps = 10;
        slog.prediction.dt = 0.10;
        
        % Risk
        slog.risk.min_ttc = min_ttc_val;
        if isfield(info, 'ttc_vector')
            slog.risk.ttc_vector = info.ttc_vector;
        else
            slog.risk.ttc_vector = [];
        end
        
        % Interaction
        int_log = [];
        if isfield(info, 'interactions') && ~isempty(info.interactions)
            for i_i = 1:length(info.interactions)
                ix = info.interactions(i_i);
                int_log = [int_log; struct('id', ix.id, 'class_name', ix.class_name, 'primary_concern', ix.primary_concern, 'risk_level', ix.risk_level, 'ttc', ix.ttc)];
            end
        end
        slog.interaction = int_log;
        
        % Decision
        if isfield(info, 'decision')
            slog.decision = info.decision;
        else
            slog.decision = struct('macro_intent', info.macro_intent, 'target_v', 0, 'reason', '');
        end
        
        % Free Space
        if isfield(info, 'y_min_vec')
            slog.freeSpace.y_min_vec = info.y_min_vec';
            slog.freeSpace.y_max_vec = info.y_max_vec';
            slog.freeSpace.min_corridor_width = info.min_delta_y;
        else
            slog.freeSpace = struct('y_min_vec', [], 'y_max_vec', [], 'min_corridor_width', 0);
        end
        
        % Topology
        slog.topology.selected = info.selected_topology;
        if isfield(info, 'locked_side'), slog.topology.locked_side = info.locked_side; else, slog.topology.locked_side = 'none'; end
        if isfield(info, 'ok_geom_left'), slog.topology.ok_geom_left = info.ok_geom_left; else, slog.topology.ok_geom_left = true; end
        if isfield(info, 'ok_geom_right'), slog.topology.ok_geom_right = info.ok_geom_right; else, slog.topology.ok_geom_right = true; end
        if isfield(info, 'J_left'), slog.topology.J_left = info.J_left; else, slog.topology.J_left = 0; end
        if isfield(info, 'J_right'), slog.topology.J_right = info.J_right; else, slog.topology.J_right = 0; end
        
        % MPC
        slog.mpc.status = status;
        slog.mpc.solve_time_ms = info.solve_time_ms;
        slog.mpc.is_soft = info.is_soft;
        slog.mpc.failure_reason = info.failure_reason;
        if isfield(info, 'pred_ego_traj') && ~isempty(info.pred_ego_traj)
            slog.mpc.pred_ego_traj = info.pred_ego_traj;
        else
            slog.mpc.pred_ego_traj = [];
        end
        slog.mpc.u_cmd = u_cmd(:)';
        
        % Safety
        slog.safety.filter_active = info.filter_active;
        slog.safety.filter_reason = info.filter_reason;
        if isfield(info, 'u_safe'), slog.safety.u_safe = info.u_safe(:)'; else, slog.safety.u_safe = u_cmd(:)'; end
        
        % Actuation
        slog.actuation.delta_cmd = u_cmd(1);
        slog.actuation.a_cmd = u_cmd(2);
        slog.actuation.delta_actual = delta_actual_cmd;
        slog.actuation.a_actual = a_actual_cmd;
        slog.actuation.delta_plant = world.ego.delta;
        slog.actuation.a_plant = world.ego.a;
        slog.actuation.steering_error = act_info.steering_error;
        slog.actuation.acceleration_error = act_info.acceleration_error;
        
        % Ego post-dynamics
        slog.ego = struct('x', world.ego.x, 'y', world.ego.y, 'theta', world.ego.theta, 'v', world.ego.v, 'delta', world.ego.delta, 'a', world.ego.a);
        
        simulationLog{k} = slog;
    end
    
    % Attach metadata to history for export
    history.simulationLog = simulationLog;
    history.simMeta = simMeta;
    
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
    metrics.min_clearance = min_clr;
    metrics.hard_qp_count = hard_qp_count;
    metrics.soft_qp_count = soft_qp_count;
    metrics.safety_rejected_count = safety_rejected_count;
    metrics.emergency_braking_count = emergency_braking_count;
    metrics.mean_solve_time = mean_solve_time;
    metrics.intent_counts = intent_counts;
    metrics.lat_rmse = lat_rmse;
    
    % Stochastic Environment Forensic Metrics
    if strcmpi(world.traffic_mode, 'stochastic') && ~isempty(world.traffic_generator)
        metrics.traffic_count = world.traffic_generator.total_spawned;
        metrics.active_agent_count = length(world.traffic_generator.active_agents);
        metrics.agent_type_counts = world.traffic_generator.getClassCounts();
        metrics.pedestrian_crossings = world.traffic_generator.pedestrian_crossings;
        metrics.cattle_crossings = world.traffic_generator.cattle_crossings;
        metrics.vehicles_passed_ego = world.traffic_generator.vehicles_passed_ego;
        metrics.max_simultaneous_active = max(history.n_world_agents);
        metrics.max_simultaneous_observed = max(history.n_observed_agents);
        metrics.max_simultaneous_detected = max(history.n_detected_agents);
        metrics.max_simultaneous_predicted = max(history.n_predicted_agents);
        metrics.max_simultaneous_interacting = max(history.n_interacting_agents);
        metrics.max_simultaneous_planner_relevant = max(history.n_planner_relevant_agents);
        metrics.max_simultaneous_observed_classes = max(history.n_observed_classes);
    else
        metrics.traffic_count = world.n_agents;
        metrics.active_agent_count = world.n_agents;
        metrics.agent_type_counts = struct('car', world.n_agents, 'bike', 0, 'auto', 0, 'pedestrian', 0, 'cattle', 0);
        metrics.pedestrian_crossings = 0;
        metrics.cattle_crossings = 0;
        metrics.vehicles_passed_ego = 0;
        metrics.deferred_spawns = 0;
        metrics.max_simultaneous_active = world.n_agents;
        metrics.max_simultaneous_observed = world.n_agents;
        metrics.max_simultaneous_detected = max(history.n_detected_agents);
        metrics.max_simultaneous_predicted = max(history.n_predicted_agents);
        metrics.max_simultaneous_interacting = max(history.n_interacting_agents);
        metrics.max_simultaneous_planner_relevant = max(history.n_planner_relevant_agents);
        metrics.max_simultaneous_observed_classes = max(history.n_observed_classes);
    end
    
    % Multi-Agent Hero Frame Selection (highest heterogeneous class diversity within FOV)
    [max_cls, hero_idx] = max(history.n_observed_classes);
    if max_cls < 4
        [~, hero_idx] = max(history.n_observed_agents);
    end
    metrics.hero_frame.step = hero_idx;
    metrics.hero_frame.time = history.t(hero_idx);
    metrics.hero_frame.ego_x = history.ego_x(hero_idx);
    metrics.hero_frame.ego_y = history.ego_y(hero_idx);
    metrics.hero_frame.ego_v = history.ego_v(hero_idx);
    metrics.hero_frame.n_world = history.n_world_agents(hero_idx);
    metrics.hero_frame.n_observed = history.n_observed_agents(hero_idx);
    metrics.hero_frame.n_detected = history.n_detected_agents(hero_idx);
    metrics.hero_frame.n_predicted = history.n_predicted_agents(hero_idx);
    metrics.hero_frame.n_interacting = history.n_interacting_agents(hero_idx);
    metrics.hero_frame.n_planner_relevant = history.n_planner_relevant_agents(hero_idx);
    metrics.hero_frame.observed_classes = history.observed_classes{hero_idx};
    metrics.hero_frame.macro_intent = history.macro_intent{hero_idx};
    metrics.hero_frame.min_clr = history.min_clearance(hero_idx);
    metrics.min_ttc = min(history.min_ttc);
    
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
        if strcmpi(world.traffic_mode, 'stochastic')
            fprintf('  Continuous Traffic Ecosystem & Multi-Agent Processing Metrics:\n');
            fprintf('     - Max Simultaneous Active (World):    %6d\n', metrics.max_simultaneous_active);
            fprintf('     - Max Simultaneous Observed (Ego):    %6d\n', metrics.max_simultaneous_observed);
            fprintf('     - Max Simultaneous Detected:          %6d\n', metrics.max_simultaneous_detected);
            fprintf('     - Max Simultaneous Predicted:         %6d\n', metrics.max_simultaneous_predicted);
            fprintf('     - Max Simultaneous Interacting:       %6d\n', metrics.max_simultaneous_interacting);
            fprintf('     - Max Simultaneous Planner-Relevant:  %6d\n', metrics.max_simultaneous_planner_relevant);
            fprintf('     - Max Heterogeneous Classes in Frame: %6d\n', metrics.max_simultaneous_observed_classes);
            fprintf('     - Total Spawned Agents:               %6d\n', metrics.traffic_count);
            fprintf('     - Cars Spawned:                       %6d\n', metrics.agent_type_counts.car);
            fprintf('     - Bikes Spawned:                      %6d\n', metrics.agent_type_counts.bike);
            fprintf('     - Autos Spawned:                      %6d\n', metrics.agent_type_counts.auto);
            fprintf('     - Pedestrians Spawned:                %6d\n', metrics.agent_type_counts.pedestrian);
            fprintf('     - Cattle Spawned:                     %6d\n', metrics.agent_type_counts.cattle);
            fprintf('     - Pedestrian Crossings:               %6d\n', metrics.pedestrian_crossings);
            fprintf('     - Cattle Crossings:                   %6d\n', metrics.cattle_crossings);
            fprintf('     - Vehicles Passed Ego:                %6d\n', metrics.vehicles_passed_ego);
            fprintf('  Multi-Agent Hero Frame Snapshot:\n');
            fprintf('     - Hero Timestep:                      Step %d (t = %.2f s)\n', metrics.hero_frame.step, metrics.hero_frame.time);
            fprintf('     - Ego Position:                       (x = %.2f m, y = %.2f m, v = %.2f m/s)\n', metrics.hero_frame.ego_x, metrics.hero_frame.ego_y, metrics.hero_frame.ego_v);
            fprintf('     - Agents Simultaneously Observed:     %d\n', metrics.hero_frame.n_observed);
            fprintf('     - Agents Simultaneously Detected:     %d\n', metrics.hero_frame.n_detected);
            fprintf('     - Agents Simultaneously Predicted:    %d\n', metrics.hero_frame.n_predicted);
            fprintf('     - Heterogeneous Classes Present:      %s\n', strjoin(metrics.hero_frame.observed_classes, ', '));
            fprintf('     - Macro Intent & Clearance:           %s (min_clr = +%.3f m)\n', metrics.hero_frame.macro_intent, metrics.hero_frame.min_clr);
        end
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
