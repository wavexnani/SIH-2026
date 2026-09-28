classdef StochasticTrafficGenerator < handle
    % STOCHASTICTRAFFICGENERATOR Controlled Continuous Stochastic Traffic Ecosystem for Unstructured Roads
    %
    % Manages:
    %   - Seeded, reproducible Poisson-like arrival process with dedicated RandStream
    %   - Initial scene population with multiple simultaneous heterogeneous road users (at t=0)
    %   - Continuous traffic replenishment according to density levels (LOW, MEDIUM, HIGH)
    %   - Heterogeneous road users: car, bike, auto, pedestrian, cattle
    %   - Bidirectional traffic (forward, oncoming) and lateral crossing events
    %   - Strict Spawn Safety Gate to prevent overlapping/colliding insertions
    %   - Online agent lifecycle management (no precomputed trajectories, realistic persistence)
    %   - Complete forensic telemetry of all stochastic realizations
    
    properties
        seed                double = 42
        rng_stream          % Dedicated RandStream object
        density_level       char = 'MEDIUM'  % 'LOW', 'MEDIUM', 'HIGH'
        target_active_agents double = 9      % Target active agents in world
        max_active_agents   double = 16      % Max active capacity bound
        
        rate_lambda         double = 0.85    % Average arrival rate parameter (s^-1)
        time_since_last     double = 0.0     % Timer for next arrival check
        next_arrival_time   double = 1.0     % Scheduled arrival time (s)
        
        % Agent ID sequencing
        next_id             double = 1
        
        % Active Traffic Agents
        active_agents       TrafficAgent = TrafficAgent.empty(0, 0)
        
        % Class Selection Probability Weights [car, bike, auto, pedestrian, cattle]
        class_weights       double = [0.28, 0.25, 0.22, 0.15, 0.10]
        class_names         cell = {'car', 'bike', 'auto', 'pedestrian', 'cattle'}
        
        % Forensic realization log
        spawn_history       struct = struct('id', {}, 'id_str', {}, 'class_type', {}, ...
                                            'spawn_time', {}, 'spawn_x', {}, 'spawn_y', {}, ...
                                            'initial_speed', {}, 'target_speed', {}, ...
                                            'direction', {}, 'entry_source', {})
        
        % Statistics & Counters
        total_spawned           double = 0
        total_despawned         double = 0
        deferred_spawns         double = 0
        pedestrian_crossings    double = 0
        cattle_crossings        double = 0
        vehicles_passed_ego     double = 0
        max_simultaneous_active double = 0
        
        % Scene dimensions
        local_scene_ahead   double = 65.0    % Forward local scene range (m)
        local_scene_behind  double = 20.0    % Rear local scene range (m)
    end
    
    methods
        function obj = StochasticTrafficGenerator(seed, varargin)
            if nargin >= 1 && ~isempty(seed), obj.seed = seed; end
            
            p = inputParser;
            addParameter(p, 'density', 'MEDIUM', @ischar);
            addParameter(p, 'max_active', [], @isnumeric);
            addParameter(p, 'class_weights', [0.28, 0.25, 0.22, 0.15, 0.10], @isnumeric);
            addParameter(p, 'rate_lambda', 0.85, @isnumeric);
            if nargin > 1
                parse(p, varargin{:});
                obj.density_level = upper(p.Results.density);
                if ~isempty(p.Results.max_active)
                    obj.max_active_agents = p.Results.max_active;
                end
                weights = p.Results.class_weights;
                obj.class_weights = weights / sum(weights);
                obj.rate_lambda = p.Results.rate_lambda;
            end
            
            % Configure density targets
            obj.configureDensity(obj.density_level);
            
            obj.reset();
        end
        
        function configureDensity(obj, density_str)
            % CONFIGUREDENSITY Sets target and maximum active agents based on density level
            obj.density_level = upper(density_str);
            switch obj.density_level
                case 'LOW'
                    obj.target_active_agents = 5;
                    obj.max_active_agents = 8;
                    obj.rate_lambda = 0.50;
                case 'MEDIUM'
                    obj.target_active_agents = 9;
                    obj.max_active_agents = 14;
                    obj.rate_lambda = 0.85;
                case 'HIGH'
                    obj.target_active_agents = 14;
                    obj.max_active_agents = 20;
                    obj.rate_lambda = 1.30;
                otherwise
                    obj.target_active_agents = 9;
                    obj.max_active_agents = 14;
                    obj.rate_lambda = 0.85;
            end
        end
        
        function reset(obj)
            % RESET Re-seeds stream and clears all agent history for exact forensic reproduction
            obj.rng_stream = RandStream('mt19937ar', 'Seed', obj.seed);
            obj.next_id = 1;
            obj.active_agents = TrafficAgent.empty(0, 0);
            obj.time_since_last = 0.0;
            obj.next_arrival_time = -log(max(1e-4, rand(obj.rng_stream))) / obj.rate_lambda;
            obj.spawn_history = struct('id', {}, 'id_str', {}, 'class_type', {}, ...
                                       'spawn_time', {}, 'spawn_x', {}, 'spawn_y', {}, ...
                                       'initial_speed', {}, 'target_speed', {}, ...
                                       'direction', {}, 'entry_source', {});
            obj.total_spawned = 0;
            obj.total_despawned = 0;
            obj.deferred_spawns = 0;
            obj.pedestrian_crossings = 0;
            obj.cattle_crossings = 0;
            obj.vehicles_passed_ego = 0;
            obj.max_simultaneous_active = 0;
        end
        
        function populateInitialScene(obj, road_geom, ego_state)
            % POPULATEINITIALSCENE Instantiates an initial dense, heterogeneous population
            % at t = 0 so the ego starts in a live village road scene with multiple objects
            % simultaneously present within its local perception area.
            
            if isempty(road_geom)
                road_geom = RoadGeometry('unstructured');
            end
            if isempty(ego_state)
                ego_state = struct('x', 10.0, 'y', 2.0, 'v', 5.0, 'theta', 0.0);
            end
            
            ego_x = ego_state.x;
            
            % Generate template initial agents based on density
            % Template specifies relative dx from ego, lane placement, class, direction, and behavior
            switch obj.density_level
                case 'LOW'
                    templates = { ...
                        {'bike',       ego_x + 14.0, 'right_edge',    1, 5.5, 'CRUISING'}, ...
                        {'pedestrian', ego_x + 20.0, 'right_edge',    1, 1.1, 'WALKING'}, ...
                        {'car',        ego_x + 28.0, 'forward_lane',  1, 5.2, 'CRUISING'}, ...
                        {'cattle',     ego_x + 36.0, 'left_edge',     0, 0.35, 'CROSSING'}, ...
                        {'auto',       ego_x + 44.0, 'oncoming_lane', -1, 4.0, 'CRUISING'} ...
                    };
                case 'HIGH'
                    templates = { ...
                        {'bike',       ego_x + 12.0, 'right_edge',    1, 5.8, 'CRUISING'}, ...
                        {'pedestrian', ego_x + 18.0, 'right_edge',    1, 1.0, 'WALKING'}, ...
                        {'car',        ego_x + 25.0, 'forward_lane',  1, 5.0, 'CRUISING'}, ...
                        {'cattle',     ego_x + 32.0, 'left_edge',     0, 0.35, 'CROSSING'}, ...
                        {'auto',       ego_x + 40.0, 'oncoming_lane', -1, 4.2, 'CRUISING'}, ...
                        {'pedestrian', ego_x + 46.0, 'left_edge',     0, 0.95, 'CROSSING'}, ...
                        {'bike',       ego_x + 52.0, 'oncoming_edge', -1, 6.0, 'CRUISING'}, ...
                        {'car',        ego_x + 58.0, 'forward_lane',  1, 4.8, 'CRUISING'}, ...
                        {'car',        ego_x + 65.0, 'oncoming_lane', -1, 6.5, 'CRUISING'}, ...
                        {'bike',       ego_x + 72.0, 'right_edge',    1, 5.2, 'CRUISING'}, ...
                        {'auto',       ego_x + 80.0, 'forward_lane',  1, 4.0, 'CRUISING'}, ...
                        {'pedestrian', ego_x + 86.0, 'right_edge',    1, 1.1, 'WALKING'}, ...
                        {'cattle',     ego_x + 94.0, 'right_edge',    0, 0.2, 'GRAZING'} ...
                    };
                otherwise % 'MEDIUM'
                    templates = { ...
                        {'bike',       ego_x + 14.0, 'right_edge',    1, 5.8, 'CRUISING'}, ...
                        {'pedestrian', ego_x + 20.0, 'right_edge',    1, 1.1, 'WALKING'}, ...
                        {'car',        ego_x + 28.0, 'forward_lane',  1, 5.2, 'CRUISING'}, ...
                        {'cattle',     ego_x + 36.0, 'left_edge',     0, 0.35, 'CROSSING'}, ...
                        {'auto',       ego_x + 44.0, 'oncoming_lane', -1, 4.0, 'CRUISING'}, ...
                        {'bike',       ego_x + 52.0, 'oncoming_edge', -1, 6.2, 'CRUISING'}, ...
                        {'pedestrian', ego_x + 60.0, 'left_edge',     1, 1.0, 'WALKING'}, ...
                        {'car',        ego_x + 70.0, 'oncoming_lane', -1, 6.5, 'CRUISING'}, ...
                        {'auto',       ego_x + 80.0, 'forward_lane',  1, 4.2, 'CRUISING'} ...
                    };
            end
            
            for k = 1:length(templates)
                tmpl = templates{k};
                c_type = tmpl{1};
                sp_x = tmpl{2} + 3.0 * (rand(obj.rng_stream) - 0.5);
                lane_type = tmpl{3};
                dir = tmpl{4};
                v_init = max(0.2, tmpl{5} + 0.8 * (rand(obj.rng_stream) - 0.5));
                b_state = tmpl{6};
                
                % Query road bounds at sp_x
                [y_c, th_r, ~] = road_geom.getCenterline(sp_x);
                [y_min_sp, y_max_sp] = road_geom.getBounds(sp_x);
                
                target_y_cross = y_c;
                switch lane_type
                    case 'right_edge'
                        if strcmp(c_type, 'bike')
                            sp_y = y_min_sp + 0.45;
                        else
                            sp_y = y_min_sp + 0.25; % Pedestrian on outer shoulder
                        end
                        if dir == 1, th_init = th_r; else, th_init = th_r + pi; end
                        entry_src = 'right_edge';
                        
                    case 'forward_lane'
                        sp_y = y_min_sp + (y_c - y_min_sp) / 2.0;
                        th_init = th_r;
                        entry_src = 'main_ahead';
                        
                    case 'oncoming_lane'
                        sp_y = y_c + (y_max_sp - y_c) / 2.0;
                        th_init = th_r + pi;
                        entry_src = 'main_ahead';
                        
                    case 'oncoming_edge'
                        sp_y = y_max_sp - 0.45;
                        th_init = th_r + pi;
                        entry_src = 'main_ahead';
                        
                    case 'left_edge'
                        if strcmp(b_state, 'CROSSING')
                            sp_y = y_max_sp - 0.15;
                            target_y_cross = y_min_sp + 0.25;
                            th_init = -pi/2;
                            dir = 0;
                        else
                            sp_y = y_max_sp - 0.30;
                            th_init = th_r + pi;
                        end
                        entry_src = 'left_edge';
                end
                
                % Safety check
                if ~obj.checkSpawnSafety(sp_x, sp_y, c_type, ego_state)
                    continue;
                end
                
                % Instantiate agent
                new_id = obj.next_id;
                obj.next_id = obj.next_id + 1;
                
                ag = TrafficAgent(new_id, c_type, sp_x, sp_y, v_init, dir, target_y_cross);
                ag.theta = th_init;
                ag.behavior_state = b_state;
                ag.entry_source = entry_src;
                ag.spawn_time = 0.0;
                
                if strcmp(c_type, 'pedestrian') && strcmp(b_state, 'CROSSING')
                    obj.pedestrian_crossings = obj.pedestrian_crossings + 1;
                elseif strcmp(c_type, 'cattle') && strcmp(b_state, 'CROSSING')
                    obj.cattle_crossings = obj.cattle_crossings + 1;
                end
                
                obj.active_agents(end+1) = ag;
                obj.total_spawned = obj.total_spawned + 1;
                
                % Log
                rec.id = new_id;
                rec.id_str = ag.id_str;
                rec.class_type = c_type;
                rec.spawn_time = 0.0;
                rec.spawn_x = sp_x;
                rec.spawn_y = sp_y;
                rec.initial_speed = v_init;
                rec.target_speed = ag.v_target;
                rec.direction = dir;
                rec.entry_source = entry_src;
                obj.spawn_history(end+1) = rec;
            end
            
            obj.max_simultaneous_active = length(obj.active_agents);
        end
        
        function step(obj, dt, sim_time, road_geom, ego_state)
            % STEP Advances traffic generation, online agent updates, and lifecycle management
            
            if isempty(ego_state)
                ego_state = struct('x', 10.0, 'y', 2.0, 'v', 5.0, 'theta', 0.0);
            end
            
            % If empty at first step, populate initial scene
            if isempty(obj.active_agents) && obj.next_id == 1
                obj.populateInitialScene(road_geom, ego_state);
            end
            
            % 1. Step all currently active agents online
            for i = length(obj.active_agents):-1:1
                ag = obj.active_agents(i);
                if ~ag.is_active
                    continue;
                end
                
                prev_x = ag.x;
                
                % Online behavioral step
                ag.stepOnline(dt, road_geom, obj.active_agents, ego_state);
                
                % Track vehicles passing ego
                if (prev_x >= ego_state.x && ag.x < ego_state.x && ag.direction == -1) || ...
                   (prev_x <= ego_state.x && ego_state.x > ag.x + ag.length && ag.direction == 1)
                    obj.vehicles_passed_ego = obj.vehicles_passed_ego + 1;
                end
                
                % 2. Lifecycle termination check
                % IMPORTANT: Agent remains active until it physically leaves the modeled scene
                should_despawn = false;
                
                % Road longitudinal bounds
                if ag.x < -25.0 || ag.x > (road_geom.road_length + 25.0)
                    should_despawn = true;
                end
                
                % Crossing completed and fully exited road onto shoulder
                if strcmp(ag.behavior_state, 'EXITING')
                    should_despawn = true;
                end
                
                % Passed far behind ego (no longer relevant to ego perception or interaction)
                if (ag.x < ego_state.x - obj.local_scene_behind) && (ag.direction == -1 || ag.v <= ego_state.v)
                    should_despawn = true;
                end
                
                % Traveled far ahead beyond ego local scene
                if (ag.x > ego_state.x + obj.local_scene_ahead + 25.0) && (ag.direction == 1 && ag.v >= ego_state.v)
                    should_despawn = true;
                end
                
                if should_despawn
                    ag.is_active = false;
                    obj.active_agents(i) = [];
                    obj.total_despawned = obj.total_despawned + 1;
                end
            end
            
            % Track max simultaneous active agents
            current_active_count = length(obj.active_agents);
            if current_active_count > obj.max_simultaneous_active
                obj.max_simultaneous_active = current_active_count;
            end
            
            % 3. Check for continuous traffic replenishment
            obj.time_since_last = obj.time_since_last + dt;
            is_underpopulated = (current_active_count < obj.target_active_agents);
            arrival_trigger = (obj.time_since_last >= obj.next_arrival_time);
            
            if (is_underpopulated || arrival_trigger) && current_active_count < obj.max_active_agents
                obj.time_since_last = 0.0;
                % Next Poisson inter-arrival interval
                obj.next_arrival_time = -log(max(1e-4, rand(obj.rng_stream))) / obj.rate_lambda;
                
                obj.attemptSpawn(sim_time, road_geom, ego_state);
            end
        end
        
        function attemptSpawn(obj, sim_time, road_geom, ego_state)
            % ATTEMPTSPAWN Evaluates physical possibility and instantiates a candidate agent
            
            % 1. Stochastic Class Selection
            r_class = rand(obj.rng_stream);
            cum_w = cumsum(obj.class_weights);
            class_idx = find(r_class <= cum_w, 1, 'first');
            if isempty(class_idx), class_idx = 1; end
            chosen_class = obj.class_names{class_idx};
            
            % 2. Entry Location & Direction Determination
            % Support multiple physical entry regions: ahead, behind, road edges
            [y_c_ego, ~, ~] = road_geom.getCenterline(ego_state.x);
            
            switch chosen_class
                case {'car', 'auto'}
                    r_dir = rand(obj.rng_stream);
                    if r_dir < 0.55
                        % Oncoming vehicle: enters ahead moving toward ego (-x) in opposing lane
                        dir = -1;
                        spawn_x = min(road_geom.road_length - 5.0, ego_state.x + 50.0 + 20.0 * rand(obj.rng_stream));
                        [y_c_sp, th_sp, ~] = road_geom.getCenterline(spawn_x);
                        [~, y_max_sp] = road_geom.getBounds(spawn_x);
                        spawn_y = y_c_sp + (y_max_sp - y_c_sp) / 2.0; % Opposing lane
                        entry_src = 'main_ahead_oncoming';
                        theta_init = th_sp + pi;
                        
                        if strcmp(chosen_class, 'car')
                            v_init = 5.5 + 2.5 * rand(obj.rng_stream);
                        else
                            v_init = 3.8 + 1.8 * rand(obj.rng_stream);
                        end
                    elseif r_dir < 0.85
                        % Same-direction vehicle ahead: enters ahead moving +x
                        dir = 1;
                        spawn_x = ego_state.x + 35.0 + 25.0 * rand(obj.rng_stream);
                        [y_c_sp, th_sp, ~] = road_geom.getCenterline(spawn_x);
                        [y_min_sp, ~] = road_geom.getBounds(spawn_x);
                        spawn_y = y_min_sp + (y_c_sp - y_min_sp) / 2.0; % Forward lane
                        entry_src = 'main_ahead_forward';
                        theta_init = th_sp;
                        
                        if strcmp(chosen_class, 'car')
                            v_init = 5.0 + 2.5 * rand(obj.rng_stream);
                        else
                            v_init = 3.5 + 1.5 * rand(obj.rng_stream);
                        end
                    else
                        % Entering from behind: approaching ego from rear
                        dir = 1;
                        spawn_x = max(2.0, ego_state.x - 18.0 - 6.0 * rand(obj.rng_stream));
                        [y_c_sp, th_sp, ~] = road_geom.getCenterline(spawn_x);
                        [y_min_sp, ~] = road_geom.getBounds(spawn_x);
                        spawn_y = y_min_sp + (y_c_sp - y_min_sp) / 2.0;
                        entry_src = 'main_behind';
                        theta_init = th_sp;
                        v_init = 5.5 + 2.0 * rand(obj.rng_stream);
                    end
                    target_y_cross = spawn_y;
                    
                case 'bike'
                    % Motorcycles: either oncoming or same-direction, closer to road edges
                    r_bike = rand(obj.rng_stream);
                    if r_bike < 0.50
                        dir = -1;
                        spawn_x = min(road_geom.road_length - 5.0, ego_state.x + 45.0 + 25.0 * rand(obj.rng_stream));
                        [~, th_sp, ~] = road_geom.getCenterline(spawn_x);
                        [~, y_max_sp] = road_geom.getBounds(spawn_x);
                        spawn_y = y_max_sp - 0.85;
                        theta_init = th_sp + pi;
                        entry_src = 'main_ahead_oncoming';
                    elseif r_bike < 0.85
                        dir = 1;
                        spawn_x = ego_state.x + 25.0 + 25.0 * rand(obj.rng_stream);
                        [~, th_sp, ~] = road_geom.getCenterline(spawn_x);
                        [y_min_sp, ~] = road_geom.getBounds(spawn_x);
                        spawn_y = y_min_sp + 0.85;
                        theta_init = th_sp;
                        entry_src = 'main_ahead_forward';
                    else
                        dir = 1;
                        spawn_x = max(2.0, ego_state.x - 16.0 - 5.0 * rand(obj.rng_stream));
                        [~, th_sp, ~] = road_geom.getCenterline(spawn_x);
                        [y_min_sp, ~] = road_geom.getBounds(spawn_x);
                        spawn_y = y_min_sp + 0.85;
                        theta_init = th_sp;
                        entry_src = 'main_behind';
                    end
                    v_init = 5.5 + 3.0 * rand(obj.rng_stream);
                    target_y_cross = spawn_y;
                    
                case 'pedestrian'
                    % 55% crossing across road, 45% walking along shoulder
                    if rand(obj.rng_stream) < 0.55
                        dir = 0; % Crossing
                        spawn_x = ego_state.x + 20.0 + 25.0 * rand(obj.rng_stream);
                        [y_min_sp, y_max_sp] = road_geom.getBounds(spawn_x);
                        if rand(obj.rng_stream) < 0.50
                            % Cross from right to left
                            spawn_y = y_min_sp + 0.10;
                            target_y_cross = y_max_sp - 0.20;
                            theta_init = pi/2;
                            entry_src = 'right_edge';
                        else
                            % Cross from left to right
                            spawn_y = y_max_sp - 0.10;
                            target_y_cross = y_min_sp + 0.20;
                            theta_init = -pi/2;
                            entry_src = 'left_edge';
                        end
                        v_init = 0.95 + 0.35 * rand(obj.rng_stream);
                        obj.pedestrian_crossings = obj.pedestrian_crossings + 1;
                    else
                        % Walking along shoulder
                        dir = 1;
                        spawn_x = ego_state.x + 18.0 + 30.0 * rand(obj.rng_stream);
                        [y_min_sp, ~] = road_geom.getBounds(spawn_x);
                        spawn_y = y_min_sp + 0.40;
                        target_y_cross = spawn_y;
                        theta_init = 0.0;
                        v_init = 1.10 + 0.25 * rand(obj.rng_stream);
                        entry_src = 'right_edge';
                    end
                    
                case 'cattle'
                    % 60% crossing, 40% roadside grazing
                    if rand(obj.rng_stream) < 0.60
                        dir = 0; % Crossing
                        spawn_x = ego_state.x + 24.0 + 25.0 * rand(obj.rng_stream);
                        [y_min_sp, y_max_sp] = road_geom.getBounds(spawn_x);
                        if rand(obj.rng_stream) < 0.50
                            spawn_y = y_min_sp + 0.15;
                            target_y_cross = y_max_sp - 0.25;
                            theta_init = pi/2;
                            entry_src = 'right_edge';
                        else
                            spawn_y = y_max_sp - 0.15;
                            target_y_cross = y_min_sp + 0.25;
                            theta_init = -pi/2;
                            entry_src = 'left_edge';
                        end
                        v_init = 0.35 + 0.20 * rand(obj.rng_stream);
                        obj.cattle_crossings = obj.cattle_crossings + 1;
                    else
                        dir = 1; % Grazing
                        spawn_x = ego_state.x + 25.0 + 35.0 * rand(obj.rng_stream);
                        [~, y_max_sp] = road_geom.getBounds(spawn_x);
                        spawn_y = y_max_sp - 0.50;
                        target_y_cross = spawn_y;
                        theta_init = 0.0;
                        v_init = 0.05;
                        entry_src = 'left_edge';
                    end
            end
            
            % 3. SPAWN SAFETY GATE
            % Verify candidate footprint does not overlap ego or any existing active agent
            is_clear = obj.checkSpawnSafety(spawn_x, spawn_y, chosen_class, ego_state);
            if ~is_clear
                obj.deferred_spawns = obj.deferred_spawns + 1;
                return;
            end
            
            % 4. Instantiate and Activate Agent
            new_id = obj.next_id;
            obj.next_id = obj.next_id + 1;
            
            new_agent = TrafficAgent(new_id, chosen_class, spawn_x, spawn_y, v_init, dir, target_y_cross);
            new_agent.theta = theta_init;
            new_agent.entry_source = entry_src;
            new_agent.spawn_time = sim_time;
            
            obj.active_agents(end+1) = new_agent;
            obj.total_spawned = obj.total_spawned + 1;
            
            % 5. Log Forensic Telemetry
            rec.id = new_id;
            rec.id_str = new_agent.id_str;
            rec.class_type = chosen_class;
            rec.spawn_time = sim_time;
            rec.spawn_x = spawn_x;
            rec.spawn_y = spawn_y;
            rec.initial_speed = v_init;
            rec.target_speed = new_agent.v_target;
            rec.direction = dir;
            rec.entry_source = entry_src;
            
            obj.spawn_history(end+1) = rec;
        end
        
        function is_clear = checkSpawnSafety(obj, spawn_x, spawn_y, class_type, ego_state)
            % CHECKSPAWNSAFETY Evaluates clearance to ego and all active agents using oriented road headway
            is_clear = true;
            
            % Check ego distance (longitudinal and lateral)
            dx_ego = abs(spawn_x - ego_state.x);
            dy_ego = abs(spawn_y - ego_state.y);
            if dy_ego < 1.2
                if dx_ego < 8.0, is_clear = false; return; end
            else
                if dx_ego < 4.5 && dy_ego < 0.8, is_clear = false; return; end
            end
            
            % Check all existing active agents
            for i = 1:length(obj.active_agents)
                ag = obj.active_agents(i);
                if ~ag.is_active, continue; end
                dx = abs(spawn_x - ag.x);
                dy = abs(spawn_y - ag.y);
                
                % Same lane / conflict zone vs adjacent lane / shoulder
                if dy < 1.1
                    if strcmp(class_type, 'pedestrian') || strcmp(ag.class_type, 'pedestrian')
                        d_lon_min = 4.0;
                    else
                        d_lon_min = 6.0;
                    end
                else
                    d_lon_min = 3.2;
                end
                
                if dx < d_lon_min && dy < 0.85
                    is_clear = false;
                    return;
                end
            end
        end
        
        function agents_array = getLegacyAgentsArray(obj)
            % GETLEGACYAGENTSARRAY Converts all active TrafficAgents into an array of legacy Agent objects
            N = length(obj.active_agents);
            if N == 0
                agents_array = Agent.empty(0, 0);
                return;
            end
            
            agents_array = Agent.empty(N, 0);
            for i = 1:N
                agents_array(i) = obj.active_agents(i).toLegacyAgent();
            end
        end
        
        function counts = getClassCounts(obj)
            % GETCLASSCOUNTS Returns breakdown count of all spawned agents by class
            counts = struct('car', 0, 'bike', 0, 'auto', 0, 'pedestrian', 0, 'cattle', 0);
            for i = 1:length(obj.spawn_history)
                c_type = obj.spawn_history(i).class_type;
                if isfield(counts, c_type)
                    counts.(c_type) = counts.(c_type) + 1;
                end
            end
        end
        
        function active_counts = getActiveCounts(obj)
            % GETACTIVECOUNTS Returns breakdown of currently active agents by class
            active_counts = struct('car', 0, 'bike', 0, 'auto', 0, 'pedestrian', 0, 'cattle', 0);
            for i = 1:length(obj.active_agents)
                ag = obj.active_agents(i);
                if ag.is_active && isfield(active_counts, ag.class_type)
                    active_counts.(ag.class_type) = active_counts.(ag.class_type) + 1;
                end
            end
        end
    end
end
