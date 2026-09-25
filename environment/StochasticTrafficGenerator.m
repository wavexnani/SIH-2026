classdef StochasticTrafficGenerator < handle
    % STOCHASTICTRAFFICGENERATOR Controlled Stochastic Traffic Process for Unstructured Roads
    %
    % Manages:
    %   - Seeded, reproducible Poisson-like arrival process with dedicated RandStream
    %   - Heterogeneous road user generation: car, bike, auto, pedestrian, cattle
    %   - Bidirectional traffic (forward and oncoming) and lateral crossing events
    %   - Strict Spawn Safety Gate to prevent overlapping/colliding insertions
    %   - Online agent lifecycle management (no precomputed trajectories, realistic persistence)
    %   - Complete forensic telemetry of all stochastic realizations
    
    properties
        seed                double = 42
        rng_stream          % Dedicated RandStream object
        rate_lambda         double = 0.25    % Average arrivals per second (~1 every 4s)
        time_since_last     double = 0.0     % Timer for next arrival check
        next_arrival_time   double = 2.0     % Scheduled arrival time (s)
        
        % Agent ID sequencing
        next_id             double = 1
        
        % Active Traffic Agents
        active_agents       TrafficAgent = TrafficAgent.empty(0, 0)
        max_active_agents   double = 8       % Bound concurrent active density on 150m road
        
        % Class Selection Probability Weights [car, bike, auto, pedestrian, cattle]
        class_weights       double = [0.28, 0.25, 0.22, 0.15, 0.10]
        class_names         cell = {'car', 'bike', 'auto', 'pedestrian', 'cattle'}
        
        % Forensic realization log
        spawn_history       struct = struct('id', {}, 'id_str', {}, 'class_type', {}, ...
                                            'spawn_time', {}, 'spawn_x', {}, 'spawn_y', {}, ...
                                            'initial_speed', {}, 'target_speed', {}, ...
                                            'direction', {}, 'entry_source', {})
        
        % Statistics & Counters
        total_spawned       double = 0
        total_despawned     double = 0
        deferred_spawns     double = 0
        pedestrian_crossings double = 0
        cattle_crossings    double = 0
    end
    
    methods
        function obj = StochasticTrafficGenerator(seed, rate_lambda, varargin)
            if nargin >= 1 && ~isempty(seed), obj.seed = seed; end
            if nargin >= 2 && ~isempty(rate_lambda), obj.rate_lambda = rate_lambda; end
            
            p = inputParser;
            addParameter(p, 'max_active', 8, @isnumeric);
            addParameter(p, 'class_weights', [0.28, 0.25, 0.22, 0.15, 0.10], @isnumeric);
            if nargin > 2
                parse(p, varargin{:});
                obj.max_active_agents = p.Results.max_active;
                weights = p.Results.class_weights;
                obj.class_weights = weights / sum(weights);
            end
            
            obj.reset();
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
        end
        
        function step(obj, dt, sim_time, road_geom, ego_state)
            % STEP Advances traffic generation, online agent updates, and lifecycle management
            
            % 1. Step all currently active agents online
            for i = length(obj.active_agents):-1:1
                ag = obj.active_agents(i);
                if ~ag.is_active
                    continue;
                end
                
                % Online behavioral step
                ag.stepOnline(dt, road_geom, obj.active_agents, ego_state);
                
                % 2. Lifecycle termination check
                should_despawn = false;
                
                % Boundary check: agent moved past road limits
                if ag.x < -25.0 || ag.x > (road_geom.road_length + 25.0)
                    should_despawn = true;
                end
                
                % Crossing completion check
                if strcmp(ag.behavior_state, 'EXITING')
                    should_despawn = true;
                end
                
                % Passed ego and traveled far behind
                if (ag.x < ego_state.x - 30.0) && (ag.direction == -1 || ag.v <= ego_state.v)
                    should_despawn = true;
                end
                
                if should_despawn
                    ag.is_active = false;
                    obj.active_agents(i) = [];
                    obj.total_despawned = obj.total_despawned + 1;
                end
            end
            
            % 3. Check for new traffic arrival
            obj.time_since_last = obj.time_since_last + dt;
            if obj.time_since_last >= obj.next_arrival_time
                obj.time_since_last = 0.0;
                % Next Poisson inter-arrival interval
                obj.next_arrival_time = -log(max(1e-4, rand(obj.rng_stream))) / obj.rate_lambda;
                
                if length(obj.active_agents) < obj.max_active_agents
                    obj.attemptSpawn(sim_time, road_geom, ego_state);
                else
                    obj.deferred_spawns = obj.deferred_spawns + 1;
                end
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
            
            % 2. Direction and Entry Geometry Determination
            [y_c_ego, ~, ~] = road_geom.getCenterline(ego_state.x);
            [y_min_ego, y_max_ego] = road_geom.getBounds(ego_state.x);
            
            switch chosen_class
                case {'car', 'auto'}
                    % 55% oncoming, 45% same-direction
                    if rand(obj.rng_stream) < 0.55
                        % Oncoming vehicle: enters ahead of ego moving -x
                        dir = -1;
                        spawn_x = min(road_geom.road_length - 5.0, ego_state.x + 65.0 + 25.0 * rand(obj.rng_stream));
                        [y_c_sp, ~, ~] = road_geom.getCenterline(spawn_x);
                        [~, y_max_sp] = road_geom.getBounds(spawn_x);
                        spawn_y = y_c_sp + (y_max_sp - y_c_sp) / 2.0; % Oncoming lane
                        entry_src = 'main_ahead';
                        
                        if strcmp(chosen_class, 'car')
                            v_init = 6.0 + 3.0 * rand(obj.rng_stream);
                        else
                            v_init = 4.0 + 2.0 * rand(obj.rng_stream);
                        end
                        theta_init = pi;
                    else
                        % Same-direction vehicle ahead of ego
                        dir = 1;
                        spawn_x = ego_state.x + 35.0 + 20.0 * rand(obj.rng_stream);
                        [y_c_sp, ~, ~] = road_geom.getCenterline(spawn_x);
                        [y_min_sp, ~] = road_geom.getBounds(spawn_x);
                        spawn_y = y_min_sp + (y_c_sp - y_min_sp) / 2.0; % Right lane
                        entry_src = 'main_ahead';
                        
                        if strcmp(chosen_class, 'car')
                            v_init = 5.0 + 3.5 * rand(obj.rng_stream);
                        else
                            v_init = 3.5 + 2.0 * rand(obj.rng_stream);
                        end
                        theta_init = 0.0;
                    end
                    target_y_cross = spawn_y;
                    
                case 'bike'
                    % Motorcycles: either oncoming or same-direction, closer to edges
                    if rand(obj.rng_stream) < 0.50
                        dir = -1;
                        spawn_x = min(road_geom.road_length - 5.0, ego_state.x + 55.0 + 30.0 * rand(obj.rng_stream));
                        [~, y_max_sp] = road_geom.getBounds(spawn_x);
                        spawn_y = y_max_sp - 1.10;
                        theta_init = pi;
                        entry_src = 'main_ahead';
                    else
                        dir = 1;
                        spawn_x = ego_state.x + 30.0 + 25.0 * rand(obj.rng_stream);
                        [y_min_sp, ~] = road_geom.getBounds(spawn_x);
                        spawn_y = y_min_sp + 1.10;
                        theta_init = 0.0;
                        entry_src = 'main_ahead';
                    end
                    v_init = 5.5 + 4.0 * rand(obj.rng_stream);
                    target_y_cross = spawn_y;
                    
                case 'pedestrian'
                    % 60% crossing, 40% walking along edge
                    if rand(obj.rng_stream) < 0.60
                        dir = 0; % Crossing
                        spawn_x = ego_state.x + 22.0 + 20.0 * rand(obj.rng_stream);
                        [y_min_sp, y_max_sp] = road_geom.getBounds(spawn_x);
                        if rand(obj.rng_stream) < 0.50
                            % Cross from right to left
                            spawn_y = y_min_sp - 0.20;
                            target_y_cross = y_max_sp + 0.40;
                            theta_init = pi/2;
                            entry_src = 'right_edge';
                        else
                            % Cross from left to right
                            spawn_y = y_max_sp + 0.20;
                            target_y_cross = y_min_sp - 0.40;
                            theta_init = -pi/2;
                            entry_src = 'left_edge';
                        end
                        v_init = 0.95 + 0.35 * rand(obj.rng_stream);
                        obj.pedestrian_crossings = obj.pedestrian_crossings + 1;
                    else
                        % Walking along shoulder
                        dir = 1;
                        spawn_x = ego_state.x + 25.0 + 20.0 * rand(obj.rng_stream);
                        [y_min_sp, ~] = road_geom.getBounds(spawn_x);
                        spawn_y = y_min_sp + 0.35;
                        target_y_cross = spawn_y;
                        theta_init = 0.0;
                        v_init = 1.10 + 0.25 * rand(obj.rng_stream);
                        entry_src = 'right_edge';
                    end
                    
                case 'cattle'
                    % 65% crossing, 35% roadside grazing
                    if rand(obj.rng_stream) < 0.65
                        dir = 0; % Crossing
                        spawn_x = ego_state.x + 28.0 + 25.0 * rand(obj.rng_stream);
                        [y_min_sp, y_max_sp] = road_geom.getBounds(spawn_x);
                        if rand(obj.rng_stream) < 0.50
                            spawn_y = y_min_sp - 0.30;
                            target_y_cross = y_max_sp + 0.50;
                            theta_init = pi/2;
                            entry_src = 'right_edge';
                        else
                            spawn_y = y_max_sp + 0.30;
                            target_y_cross = y_min_sp - 0.50;
                            theta_init = -pi/2;
                            entry_src = 'left_edge';
                        end
                        v_init = 0.40 + 0.20 * rand(obj.rng_stream);
                        obj.cattle_crossings = obj.cattle_crossings + 1;
                    else
                        dir = 1; % Grazing
                        spawn_x = ego_state.x + 30.0 + 35.0 * rand(obj.rng_stream);
                        [y_min_sp, y_max_sp] = road_geom.getBounds(spawn_x);
                        spawn_y = y_max_sp - 0.40;
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
            % CHECKSPAWNSAFETY Evaluates clearance to ego and all active agents
            is_clear = true;
            
            % Minimum headway clearance buffers
            if strcmp(class_type, 'pedestrian') || strcmp(class_type, 'cattle')
                d_min = 4.5;
            else
                d_min = 8.0;
            end
            
            % Check ego distance
            if hypot(spawn_x - ego_state.x, spawn_y - ego_state.y) < (d_min + 3.0)
                is_clear = false;
                return;
            end
            
            % Check all existing active agents
            for i = 1:length(obj.active_agents)
                ag = obj.active_agents(i);
                if ~ag.is_active, continue; end
                if hypot(spawn_x - ag.x, spawn_y - ag.y) < d_min
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
    end
end
