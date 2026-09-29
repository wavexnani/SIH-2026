classdef HanddrawnVillageTrafficGenerator < handle
    % HANDDRAWNVILLAGETRAFFICGENERATOR Interactive Traffic Generator for Hand-Drawn Village Scene
    %
    % Digitalizes the hand-drawn rural road scene with:
    %   1. Fast Bike overtaking ego and squeezing past auto and pothole 1
    %   2. Auto-Rickshaw slowing down and stopping to drop off a passenger
    %   3. Passenger (Pedestrian 1) disembarking from stopped auto-rickshaw
    %   4. Oncoming Car 1 slowing down for auto/bike squeeze bottleneck
    %   5. Lead Car cruising then decelerating for speed breaker and crossing pedestrian
    %   6. Reckless Pedestrian 2 crossing with stochastic Brownian jitter without caring about traffic
    %   7. Oncoming Car 2 crawling over speed breaker/bridge then accelerating
    %   8-15. Sheep Flock (8 sheep) with boids flocking (cohesion, separation, foraging & startle)
    
    properties
        seed                    double = 42
        rng_stream
        active_agents           = []
        next_id                 double = 1
        
        % Lifecycle counters
        total_spawned           double = 23
        sheep_side              char = 'lower' % 'lower' (corrected) or 'upper' (legacy)
        n_sheep                 double = 16
        pedestrian_crossings    double = 0
        cattle_crossings        double = 0
        sheep_crossings         double = 0
        vehicles_passed_ego     double = 0
        max_simultaneous_active double = 0
        passenger_spawned       logical = false
        density_level           char = 'MEDIUM'
        enable_blind_bend_agents logical = false
    end
    
    methods
        function obj = HanddrawnVillageTrafficGenerator(seed)
            if nargin >= 1 && ~isempty(seed), obj.seed = seed; end
            obj.rng_stream = RandStream('mt19937ar', 'Seed', obj.seed);
            obj.active_agents = [];
            obj.next_id = 1;
            obj.passenger_spawned = false;
        end
        
        function configureDensity(obj, density_str)
            if nargin >= 2 && ~isempty(density_str)
                obj.density_level = upper(density_str);
            end
        end
        
        function reset(obj)
            obj.rng_stream = RandStream('mt19937ar', 'Seed', obj.seed);
            obj.active_agents = [];
            obj.next_id = 1;
            obj.passenger_spawned = false;
        end
        
        function populateInitialScene(obj, road_geom, ego_state)
            obj.populateScene(road_geom, ego_state);
        end
        
        function populateScene(obj, road_geom, ego_state)
            % POPULATESCENE Initializes the agents according to the hand-drawn blueprint
            obj.active_agents = [];
            
            % Helper to query centerline if road_geom provided
            getYc = @(x_val) 3.0;
            if nargin >= 2 && ~isempty(road_geom) && ismethod(road_geom, 'getCenterline')
                getYc = @(x_val) road_geom.getCenterline(x_val);
            end
            
            % Agent 1: Fast Bike (Overtaking Ego, squeezing through bottleneck)
            % Starts behind/beside ego in upper lane, v=9.5 m/s
            y_c1 = getYc(3.0);
            ag1 = TrafficAgent(1, 'bike', 3.0, y_c1 + 1.15, 9.5, 1);
            ag1.id_str = 'BIKE_001';
            ag1.v_target = 9.5;
            ag1.behavior_state = 'APPROACH';
            ag1.length = 1.80;
            ag1.width = 0.60;
            
            % Agent 2: Auto-Rickshaw (Slowing down and stopping to drop passenger)
            % Starts ahead in right lane (y=2.2m), v=3.5 m/s
            y_c2 = getYc(22.0);
            ag2 = TrafficAgent(2, 'auto', 22.0, y_c2 - 0.80, 3.5, 1);
            ag2.id_str = 'AUTO_002';
            ag2.v_target = 0.0; % Decelerating to stop
            ag2.y_target = 0.85; % Pulling to lower shoulder
            ag2.behavior_state = 'DECELERATING_TO_STOP';
            
            % Agent 4: Oncoming Car 1 (Approaching opposing lane, squeezing bottleneck)
            % Starts at x=34m, opposing lane, v=-6.5 m/s
            y_c4 = getYc(34.0);
            ag4 = TrafficAgent(4, 'car', 34.0, y_c4 + 1.30, 6.5, -1);
            ag4.id_str = 'CAR_ONCOMING_1';
            ag4.v_target = 6.5;
            ag4.behavior_state = 'YIELD_BOTTLENECK';
            ag4.theta = pi;
            
            % Agent 5: Lead Car (Ahead of auto, cruising then braking for speed breaker)
            % Starts at x=48m, lower lane, v=4.2 m/s
            y_c5 = getYc(48.0);
            ag5 = TrafficAgent(5, 'car', 48.0, y_c5 - 1.20, 4.2, 1);
            ag5.id_str = 'CAR_LEAD';
            ag5.v_target = 4.2;
            ag5.behavior_state = 'CRUISING';
            
            % Agent 6: Reckless Pedestrian 2 (Crossing near pothole 2 & speed breaker)
            % Starts at x=83m, y=0.4m (lower shoulder)
            ag6 = TrafficAgent(6, 'pedestrian', 83.0, 0.40, 1.1, 0, 5.50);
            ag6.id_str = 'PED_CROSSING';
            ag6.behavior_state = 'CROSSING';
            ag6.theta = pi/2;
            
            % Agent 7: Oncoming Car 2 (Across bridge/canal, crawling over speed breaker)
            % Starts at x=115m, upper lane, v=-2.4 m/s
            y_c7 = getYc(115.0);
            ag7 = TrafficAgent(7, 'car', 115.0, y_c7 + 1.25, 2.4, -1);
            ag7.id_str = 'CAR_ONCOMING_2';
            ag7.v_target = 2.4;
            ag7.behavior_state = 'SPEED_BREAKER_CRAWL';
            ag7.theta = pi;
            
            % Sheep Flock: 16 sheep tightly clustered on the curved road section (lower road edge & verge)
            if strcmpi(obj.sheep_side, 'upper')
                % Legacy baseline: 8 sheep on upper verge
                y_c8 = getYc(122.6); ag8 = TrafficAgent(8, 'sheep', 122.6, y_c8 + 2.45, 0.12, 1); ag8.id_str = 'SHEEP_01'; ag8.behavior_state = 'GRAZING_FLOCK';
                y_c9 = getYc(123.5); ag9 = TrafficAgent(9, 'sheep', 123.5, y_c9 + 2.65, 0.10, 1); ag9.id_str = 'SHEEP_02'; ag9.behavior_state = 'GRAZING_FLOCK';
                y_c10 = getYc(124.2); ag10 = TrafficAgent(10, 'sheep', 124.2, y_c10 + 2.35, 0.14, 1); ag10.id_str = 'SHEEP_03'; ag10.behavior_state = 'GRAZING_FLOCK';
                y_c11 = getYc(124.8); ag11 = TrafficAgent(11, 'sheep', 124.8, y_c11 + 2.75, 0.08, 1); ag11.id_str = 'SHEEP_04'; ag11.behavior_state = 'GRAZING_FLOCK';
                y_c12 = getYc(123.1); ag12 = TrafficAgent(12, 'sheep', 123.1, y_c12 + 2.85, 0.11, 1); ag12.id_str = 'SHEEP_05'; ag12.behavior_state = 'GRAZING_FLOCK';
                y_c13 = getYc(125.6); ag13 = TrafficAgent(13, 'sheep', 125.6, y_c13 + 2.50, 0.13, 1); ag13.id_str = 'SHEEP_06'; ag13.behavior_state = 'GRAZING_FLOCK';
                y_c14 = getYc(125.0); ag14 = TrafficAgent(14, 'sheep', 125.0, y_c14 + 2.25, 0.09, 1); ag14.id_str = 'SHEEP_07'; ag14.behavior_state = 'GRAZING_FLOCK';
                y_c15 = getYc(123.8); ag15 = TrafficAgent(15, 'sheep', 123.8, y_c15 + 2.45, 0.12, 1); ag15.id_str = 'SHEEP_08'; ag15.behavior_state = 'GRAZING_FLOCK';
                obj.active_agents = [ag1; ag2; ag4; ag5; ag6; ag7; ag8; ag9; ag10; ag11; ag12; ag13; ag14; ag15];
                obj.next_id = 16;
            else
                % Corrected: 16 sheep tightly clustered on the LOWER side of the road
                % Spanning x in [119.5, 124.6] m along lower shoulder & road edge (same side car is travelling towards)
                sheep_x = [119.5, 120.2, 120.8, 121.4, 121.9, 122.5, 123.1, 123.7, ...
                           120.0, 120.7, 121.3, 122.0, 122.6, 123.3, 124.0, 124.6];
                sheep_dy = [-2.75, -2.95, -2.70, -3.10, -2.85, -3.20, -2.75, -3.05, ...
                            -3.25, -3.40, -3.20, -3.45, -3.30, -3.40, -3.20, -3.35];
                sheep_vx = [0.06, 0.04, 0.08, 0.05, 0.07, 0.04, 0.06, 0.05, ...
                            0.04, 0.05, 0.03, 0.06, 0.04, 0.04, 0.05, 0.04];
                
                sheep_agents = [];
                for s_i = 1:16
                    sx = sheep_x(s_i);
                    y_cs = getYc(sx);
                    sy = y_cs + sheep_dy(s_i);
                    sh_ag = TrafficAgent(7 + s_i, 'sheep', sx, sy, sheep_vx(s_i), 1);
                    sh_ag.id_str = sprintf('SHEEP_%02d', s_i);
                    sh_ag.length = 1.05;
                    sh_ag.width = 0.55;
                    sh_ag.behavior_state = 'GRAZING_FLOCK';
                    sheep_agents = [sheep_agents; sh_ag];
                end
                
                obj.active_agents = [ag1; ag2; ag4; ag5; ag6; ag7; sheep_agents];
                obj.next_id = 24;
            end
            
            % Blind Bend Challenge Agents (Heavy Tractor + Motorcycle descending the bend)
            if obj.enable_blind_bend_agents
                y_c_tr = getYc(180.0);
                ag_tr = TrafficAgent(24, 'truck', 180.0, y_c_tr + 1.25, 2.0, -1);
                ag_tr.id_str = 'TRACTOR_001';
                ag_tr.length = 5.8;
                ag_tr.width = 2.2;
                ag_tr.behavior_state = 'BLIND_BEND_CRAWL';
                ag_tr.theta = pi;
                
                y_c_bk2 = getYc(190.0);
                ag_bk2 = TrafficAgent(25, 'bike', 190.0, y_c_bk2 + 0.85, 2.8, -1);
                ag_bk2.id_str = 'BIKE_002';
                ag_bk2.length = 1.8;
                ag_bk2.width = 0.6;
                ag_bk2.behavior_state = 'BEND_DESCENT';
                ag_bk2.theta = pi;
                
                obj.active_agents = [obj.active_agents; ag_tr; ag_bk2];
                obj.next_id = 26;
            end
            
            obj.pedestrian_crossings = 1;
            obj.cattle_crossings = 0;
            obj.sheep_crossings = 2;
        end
        
        function step(obj, dt, sim_time, road_geom, ego_state)
            % STEP Advances custom online behaviors for each hand-drawn actor
            if isempty(obj.active_agents)
                obj.populateScene(road_geom, ego_state);
            end
            
            % 1. Step Auto-Rickshaw and Passenger Drop-off Logic
            auto_ag = obj.getAgentById(2);
            if ~isempty(auto_ag) && auto_ag.is_active
                if auto_ag.v > 0.15
                    % Decelerate and veer towards right/lower shoulder
                    auto_ag.v = max(0.0, auto_ag.v - 1.40 * dt);
                    auto_ag.y = max(0.85, auto_ag.y - 0.65 * dt);
                    auto_ag.x = auto_ag.x + auto_ag.v * dt;
                    auto_ag.vx = auto_ag.v;
                    auto_ag.vy = -0.65 * (auto_ag.y > 0.85);
                else
                    % Completely stopped on shoulder
                    auto_ag.v = 0.0;
                    auto_ag.vx = 0.0;
                    auto_ag.vy = 0.0;
                    auto_ag.y = 0.85;
                    auto_ag.behavior_state = 'STOPPED_PASSENGER_DROP';
                    
                    % Spawn Disembarking Passenger (Agent 3) once stopped
                    if ~obj.passenger_spawned && sim_time >= 2.5
                        ag3 = TrafficAgent(3, 'pedestrian', auto_ag.x + 0.5, 0.40, 0.8, 1);
                        ag3.id_str = 'PED_PASSENGER';
                        ag3.behavior_state = 'WALKING';
                        ag3.theta = 0.0;
                        obj.active_agents = [obj.active_agents; ag3];
                        obj.passenger_spawned = true;
                    end
                end
            end
            
            % 2. Step Fast Bike with Realistic Dynamics & Cut-In Maneuver
            bike_ag = obj.getAgentById(1);
            if ~isempty(bike_ag) && bike_ag.is_active
                bike_ag.x = bike_ag.x + bike_ag.v * dt;
                
                [y_c_bike, th_bike] = deal(3.0, 0.0);
                if ~isempty(road_geom) && ismethod(road_geom, 'getCenterline')
                    [y_c_bike, th_bike, ~] = road_geom.getCenterline(bike_ag.x);
                end
                
                % Dynamic 4-Phase Trajectory:
                % Phase 1 (x < 6.0m): High-speed approach in upper lane
                % Phase 2 (6.0 <= x <= 15.0m): Dynamic bank & cut-in across ego's front path to avoid oncoming car 1
                % Phase 3 (15.0 < x <= 27.0m): Squeeze through center corridor between oncoming car 1 and auto/pothole
                % Phase 4 (27.0 < x <= 36.0m): Re-center into upper lane and cruise
                if bike_ag.x < 6.0
                    bike_ag.behavior_state = 'APPROACH';
                    y_off = 1.15;
                    dy_dx = 0.0;
                elseif bike_ag.x <= 15.0
                    bike_ag.behavior_state = 'DYNAMIC_CUT_IN';
                    s = (bike_ag.x - 6.0) / 9.0;
                    h = 3.0 * s^2 - 2.0 * s^3;
                    hp = (6.0 * s * (1.0 - s)) / 9.0;
                    y_off = 1.15 - 1.30 * h;
                    dy_dx = -1.30 * hp;
                elseif bike_ag.x <= 27.0
                    bike_ag.behavior_state = 'BOTTLENECK_SQUEEZE';
                    y_off = -0.15;
                    dy_dx = 0.0;
                elseif bike_ag.x <= 36.0
                    bike_ag.behavior_state = 'RECENTERING';
                    s2 = (bike_ag.x - 27.0) / 9.0;
                    h2 = 3.0 * s2^2 - 2.0 * s2^3;
                    hp2 = (6.0 * s2 * (1.0 - s2)) / 9.0;
                    y_off = -0.15 + 1.30 * h2;
                    dy_dx = 1.30 * hp2;
                else
                    bike_ag.behavior_state = 'CRUISING';
                    y_off = 1.15;
                    dy_dx = 0.0;
                end
                
                bike_ag.y = y_c_bike + y_off;
                rel_psi = atan(dy_dx);
                bike_ag.theta = th_bike + rel_psi;
                bike_ag.vx = bike_ag.v * cos(rel_psi);
                bike_ag.vy = bike_ag.v * sin(rel_psi);
            end
            
            % 3. Step Oncoming Car 1
            car_onc1 = obj.getAgentById(4);
            if ~isempty(car_onc1) && car_onc1.is_active
                car_onc1.x = car_onc1.x - car_onc1.v * dt;
                car_onc1.vx = -car_onc1.v;
                car_onc1.vy = 0.0;
                if ~isempty(road_geom)
                    [y_c_onc1, th_onc1, ~] = road_geom.getCenterline(car_onc1.x);
                    car_onc1.y = y_c_onc1 + 1.40;
                    car_onc1.theta = th_onc1 + pi;
                else
                    car_onc1.y = 4.40;
                    car_onc1.theta = pi;
                end
            end
            
            % 4. Step Lead Car
            car_lead = obj.getAgentById(5);
            if ~isempty(car_lead) && car_lead.is_active
                if car_lead.x >= 70.0 && car_lead.x < 92.0
                    % Decelerating for speed breaker (x=88m) and crossing pedestrian (x=83m)
                    car_lead.v = max(2.2, car_lead.v - 1.2 * dt);
                    car_lead.behavior_state = 'SLOWING_SPEED_BREAKER';
                elseif car_lead.x >= 92.0
                    % Accelerating after speed breaker across bridge and curve
                    car_lead.v = min(5.2, car_lead.v + 1.0 * dt);
                    car_lead.behavior_state = 'ACCELERATING';
                end
                car_lead.x = car_lead.x + car_lead.v * dt;
                car_lead.vx = car_lead.v;
                if ~isempty(road_geom)
                    [y_c_lead, th_lead, ~] = road_geom.getCenterline(car_lead.x);
                    car_lead.y = y_c_lead - 1.20;
                    car_lead.theta = th_lead;
                else
                    car_lead.y = 1.80;
                end
            end
            
            % 5. Step Reckless Pedestrian 2 (Stochastic Brownian Crossing near speed breaker)
            ped2 = obj.getAgentById(6);
            if ~isempty(ped2) && ped2.is_active
                % Crossing starts as Lead Car approaches (sim_time >= 9.0s)
                if sim_time >= 9.0 && ped2.y < 5.6
                    jitter_y = 0.25 * (rand(obj.rng_stream) - 0.5);
                    jitter_x = 0.20 * (rand(obj.rng_stream) - 0.5);
                    v_cross = 0.95 + jitter_y;
                    ped2.y = ped2.y + v_cross * dt;
                    ped2.x = ped2.x + jitter_x * dt;
                    ped2.vy = v_cross;
                    ped2.vx = jitter_x;
                    ped2.theta = atan2(ped2.vy, ped2.vx);
                    ped2.behavior_state = 'CROSSING_RECKLESS';
                elseif ped2.y >= 5.6
                    ped2.behavior_state = 'EXITED_ROAD';
                    ped2.vx = 0.0; ped2.vy = 0.0;
                end
            end
            
            % 6. Step Oncoming Car 2 (Speed breaker crawl + acceleration)
            car_onc2 = obj.getAgentById(7);
            if ~isempty(car_onc2) && car_onc2.is_active
                if car_onc2.x > 86.0
                    car_onc2.v = 2.4;
                    car_onc2.behavior_state = 'SPEED_BREAKER_CRAWL';
                else
                    car_onc2.v = min(5.5, car_onc2.v + 1.2 * dt);
                    car_onc2.behavior_state = 'ACCELERATING';
                end
                car_onc2.x = car_onc2.x - car_onc2.v * dt;
                car_onc2.vx = -car_onc2.v;
                if ~isempty(road_geom)
                    [y_c_onc2, th_onc2, ~] = road_geom.getCenterline(car_onc2.x);
                    car_onc2.y = y_c_onc2 + 1.25;
                    car_onc2.theta = th_onc2 + pi;
                else
                    car_onc2.y = 4.30;
                    car_onc2.theta = pi;
                end
            end
            
            % 7. Step Disembarked Passenger (if spawned)
            ped1 = obj.getAgentById(3);
            if ~isempty(ped1) && ped1.is_active
                ped1.x = ped1.x + 0.85 * dt;
                ped1.vx = 0.85;
                ped1.vy = 0.0;
                ped1.y = 0.40;
            end
            
            % 8. Step Sheep Flock (16 sheep with Boids flocking: cohesion, separation, foraging & startle)
            if strcmpi(obj.sheep_side, 'upper')
                sheep_ids = 8:15;
            else
                sheep_ids = 8:23;
            end
            
            % Compute local flock centroid
            x_flock = 0.0; y_flock = 0.0; n_flock = 0;
            for sid = sheep_ids
                sh = obj.getAgentById(sid);
                if ~isempty(sh) && sh.is_active
                    x_flock = x_flock + sh.x;
                    y_flock = y_flock + sh.y;
                    n_flock = n_flock + 1;
                end
            end
            if n_flock > 0
                x_flock = x_flock / n_flock;
                y_flock = y_flock / n_flock;
            end
            
            for sid = sheep_ids
                sh = obj.getAgentById(sid);
                if isempty(sh) || ~sh.is_active, continue; end
                
                dist_to_ego = hypot(sh.x - ego_state.x, sh.y - ego_state.y);
                
                if ~isempty(road_geom)
                    [y_c_sh, th_sh, ~] = road_geom.getCenterline(sh.x);
                    [y_min_sh, y_max_sh] = road_geom.getBounds(sh.x);
                else
                    y_c_sh = 3.0; th_sh = 0.0; y_min_sh = 0.0; y_max_sh = 6.0;
                end
                
                if strcmpi(obj.sheep_side, 'upper')
                    % Legacy upper nominal anchors
                    switch sid
                        case 8,  y_nom = y_c_sh + 2.45;
                        case 9,  y_nom = y_c_sh + 2.65;
                        case 10, y_nom = y_c_sh + 2.35;
                        case 11, y_nom = y_c_sh + 2.75;
                        case 12, y_nom = y_c_sh + 2.85;
                        case 13, y_nom = y_c_sh + 2.50;
                        case 14, y_nom = y_c_sh + 2.25;
                        case 15, y_nom = y_c_sh + 2.45;
                        otherwise, y_nom = y_c_sh + 2.50;
                    end
                else
                    % Lower side nominal anchors (resting & grazing along lower road edge)
                    s_idx = sid - 7;
                    dy_list = [-2.75, -2.95, -2.70, -3.10, -2.85, -3.20, -2.75, -3.05, ...
                               -3.25, -3.40, -3.20, -3.45, -3.30, -3.40, -3.20, -3.35];
                    if s_idx >= 1 && s_idx <= length(dy_list)
                        y_nom = y_c_sh + dy_list(s_idx);
                    else
                        y_nom = y_c_sh - 2.50;
                    end
                end
                
                % Boids Flocking Vector 1: Cohesion toward flock centroid
                coh_x = 0.08 * (x_flock - sh.x);
                coh_y = 0.08 * (y_flock - sh.y);
                
                % Boids Flocking Vector 2: Pairwise Separation (repulsion from close mates < 0.70m)
                rep_x = 0.0; rep_y = 0.0;
                for oid = sheep_ids
                    if oid == sid, continue; end
                    other = obj.getAgentById(oid);
                    if isempty(other) || ~other.is_active, continue; end
                    dx_so = sh.x - other.x;
                    dy_so = sh.y - other.y;
                    d_so = hypot(dx_so, dy_so);
                    if d_so < 0.70 && d_so > 1e-4
                        rep_mag = (0.70 - d_so) / d_so;
                        rep_x = rep_x + 0.35 * dx_so * rep_mag;
                        rep_y = rep_y + 0.35 * dy_so * rep_mag;
                    end
                end
                
                if dist_to_ego < 10.0 && ego_state.x < sh.x
                    % Collective startle/scatter
                    if strcmpi(obj.sheep_side, 'upper')
                        sh.y = min(y_max_sh + 0.70, sh.y + 0.30 * dt);
                        sh.vy = 0.30;
                        sh.theta = th_sh + pi/2 + 0.15 * (rand(obj.rng_stream) - 0.5);
                    else
                        sh.y = max(y_min_sh - 0.70, sh.y - 0.25 * dt);
                        sh.vy = -0.25;
                        sh.theta = th_sh - pi/2 + 0.15 * (rand(obj.rng_stream) - 0.5);
                    end
                    sh.vx = coh_x + rep_x + 0.04 * (rand(obj.rng_stream) - 0.5);
                    sh.v = hypot(sh.vx, sh.vy);
                    sh.behavior_state = 'SCATTERING_FLOCK';
                else
                    % Natural grazing & foraging wander
                    drift_x = 0.03 * (rand(obj.rng_stream) - 0.5);
                    drift_y = 0.03 * (rand(obj.rng_stream) - 0.5);
                    pull_y = 0.20 * (y_nom - sh.y);
                    
                    sh.vx = coh_x + rep_x + drift_x;
                    sh.vy = coh_y + rep_y + pull_y + drift_y;
                    
                    v_mag = hypot(sh.vx, sh.vy);
                    if v_mag > 0.15
                        sh.vx = (sh.vx / v_mag) * 0.15;
                        sh.vy = (sh.vy / v_mag) * 0.15;
                    end
                    
                    sh.x = sh.x + sh.vx * dt;
                    sh.y = sh.y + sh.vy * dt;
                    sh.v = hypot(sh.vx, sh.vy);
                    
                    if sh.v > 0.02
                        sh.theta = atan2(sh.vy, sh.vx);
                    else
                        sh.theta = th_sh + 0.20 * (rand(obj.rng_stream) - 0.5);
                    end
                    sh.behavior_state = 'GRAZING_FLOCK';
                end
            end
            
            % 9. Step Tractor & Country Bike (descending the blind bend)
            tr = obj.getAgentById(24);
            if ~isempty(tr) && tr.is_active
                if sim_time >= 22.0
                    tr.x = tr.x - tr.v * dt;
                end
                if ~isempty(road_geom)
                    [y_c_tr, th_tr, ~] = road_geom.getCenterline(tr.x);
                    tr.y = y_c_tr + 1.25;
                    tr.theta = th_tr + pi;
                end
            end
            bk2 = obj.getAgentById(25);
            if ~isempty(bk2) && bk2.is_active
                if sim_time >= 25.0
                    bk2.x = bk2.x - bk2.v * dt;
                end
                if ~isempty(road_geom)
                    [y_c_bk2, th_bk2, ~] = road_geom.getCenterline(bk2.x);
                    bk2.y = y_c_bk2 + 0.85;
                    bk2.theta = th_bk2 + pi;
                end
            end
            
            % Update max active count
            n_act = length(obj.active_agents);
            if n_act > obj.max_simultaneous_active
                obj.max_simultaneous_active = n_act;
            end
        end
        
        function ag = getAgentById(obj, id)
            ag = [];
            for i = 1:length(obj.active_agents)
                if obj.active_agents(i).id == id
                    ag = obj.active_agents(i);
                    return;
                end
            end
        end
        
        function legacy_agents = getLegacyAgentsArray(obj)
            % GETLEGACYAGENTSARRAY Converts active agents into standard Agent array for WorldState
            N = length(obj.active_agents);
            if N == 0
                legacy_agents = [];
                return;
            end
            
            for i = 1:N
                ta = obj.active_agents(i);
                a = Agent(ta.id, ta.x, ta.y, ta.vx, ta.vy, 0.20);
                a.id_str = ta.id_str;
                a.type = ta.class_type;
                a.behavior_state = ta.behavior_state;
                a.direction = ta.direction;
                a.length = ta.length;
                a.width = ta.width;
                a.heading = ta.theta;
                a.v = hypot(ta.vx, ta.vy);
                legacy_agents(i) = a;
            end
        end
        
        function counts = getClassCounts(obj)
            counts = struct('car', 3, 'bike', 1, 'auto', 1, 'pedestrian', 2, 'cattle', 0, 'sheep', 8);
        end
        
        function counts = getSpawnCounts(obj)
            counts = struct('car', 3, 'bike', 1, 'auto', 1, 'pedestrian', 2, 'cattle', 0, 'sheep', 8);
        end
        
        function active_counts = getActiveCounts(obj)
            active_counts = struct('car', 0, 'bike', 0, 'auto', 0, 'pedestrian', 0, 'cattle', 0, 'sheep', 0);
            for i = 1:length(obj.active_agents)
                ag = obj.active_agents(i);
                if isfield(active_counts, ag.class_type)
                    active_counts.(ag.class_type) = active_counts.(ag.class_type) + 1;
                end
            end
        end
    end
end
