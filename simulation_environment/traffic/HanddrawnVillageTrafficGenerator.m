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
    %   8-11. Cattle Herd (4 cows) with stochastic 2D wandering and proximity response
    
    properties
        seed                    double = 42
        rng_stream
        active_agents           = []
        next_id                 double = 1
        
        % Lifecycle counters
        total_spawned           double = 11
        pedestrian_crossings    double = 0
        cattle_crossings        double = 0
        vehicles_passed_ego     double = 0
        max_simultaneous_active double = 0
        passenger_spawned       logical = false
        density_level           char = 'MEDIUM'
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
            % Starts behind/beside ego in upper lane, v=10.5 m/s
            y_c1 = getYc(3.0);
            ag1 = TrafficAgent(1, 'bike', 3.0, y_c1 + 1.20, 10.5, 1);
            ag1.id_str = 'BIKE_001';
            ag1.v_target = 10.5;
            ag1.behavior_state = 'CRUISING';
            
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
            
            % Cattle Herd: 4 individual cows on the curved road section (outer road edge & verge)
            % Cow 1 (Upper road edge & shoulder near curve entry)
            y_c8 = getYc(120.0);
            ag8 = TrafficAgent(8, 'cattle', 120.0, y_c8 + 2.35, 0.15, 1);
            ag8.id_str = 'COW_01';
            ag8.behavior_state = 'GRAZING';
            
            % Cow 2 (Upper verge & road boundary)
            y_c9 = getYc(125.0);
            ag9 = TrafficAgent(9, 'cattle', 125.0, y_c9 + 2.45, 0.12, 1);
            ag9.id_str = 'COW_02';
            ag9.behavior_state = 'GRAZING';
            
            % Cow 3 (Verge / shoulder grass)
            y_c10 = getYc(131.0);
            ag10 = TrafficAgent(10, 'cattle', 131.0, y_c10 + 2.55, 0.10, 1);
            ag10.id_str = 'COW_03';
            ag10.behavior_state = 'GRAZING';
            
            % Cow 4 (Upper road curve edge / shoulder)
            y_c11 = getYc(137.0);
            ag11 = TrafficAgent(11, 'cattle', 137.0, y_c11 + 2.50, 0.15, 1);
            ag11.id_str = 'COW_04';
            ag11.behavior_state = 'GRAZING';
            
            obj.active_agents = [ag1; ag2; ag4; ag5; ag6; ag7; ag8; ag9; ag10; ag11];
            obj.next_id = 12;
            obj.pedestrian_crossings = 1;
            obj.cattle_crossings = 2;
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
            
            % 2. Step Fast Bike
            bike_ag = obj.getAgentById(1);
            if ~isempty(bike_ag) && bike_ag.is_active
                % Squeeze maneuver: stays in upper lane avoiding pothole 1 and auto
                bike_ag.x = bike_ag.x + bike_ag.v * dt;
                bike_ag.vx = bike_ag.v;
                bike_ag.vy = 0.0;
                if ~isempty(road_geom)
                    [y_c_bike, th_bike, ~] = road_geom.getCenterline(bike_ag.x);
                    if bike_ag.x > 26.0
                        bike_ag.y = min(y_c_bike + 1.85, bike_ag.y + 0.4 * dt);
                    else
                        bike_ag.y = y_c_bike + 1.10 + 0.10 * sin(bike_ag.x / 10.0);
                    end
                    bike_ag.theta = th_bike;
                else
                    if bike_ag.x > 26.0
                        bike_ag.y = min(4.85, bike_ag.y + 0.4 * dt);
                    else
                        bike_ag.y = 4.10 + 0.10 * sin(bike_ag.x / 10.0);
                    end
                end
            end
            
            % 3. Step Oncoming Car 1
            car_onc1 = obj.getAgentById(4);
            if ~isempty(car_onc1) && car_onc1.is_active
                car_onc1.x = car_onc1.x - car_onc1.v * dt;
                car_onc1.vx = -car_onc1.v;
                if ~isempty(road_geom)
                    [y_c_onc1, th_onc1, ~] = road_geom.getCenterline(car_onc1.x);
                    car_onc1.y = y_c_onc1 + 1.30;
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
            
            % 8. Step Cattle Herd (4 individual cows with localized grazing & proximity reaction)
            for c_id = 8:11
                cow = obj.getAgentById(c_id);
                if isempty(cow) || ~cow.is_active, continue; end
                
                dist_to_ego = hypot(cow.x - ego_state.x, cow.y - ego_state.y);
                
                if ~isempty(road_geom)
                    [y_c_cow, th_cow, ~] = road_geom.getCenterline(cow.x);
                    [y_min_cow, y_max_cow] = road_geom.getBounds(cow.x);
                else
                    y_c_cow = 3.0; th_cow = 0.0; y_min_cow = 0.0; y_max_cow = 6.0;
                end
                
                % Desired grazing lateral offset (cluster along upper road edge and outer shoulder verge)
                switch c_id
                    case 8,  y_nom = y_c_cow + 2.35;
                    case 9,  y_nom = y_c_cow + 2.45;
                    case 10, y_nom = y_c_cow + 2.55;
                    case 11, y_nom = y_c_cow + 2.50;
                end
                
                if dist_to_ego < 12.0 && ego_state.x < cow.x
                    % Proximity alert: cow moves further onto the upper shoulder away from traffic lane
                    cow.y = min(y_max_cow + 0.6, cow.y + 0.35 * dt);
                    cow.vy = 0.35;
                    cow.vx = 0.04 * (rand(obj.rng_stream) - 0.5);
                    cow.behavior_state = 'SCATTERING_SHOULDER';
                else
                    % Localized grazing: micro-wander around y_nom
                    dy_nom = y_nom - cow.y;
                    cow.vy = 0.15 * dy_nom + 0.04 * (rand(obj.rng_stream) - 0.5);
                    cow.vx = 0.04 * (rand(obj.rng_stream) - 0.5);
                    cow.y = cow.y + cow.vy * dt;
                    cow.x = cow.x + cow.vx * dt;
                    cow.theta = th_cow + 0.15 * (rand(obj.rng_stream) - 0.5);
                    cow.behavior_state = 'GRAZING';
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
            counts = struct('car', 3, 'bike', 1, 'auto', 1, 'pedestrian', 2, 'cattle', 4);
        end
        
        function counts = getSpawnCounts(obj)
            counts = struct('car', 3, 'bike', 1, 'auto', 1, 'pedestrian', 2, 'cattle', 4);
        end
        
        function active_counts = getActiveCounts(obj)
            active_counts = struct('car', 0, 'bike', 0, 'auto', 0, 'pedestrian', 0, 'cattle', 0);
            for i = 1:length(obj.active_agents)
                ag = obj.active_agents(i);
                if isfield(active_counts, ag.class_type)
                    active_counts.(ag.class_type) = active_counts.(ag.class_type) + 1;
                end
            end
        end
    end
end
