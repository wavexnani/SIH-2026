classdef TrafficAgent < handle
    % TRAFFICAGENT Heterogeneous Dynamic Traffic Participant
    %
    % Supported Classes:
    %   - 'car': Sedan/SUV with IDM longitudinal dynamics and PD path following
    %   - 'bike': Motorcycle with smaller footprint, edge-following, agile lane positioning
    %   - 'auto': Auto-rickshaw with moderate speed, slower acceleration, intermediate footprint
    %   - 'pedestrian': Walking along road/shoulder or crossing across road with safety awareness
    %   - 'cattle': Bovine/animal movement, slow crossing, edge grazing, indifferent to traffic
    %
    % Direction:
    %   +1 = forward (moving with increasing x)
    %   -1 = oncoming (moving with decreasing x)
    %    0 = lateral crossing (pedestrian/cattle)
    
    properties
        id                  double = 1
        id_str              char = 'CAR_001'
        class_type          char = 'car'         % 'car', 'bike', 'auto', 'pedestrian', 'cattle'
        behavior_state      char = 'CRUISING'    % 'CRUISING', 'FOLLOWING', 'BRAKING', 'ACCELERATING', 'WALKING', 'CROSSING', 'GRAZING', 'EXITING'
        direction           double = 1           % +1 (forward), -1 (oncoming), 0 (crossing)
        entry_source        char = 'main_ahead'  % 'main_ahead', 'main_behind', 'left_edge', 'right_edge', 'side_road'
        
        % Kinematic State
        x                   double = 0.0
        y                   double = 0.0
        v                   double = 0.0
        theta               double = 0.0
        a                   double = 0.0
        delta               double = 0.0
        
        % Physical Dimensions
        length              double = 4.70
        width               double = 1.80
        wheelbase           double = 2.70
        
        % Desired Targets & Limits
        v_target            double = 8.0
        y_target            double = 3.0
        a_max               double = 3.0
        a_min               double = -4.0
        delta_max           double = 0.61
        delta_rate_max      double = 1.05
    end
    
    properties (Dependent)
        vx
        vy
        heading
    end
    
    properties
        % Crossing Parameters (Pedestrian / Cattle)
        crossing_target_y   double = 5.5
        crossing_speed      double = 1.0
        crossing_progress   double = 0.0
        wait_timer          double = 0.0
        
        % Reaction Delay Buffer
        tau_react           double = 0.15
        input_history       cell = {}
        
        % Active / Lifetime Status
        is_active           logical = true
        spawn_time          double = 0.0
        age                 double = 0.0
    end
    
    methods
        function val = get.vx(obj), val = obj.v * cos(obj.theta); end
        function val = get.vy(obj), val = obj.v * sin(obj.theta); end
        function val = get.heading(obj), val = obj.theta; end
        
        function set.vx(obj, val)
            vy_curr = obj.v * sin(obj.theta);
            obj.v = hypot(val, vy_curr);
            obj.theta = atan2(vy_curr, val + 1e-6);
        end
        function set.vy(obj, val)
            vx_curr = obj.v * cos(obj.theta);
            obj.v = hypot(vx_curr, val);
            obj.theta = atan2(val, vx_curr + 1e-6);
        end
        function set.heading(obj, val)
            obj.theta = val;
        end
        
        function obj = TrafficAgent(id, class_type, x, y, v, direction, varargin)
            if nargin >= 1 && ~isempty(id), obj.id = id; end
            if nargin >= 2 && ~isempty(class_type), obj.class_type = lower(class_type); end
            if nargin >= 3 && ~isempty(x), obj.x = x; end
            if nargin >= 4 && ~isempty(y), obj.y = y; end
            if nargin >= 5 && ~isempty(v), obj.v = v; end
            if nargin >= 6 && ~isempty(direction), obj.direction = direction; end
            
            % Generate ID string
            switch obj.class_type
                case 'car'
                    obj.id_str = sprintf('CAR_%03d', obj.id);
                case {'bike', 'motorcycle'}
                    obj.id_str = sprintf('BIKE_%03d', obj.id);
                    obj.class_type = 'bike';
                case {'auto', 'autorickshaw', 'auto_rickshaw'}
                    obj.id_str = sprintf('AUTO_%03d', obj.id);
                    obj.class_type = 'auto';
                case {'ped', 'pedestrian'}
                    obj.id_str = sprintf('PED_%03d', obj.id);
                    obj.class_type = 'pedestrian';
                case 'cattle'
                    obj.id_str = sprintf('CATTLE_%03d', obj.id);
                otherwise
                    obj.id_str = sprintf('AGENT_%03d', obj.id);
            end
            
            % Set class-specific physical dimensions and bounds
            obj.configureClassDefaults();
            
            % Set heading from direction
            if obj.direction == 1
                obj.theta = 0.0;
            elseif obj.direction == -1
                obj.theta = pi;
            else
                % Crossing: orient along lateral direction
                if nargin > 6 && ~isempty(varargin{1})
                    obj.crossing_target_y = varargin{1};
                end
                if obj.crossing_target_y >= obj.y
                    obj.theta = pi/2;
                else
                    obj.theta = -pi/2;
                end
            end
            
            % Parse optional parameters
            if nargin > 7 && ~isempty(varargin{2})
                obj.v_target = varargin{2};
            end
            if nargin > 8 && ~isempty(varargin{3})
                obj.entry_source = varargin{3};
            end
        end
        
        function configureClassDefaults(obj)
            switch obj.class_type
                case 'car'
                    obj.length = 4.70;
                    obj.width = 1.80;
                    obj.wheelbase = 2.70;
                    obj.v_target = max(5.0, min(12.0, obj.v));
                    obj.a_max = 3.0;
                    obj.a_min = -4.5;
                    obj.delta_max = deg2rad(35);
                    obj.delta_rate_max = deg2rad(60);
                    obj.behavior_state = 'CRUISING';
                    
                case 'bike'
                    obj.length = 1.90;
                    obj.width = 0.80;
                    obj.wheelbase = 1.35;
                    obj.v_target = max(4.5, min(10.0, obj.v));
                    obj.a_max = 3.5;
                    obj.a_min = -4.0;
                    obj.delta_max = deg2rad(40);
                    obj.delta_rate_max = deg2rad(90); % More agile
                    obj.behavior_state = 'CRUISING';
                    
                case 'auto'
                    obj.length = 2.70;
                    obj.width = 1.30;
                    obj.wheelbase = 2.00;
                    obj.v_target = max(3.5, min(7.0, obj.v));
                    obj.a_max = 1.8;
                    obj.a_min = -3.5;
                    obj.delta_max = deg2rad(32);
                    obj.delta_rate_max = deg2rad(50);
                    obj.behavior_state = 'CRUISING';
                    
                case 'pedestrian'
                    obj.length = 0.60;
                    obj.width = 0.60;
                    obj.wheelbase = 0.50;
                    obj.a_max = 1.5;
                    obj.a_min = -2.0;
                    if obj.direction == 0
                        obj.behavior_state = 'CROSSING';
                        obj.crossing_speed = max(0.8, min(1.4, obj.v));
                        obj.v = obj.crossing_speed;
                    else
                        obj.behavior_state = 'WALKING';
                        obj.v_target = max(0.9, min(1.4, obj.v));
                    end
                    
                case 'cattle'
                    obj.length = 2.20;
                    obj.width = 1.00;
                    obj.wheelbase = 1.50;
                    obj.a_max = 0.6;
                    obj.a_min = -1.0;
                    if obj.direction == 0
                        obj.behavior_state = 'CROSSING';
                        obj.crossing_speed = max(0.3, min(0.7, obj.v));
                        obj.v = obj.crossing_speed;
                    else
                        obj.behavior_state = 'GRAZING';
                        obj.v_target = 0.1;
                    end
            end
        end
        
        function stepOnline(obj, dt, road_geom, all_agents, ego_state)
            % STEPONLINE Advanced online state evolution based on local scene & behavior
            if ~obj.is_active, return; end
            obj.age = obj.age + dt;
            
            % Query local road geometry
            if ~isempty(road_geom)
                [y_center_r, theta_road, ~] = road_geom.getCenterline(obj.x);
                [y_min_r, y_max_r] = road_geom.getBounds(obj.x);
                [grade_ang, ~] = road_geom.getGrade(obj.x);
            else
                y_center_r = 3.0;
                theta_road = 0.0;
                y_min_r = 0.0;
                y_max_r = 6.0;
                grade_ang = 0.0;
            end
            
            % Dispatch behavior based on agent class
            switch obj.class_type
                case {'car', 'auto', 'bike'}
                    obj.stepVehicle(dt, y_center_r, theta_road, y_min_r, y_max_r, grade_ang, all_agents, ego_state);
                    
                case 'pedestrian'
                    obj.stepPedestrian(dt, y_center_r, y_min_r, y_max_r, all_agents, ego_state);
                    
                case 'cattle'
                    obj.stepCattle(dt, y_center_r, y_min_r, y_max_r, all_agents, ego_state);
            end
        end
        
        function stepVehicle(obj, dt, y_center_r, theta_road, y_min_r, y_max_r, grade_ang, all_agents, ego_state)
            % Longitudinal Car-Following / Speed Regulation + Lateral Path Keeping
            
            % 1. Determine lane target based on travel direction and class
            if obj.direction == 1
                % Same-direction (traveling +x): normally occupies right side of road
                if strcmp(obj.class_type, 'bike')
                    % Bikes prefer right edge corridor
                    obj.y_target = y_min_r + 1.20;
                else
                    % Cars and autos occupy right lane center
                    obj.y_target = y_min_r + (y_center_r - y_min_r) / 2.0 + 0.20;
                end
                theta_des = theta_road;
            else
                % Oncoming (traveling -x): occupies left side of road
                if strcmp(obj.class_type, 'bike')
                    obj.y_target = y_max_r - 1.20;
                else
                    obj.y_target = y_center_r + (y_max_r - y_center_r) / 2.0 - 0.20;
                end
                theta_des = theta_road + pi;
            end
            
            % 2. Detect lead vehicle in same travel direction
            lead_dist = inf;
            lead_v = obj.v;
            
            % Check other dynamic agents
            if ~isempty(all_agents)
                for i = 1:length(all_agents)
                    ag = all_agents(i);
                    if isempty(ag) || ag.id == obj.id || ~ag.is_active, continue; end
                    
                    if obj.direction == 1 && ag.x > obj.x
                        % Traffic / road user ahead in travel direction (+x)
                        dx = ag.x - obj.x - (obj.length + ag.length)/2;
                        dy = abs(ag.y - obj.y);
                        if dy <= (obj.width + ag.width)/2 + 0.6 && dx > 0 && dx < lead_dist
                            lead_dist = dx;
                            lead_v = max(0, ag.vx);
                        end
                    elseif obj.direction == -1 && ag.x < obj.x
                        % Traffic / road user ahead in oncoming travel direction (-x)
                        dx = obj.x - ag.x - (obj.length + ag.length)/2;
                        dy = abs(ag.y - obj.y);
                        if dy <= (obj.width + ag.width)/2 + 0.6 && dx > 0 && dx < lead_dist
                            lead_dist = dx;
                            lead_v = max(0, -ag.vx);
                        end
                    end
                end
            end
            
            % Also check ego if ego is ahead of this vehicle in its direction of travel
            if ~isempty(ego_state)
                if obj.direction == 1 && ego_state.x > obj.x
                    dx_ego = ego_state.x - obj.x - (obj.length + 4.7)/2;
                    dy_ego = abs(ego_state.y - obj.y);
                    if dy_ego <= (obj.width + 1.8)/2 + 0.8 && dx_ego > 0 && dx_ego < lead_dist
                        lead_dist = dx_ego;
                        lead_v = max(0, ego_state.v);
                    end
                elseif obj.direction == -1 && ego_state.x < obj.x
                    dx_ego = obj.x - ego_state.x - (obj.length + 4.7)/2;
                    dy_ego = abs(ego_state.y - obj.y);
                    if dy_ego <= (obj.width + 1.8)/2 + 0.8 && dx_ego > 0 && dx_ego < lead_dist
                        lead_dist = dx_ego;
                        lead_v = 0.0; % Ego stationary or opposing in front
                    end
                end
            end
            
            % 3. IDM Acceleration Calculation
            s0 = 3.5;       % Minimum spacing (m)
            T_gap = 1.3;    % Desired time headway (s)
            
            if lead_dist < 45.0
                s_curr = max(0.2, lead_dist);
                dv = obj.v - lead_v;
                s_star = s0 + max(0.0, obj.v * T_gap + (obj.v * dv) / (2 * sqrt(obj.a_max * abs(obj.a_min))));
                a_idm = obj.a_max * (1.0 - (obj.v / max(0.1, obj.v_target))^4 - (s_star / s_curr)^2);
                
                if a_idm < -1.0
                    obj.behavior_state = 'BRAKING';
                else
                    obj.behavior_state = 'FOLLOWING';
                end
            else
                % Free road speed tracking
                a_idm = 1.2 * (obj.v_target - obj.v);
                if a_idm > 0.5
                    obj.behavior_state = 'ACCELERATING';
                else
                    obj.behavior_state = 'CRUISING';
                end
            end
            
            % Apply longitudinal road grade gravity effect: -g * sin(grade_ang)
            % Forward vehicles experience deceleration uphill; oncoming experience acceleration
            a_grade = -9.81 * sin(grade_ang) * obj.direction;
            a_cmd = max(obj.a_min, min(obj.a_max, a_idm + a_grade));
            
            % 4. Lateral Steering (PD path keeping toward y_target)
            k_p = 0.20;
            k_d = 0.65;
            if strcmp(obj.class_type, 'bike')
                k_p = 0.25; k_d = 0.75; % Higher steering agility for bikes
            end
            
            y_err = obj.y - obj.y_target;
            theta_err = atan2(sin(obj.theta - theta_des), cos(obj.theta - theta_des));
            
            if obj.direction == 1
                delta_des = -k_d * theta_err - atan2(k_p * y_err, max(1.0, obj.v));
            else
                % Oncoming vehicle: steering polarity reflects reversed longitudinal motion
                delta_des = -k_d * theta_err + atan2(k_p * y_err, max(1.0, obj.v));
            end
            delta_cmd = max(-obj.delta_max, min(obj.delta_max, delta_des));
            
            % Rate-limit steering
            max_d_delta = obj.delta_rate_max * dt;
            delta_app = max(obj.delta - max_d_delta, min(obj.delta + max_d_delta, delta_cmd));
            
            % 5. Kinematic Bicycle Integration
            obj.x = obj.x + obj.v * cos(obj.theta) * dt;
            obj.y = obj.y + obj.v * sin(obj.theta) * dt;
            obj.v = max(0.0, obj.v + a_cmd * dt);
            
            yaw_rate = (obj.v / max(0.5, obj.wheelbase)) * tan(delta_app);
            obj.theta = atan2(sin(obj.theta + yaw_rate * dt), cos(obj.theta + yaw_rate * dt));
            obj.a = a_cmd;
            obj.delta = delta_app;
        end
        
        function stepPedestrian(obj, dt, y_center_r, y_min_r, y_max_r, all_agents, ego_state)
            % Pedestrian Kinematics: Walking along road OR Crossing across road
            if strcmp(obj.behavior_state, 'CROSSING')
                % Crossing logic
                target_y = obj.crossing_target_y;
                dy_rem = target_y - obj.y;
                
                % Scene awareness: check if ego or oncoming vehicle is very close
                ego_dist = inf;
                if ~isempty(ego_state)
                    ego_dist = hypot(obj.x - ego_state.x, obj.y - ego_state.y);
                end
                
                % If ego vehicle is within 6m and closing, pedestrian may briefly yield/wait
                if ego_dist < 6.0 && abs(obj.x - ego_state.x) < 4.5 && abs(dy_rem) > 0.5
                    obj.behavior_state = 'WAITING';
                    obj.v = 0.0;
                    return;
                else
                    obj.behavior_state = 'CROSSING';
                    obj.v = obj.crossing_speed;
                end
                
                % Check for crossing completion: once reached opposite shoulder, transition to walking or safe exit
                if abs(dy_rem) <= 0.25
                    if obj.y >= y_max_r || obj.y <= y_min_r
                        obj.behavior_state = 'EXITING';
                        obj.is_active = false;
                    else
                        obj.behavior_state = 'WALKING';
                        obj.direction = 1;
                    end
                    return;
                end
                
                crossing_dir = sign(dy_rem);
                vy = crossing_dir * obj.crossing_speed;
                % Small natural longitudinal drift (0.1 m/s)
                vx = 0.10 * cos(obj.x / 5.0);
                
                obj.x = obj.x + vx * dt;
                obj.y = obj.y + vy * dt;
                obj.theta = atan2(vy, vx);
                obj.a = 0.0;
            else
                % WALKING along road edge
                obj.behavior_state = 'WALKING';
                if obj.direction == 1
                    y_edge = y_min_r + 0.40;
                    theta_w = 0.0;
                else
                    y_edge = y_max_r - 0.40;
                    theta_w = pi;
                end
                
                % Emergent Crossing Decision:
                % If walking along edge for > 2.0s, decide to cross to opposite shoulder
                if obj.age > 2.0 && abs(obj.crossing_target_y - obj.y) > 2.5
                    % Check distance to ego before stepping into crossing
                    ego_gap = inf;
                    if ~isempty(ego_state)
                        ego_gap = hypot(obj.x - ego_state.x, obj.y - ego_state.y);
                    end
                    if ego_gap > 12.0 || abs(obj.x - ego_state.x) > 10.0
                        obj.behavior_state = 'CROSSING';
                        if obj.y < y_center_r
                            obj.crossing_target_y = y_max_r + 0.30;
                        else
                            obj.crossing_target_y = y_min_r - 0.30;
                        end
                        obj.v = obj.crossing_speed;
                        return;
                    end
                end
                
                % Lateral correction toward edge
                dy_edge = y_edge - obj.y;
                vy = max(-0.2, min(0.2, 0.5 * dy_edge));
                vx = obj.direction * obj.v_target;
                
                obj.x = obj.x + vx * dt;
                obj.y = obj.y + vy * dt;
                obj.theta = atan2(vy, vx);
                obj.v = hypot(vx, vy);
                obj.a = 0.0;
            end
        end
        
        function stepCattle(obj, dt, y_center_r, y_min_r, y_max_r, all_agents, ego_state)
            % Cattle Kinematics: Slow crossing or Roadside grazing
            if strcmp(obj.behavior_state, 'CROSSING')
                target_y = obj.crossing_target_y;
                dy_rem = target_y - obj.y;
                
                if abs(dy_rem) <= 0.30
                    % Finished crossing road corridor -> resume grazing on other side
                    obj.behavior_state = 'GRAZING';
                    obj.v = 0.05;
                    return;
                end
                
                crossing_dir = sign(dy_rem);
                vy = crossing_dir * obj.crossing_speed;
                % Minimal longitudinal drift
                vx = 0.05 * sin(obj.x / 10.0);
                
                obj.x = obj.x + vx * dt;
                obj.y = obj.y + vy * dt;
                obj.theta = atan2(vy, vx + 1e-5);
                obj.v = hypot(vx, vy);
                obj.a = 0.0;
            else
                % GRAZING near shoulder with very slow subtle wander
                obj.behavior_state = 'GRAZING';
                
                % Emergent crossing decision for grazing cattle after some time
                if obj.age > 3.0 && abs(obj.crossing_target_y - obj.y) > 2.0
                    ego_gap = inf;
                    if ~isempty(ego_state)
                        ego_gap = hypot(obj.x - ego_state.x, obj.y - ego_state.y);
                    end
                    if ego_gap > 15.0
                        obj.behavior_state = 'CROSSING';
                        if obj.y < y_center_r
                            obj.crossing_target_y = y_max_r + 0.35;
                        else
                            obj.crossing_target_y = y_min_r - 0.35;
                        end
                        obj.v = obj.crossing_speed;
                        return;
                    end
                end
                
                vx = 0.03 * cos(obj.age * 0.4);
                vy = 0.02 * sin(obj.age * 0.3);
                
                obj.x = obj.x + vx * dt;
                obj.y = obj.y + vy * dt;
                obj.theta = atan2(vy, vx + 1e-5);
                obj.v = hypot(vx, vy);
                obj.a = 0.0;
            end
        end
        
        function leg_agent = toLegacyAgent(obj)
            % Convert to legacy Agent object for complete downstream pipeline compatibility
            vx_val = obj.v * cos(obj.theta);
            vy_val = obj.v * sin(obj.theta);
            leg_agent = Agent(obj.id, obj.x, obj.y, vx_val, vy_val, 0.20);
            leg_agent.id_str = obj.id_str;
            leg_agent.type = obj.class_type;
            leg_agent.behavior_state = obj.behavior_state;
            leg_agent.direction = obj.direction;
            leg_agent.entry_source = obj.entry_source;
            leg_agent.length = obj.length;
            leg_agent.width = obj.width;
            leg_agent.v = obj.v;
            leg_agent.heading = obj.theta;
            leg_agent.a = obj.a;
        end
    end
end
