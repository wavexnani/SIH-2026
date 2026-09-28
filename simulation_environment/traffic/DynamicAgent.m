classdef DynamicAgent < TrafficAgent
    % DYNAMICAGENT Independently Simulated Dynamic Road User Model
    %
    % Backward-compatible wrapper extending TrafficAgent for heterogeneous
    % simulation (car, bike, auto, pedestrian, cattle).
    
    properties
        theta_target    double = 0.0     % Target heading angle (rad)
    end
    
    methods
        function obj = DynamicAgent(id, x, y, v, theta, v_target, y_target, theta_target, class_type)
            if nargin < 1, id = 1; end
            if nargin < 2, x = 0.0; end
            if nargin < 3, y = 3.0; end
            if nargin < 4, v = 0.0; end
            if nargin < 5 || isempty(theta), theta = 0.0; end
            if nargin < 9 || isempty(class_type), class_type = 'car'; end
            
            % Determine travel direction from initial heading
            dir = 1;
            if abs(cos(theta) - (-1.0)) < 0.3
                dir = -1;
            elseif abs(sin(theta)) > 0.7
                dir = 0;
            end
            
            obj@TrafficAgent(id, class_type, x, y, v, dir);
            obj.theta = theta;
            obj.a_min = -4.0;
            obj.tau_react = 0.20;
            
            if nargin >= 6 && ~isempty(v_target), obj.v_target = v_target; end
            if nargin >= 7 && ~isempty(y_target), obj.y_target = y_target; end
            if nargin >= 8 && ~isempty(theta_target), obj.theta_target = theta_target; end
        end
        
        function step(obj, dt, lead_x, lead_v)
            % STEP Advances dynamic state forward by time step dt
            if nargin < 3, lead_x = inf; end
            if nargin < 4, lead_v = obj.v; end
            
            % If heterogeneous stepOnline is not called directly, use legacy step
            if strcmp(obj.class_type, 'pedestrian') || strcmp(obj.class_type, 'cattle')
                obj.stepOnline(dt, [], [], []);
                return;
            end
            
            % 1. Longitudinal Acceleration Logic (IDM / Cruising)
            s0 = 3.0;      % Minimum gap (m)
            T_gap = 1.2;   % Desired time headway (s)
            
            if lead_x < inf && ((obj.direction == 1 && lead_x > obj.x) || (obj.direction == -1 && lead_x < obj.x))
                s_curr = max(0.1, abs(lead_x - obj.x) - obj.length);
                dv = obj.v - lead_v;
                s_star = s0 + max(0, obj.v * T_gap + (obj.v * dv) / (2 * sqrt(obj.a_max * abs(obj.a_min))));
                a_des = obj.a_max * (1 - (obj.v / max(0.1, obj.v_target))^4 - (s_star / s_curr)^2);
            else
                % Free-road speed control
                a_des = 1.5 * (obj.v_target - obj.v);
            end
            a_cmd = max(obj.a_min, min(obj.a_max, a_des));
            
            % 2. Lateral Steering Logic (PD Lane-Following)
            k_p = 0.15;
            k_d = 0.60;
            y_err = obj.y - obj.y_target;
            theta_err = atan2(sin(obj.theta - obj.theta_target), cos(obj.theta - obj.theta_target));
            delta_des = -k_d * theta_err - atan2(k_p * y_err, max(1.0, obj.v));
            delta_cmd = max(-obj.delta_max, min(obj.delta_max, delta_des));
            
            % 3. Apply Reaction Delay
            n_delay_steps = max(0, round(obj.tau_react / max(1e-4, dt)));
            obj.input_history{end+1} = [delta_cmd, a_cmd];
            
            if length(obj.input_history) > n_delay_steps
                u_apply = obj.input_history{1};
                obj.input_history(1) = [];
            else
                u_apply = [delta_cmd, a_cmd];
            end
            
            delta_app_raw = u_apply(1);
            a_app = u_apply(2);
            
            % Rate-limit steering application
            max_d_delta = obj.delta_rate_max * dt;
            delta_app = max(obj.delta - max_d_delta, min(obj.delta + max_d_delta, delta_app_raw));
            
            % Kinematic Motion State Integration
            obj.x = obj.x + obj.v * cos(obj.theta) * dt;
            obj.y = obj.y + obj.v * sin(obj.theta) * dt;
            obj.v = max(0.0, obj.v + a_app * dt);
            yaw_rate = (obj.v / obj.wheelbase) * tan(delta_app);
            obj.theta = atan2(sin(obj.theta + yaw_rate * dt), cos(obj.theta + yaw_rate * dt));
            obj.a = a_app;
            obj.delta = delta_app;
        end
        
        function state_vec = getStateVec(obj)
            % Returns [x, y, v, theta, a, delta]
            state_vec = [obj.x, obj.y, obj.v, obj.theta, obj.a, obj.delta];
        end
        
        function resetHistory(obj)
            obj.input_history = {};
        end
    end
end
