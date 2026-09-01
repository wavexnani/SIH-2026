classdef DynamicAgent < handle
    % DYNAMICAGENT Independently Simulated Traffic Vehicle Model
    %
    % Purpose:
    %   Provides closed-loop dynamic behavior for non-ego vehicles in simulation.
    %   Dynamic agents evolve through physical kinematic bicycle equations rather than
    %   following scripted positions.
    %
    % Features:
    %   - 2nd-order longitudinal dynamics with Intelligent Driver Model (IDM) car-following
    %   - Lateral PD lane-keeping / path-following steering controller
    %   - Reaction delay buffer (tau_react)
    %   - Physical acceleration & steering rate bounds
    %   - Conversion helper to core Agent object for seamless WorldState integration
    
    properties
        id              double = 1
        x               double = 0.0
        y               double = 0.0
        v               double = 0.0
        theta           double = 0.0
        a               double = 0.0
        delta           double = 0.0
        
        % Desired vehicle targets & physical limits
        v_target        double = 8.0     % Target speed (m/s)
        y_target        double = 3.0     % Target lateral lane center (m)
        theta_target    double = 0.0     % Target heading angle (rad)
        a_max           double = 3.0     % Max acceleration (m/s^2)
        a_min           double = -4.0    % Max braking deceleration (m/s^2)
        delta_max       double = 0.61    % Max steering angle (~35 deg in rad)
        delta_rate_max  double = 1.05    % Max steering rate (~60 deg/s in rad/s)
        wheelbase       double = 2.70    % Wheelbase (m)
        length          double = 4.70    % Vehicle length (m)
        width           double = 1.80    % Vehicle width (m)
        tau_react       double = 0.20    % Reaction delay (s)
        
        % Internal state buffers for reaction delay
        input_history   cell = {}        % History queue for delayed commands
    end
    
    methods
        function obj = DynamicAgent(id, x, y, v, theta, v_target, y_target, theta_target)
            if nargin >= 1 && ~isempty(id), obj.id = id; end
            if nargin >= 2 && ~isempty(x), obj.x = x; end
            if nargin >= 3 && ~isempty(y), obj.y = y; end
            if nargin >= 4 && ~isempty(v), obj.v = v; end
            if nargin >= 5 && ~isempty(theta), obj.theta = theta; end
            if nargin >= 6 && ~isempty(v_target), obj.v_target = v_target; end
            if nargin >= 7 && ~isempty(y_target), obj.y_target = y_target; end
            if nargin >= 8 && ~isempty(theta_target), obj.theta_target = theta_target; end
        end
        
        function step(obj, dt, lead_x, lead_v)
            % STEP Advances vehicle dynamic state forward by time step dt
            if nargin < 3, lead_x = inf; end
            if nargin < 4, lead_v = obj.v; end
            
            % 1. Longitudinal Acceleration Logic (IDM / Cruising)
            s0 = 3.0;      % Minimum gap (m)
            T_gap = 1.2;   % Desired time headway (s)
            
            if lead_x < inf && lead_x > obj.x
                s_curr = max(0.1, lead_x - obj.x - obj.length); % Net bumper-to-bumper gap
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
            theta_err = obj.theta - obj.theta_target;
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
            
            % 4. Kinematic Motion State Integration
            obj.x = obj.x + obj.v * cos(obj.theta) * dt;
            obj.y = obj.y + obj.v * sin(obj.theta) * dt;
            obj.v = max(0.0, obj.v + a_app * dt);
            obj.theta = obj.theta + (obj.v / obj.wheelbase) * tan(delta_app) * dt;
            obj.theta = atan2(sin(obj.theta), cos(obj.theta)); % Normalize heading [-pi, pi]
            obj.a = a_app;
            obj.delta = delta_app;
        end
        
        function state_vec = getStateVec(obj)
            % Returns [x, y, v, theta, a, delta]
            state_vec = [obj.x, obj.y, obj.v, obj.theta, obj.a, obj.delta];
        end
        
        function leg_agent = toLegacyAgent(obj)
            % Converts DynamicAgent state to legacy Agent instance
            vx_val = obj.v * cos(obj.theta);
            vy_val = obj.v * sin(obj.theta);
            leg_agent = Agent(obj.id, obj.x, obj.y, vx_val, vy_val, 0.2);
            leg_agent.length = obj.length;
            leg_agent.width = obj.width;
        end
        
        function resetHistory(obj)
            obj.input_history = {};
        end
    end
end

