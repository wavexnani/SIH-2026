classdef BicycleModel
    % BICYCLEMODEL Kinematic and Dynamic Bicycle Model for Autonomous Vehicle
    %
    % Kinematic state: [x, y, theta, v, delta]
    %   x:     Longitudinal position (m)
    %   y:     Lateral position (m)
    %   theta: Heading angle (rad)
    %   v:     Speed (m/s)
    %   delta: Steering angle (rad)
    
    properties
        % Vehicle Physical Parameters
        L       double = 2.7            % Wheelbase (m)
        lr      double = 1.35           % Distance from CG to rear axle (m)
        lf      double = 1.35           % Distance from CG to front axle (m)
        m       double = 1500           % Mass (kg)
        Iz      double = 2500           % Yaw moment of inertia (kg·m²)
        Cf      double = 150000         % Front cornering stiffness (N/rad)
        Cr      double = 150000         % Rear cornering stiffness (N/rad)
        
        % Actuator & Performance Limits
        max_delta       double          % Max steering angle (rad)
        max_delta_rate  double          % Max steering rate (rad/s)
        max_accel       double = 3.0    % Max acceleration (m/s²)
        max_decel       double = -6.0   % Max deceleration (m/s²)
        max_speed       double = 20.0   % Speed limit (m/s)
        steering_time_constant double = 0.15  % First-order actuator lag (s)
        max_jerk        double = 8.0    % Acceleration slew limit (m/s^3)
    end
    
    methods
        function obj = BicycleModel(varargin)
            % Constructor - accepts optional SimulationConfig or wheelbase L
            if nargin > 0 && isa(varargin{1}, 'SimulationConfig')
                cfg = varargin{1};
                obj.L = cfg.wheelbase;
                obj.lr = obj.L / 2;
                obj.lf = obj.L / 2;
                obj.max_speed = cfg.v_max;
                obj.max_accel = cfg.a_max;
                obj.max_decel = cfg.a_min;
                obj.max_delta = cfg.delta_max;
                obj.max_delta_rate = cfg.delta_rate_max;
                obj.steering_time_constant = cfg.steering_time_constant;
                obj.max_jerk = cfg.max_jerk;
            else
                obj.max_delta = deg2rad(35);
                obj.max_delta_rate = deg2rad(60);
            end
        end
        
        function [x_dot, y_dot, theta_dot, v_dot, delta_dot] = kinematics(obj, x, y, theta, v, delta, a, delta_cmd, dt)
            % Kinematic Bicycle Model
            if nargin < 9
                dt = 0.1;
            end
            
            % Clamp control inputs
            a_clamped = max(min(a, obj.max_accel), obj.max_decel);
            delta_cmd_clamped = max(min(delta_cmd, obj.max_delta), -obj.max_delta);
            
            % Rate limit on steering angle change
            max_delta_step = obj.max_delta_rate * dt;
            delta_target = max(min(delta_cmd_clamped, delta + max_delta_step), delta - max_delta_step);
            
            % State derivatives
            x_dot = v * cos(theta);
            y_dot = v * sin(theta);
            theta_dot = (v / obj.L) * tan(delta);
            v_dot = a_clamped;
            delta_dot = (delta_target - delta) / max(dt, 1e-4);
        end
        
        function state_next = stepKinematic(obj, ego_state, a_cmd, delta_cmd, dt)
            % STEP KINEMATIC Apply a physically continuous actuator-limited step.
            %
            % The planner and safety filter remain unchanged and provide the
            % commanded [delta, a].  This simulation plant stores the applied
            % steering and acceleration in EgoState, limits their rates, and
            % integrates over the resulting constant-curvature arc.  It avoids
            % the former full-step lag where heading was propagated using the
            % previous steering angle and only then steering was updated.
            x = ego_state.x;
            y = ego_state.y;
            theta = ego_state.theta;
            v = ego_state.v;
            delta = ego_state.delta;
            a_applied = ego_state.a;
            
            % Command saturation and actuator continuity.
            a_target = max(min(a_cmd, obj.max_accel), obj.max_decel);
            a_new = obj.moveToward(a_applied, a_target, obj.max_jerk * dt);
            delta_target = max(min(delta_cmd, obj.max_delta), -obj.max_delta);
            delta_rate_limited = obj.moveToward(delta, delta_target, obj.max_delta_rate * dt);
            alpha = 1 - exp(-dt / max(obj.steering_time_constant, 1e-4));
            delta_new = delta + alpha * (delta_rate_limited - delta);

            % Midpoint inputs remove the numerical one-sample steering lag and
            % integrate the bicycle state along a constant-curvature arc.
            a_mid = 0.5 * (a_applied + a_new);
            delta_mid = 0.5 * (delta + delta_new);
            v_new = max(0, min(v + dt * a_mid, obj.max_speed));
            v_mid = 0.5 * (v + v_new);
            yaw_rate = (v_mid / obj.L) * tan(delta_mid);
            theta_mid = theta + 0.5 * dt * yaw_rate;
            x_new = x + dt * v_mid * cos(theta_mid);
            y_new = y + dt * v_mid * sin(theta_mid);
            theta_new = atan2(sin(theta + dt * yaw_rate), cos(theta + dt * yaw_rate));
            
            state_next = EgoState(x_new, y_new, theta_new, v_new, ego_state.t + dt);
            state_next = state_next.setControl(a_new, delta_new);
        end

        function state_next = stepLegacyKinematic(obj, ego_state, a_cmd, delta_cmd, dt)
            % STEPLEGACYKINEMATIC Exact pre-fidelity plant step for audit-only
            % before/after comparisons.  Production callers use stepKinematic.
            [x_dot, y_dot, theta_dot, v_dot, delta_dot] = obj.kinematics( ...
                ego_state.x, ego_state.y, ego_state.theta, ego_state.v, ...
                ego_state.delta, a_cmd, delta_cmd, dt);
            x_new = ego_state.x + dt * x_dot;
            y_new = ego_state.y + dt * y_dot;
            theta_new = atan2(sin(ego_state.theta + dt * theta_dot), ...
                              cos(ego_state.theta + dt * theta_dot));
            v_new = max(0, min(ego_state.v + dt * v_dot, obj.max_speed));
            delta_new = ego_state.delta + dt * delta_dot;
            state_next = EgoState(x_new, y_new, theta_new, v_new, ego_state.t + dt);
            state_next = state_next.setControl(a_cmd, delta_new);
        end
    end

    methods (Static, Access = private)
        function value = moveToward(current, target, max_step)
            value = current + max(-max_step, min(max_step, target - current));
        end
    end
end
