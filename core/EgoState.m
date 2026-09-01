classdef EgoState
    % EGOSTATE Vehicle state representation for ego vehicle
    %
    % State: [x, y, theta, v]
    %   x:     X position (along road) in meters
    %   y:     Y position (lateral) in meters
    %   theta: Heading angle in radians
    %   v:     Longitudinal speed in m/s
    %
    % Control: [a, delta]
    %   a:     Longitudinal acceleration in m/s²
    %   delta: Steering angle in radians
    %
    % Usage:
    %   ego = EgoState(x, y, theta, v);
    %   ego.setControl(a, delta);
    %   ego = ego.update(dt, model);
    
    properties
        % State
        x           % Position X (m)
        y           % Position Y (m)
        theta       % Heading (rad)
        v           % Speed (m/s)
        
        % Control
        a           % Acceleration (m/s²)
        delta       % Steering angle (rad)
        
        % Time
        t           % Current time (s)
    end
    
    methods
        function obj = EgoState(x, y, theta, v, varargin)
            % Constructor
            % EgoState(x, y, theta, v) or EgoState(x, y, theta, v, t)
            
            obj.x     = x;
            obj.y     = y;
            obj.theta = theta;
            obj.v     = v;
            obj.a     = 0;
            obj.delta = 0;
            
            if nargin > 4
                obj.t = varargin{1};
            else
                obj.t = 0;
            end
        end
        
        function obj = setControl(obj, a, delta)
            % Set control inputs
            % Saturate to reasonable limits
            obj.a = a;
            obj.delta = delta;
        end
        
        function state_vec = toVector(obj)
            % Convert to [x, y, theta, v] vector
            state_vec = [obj.x; obj.y; obj.theta; obj.v];
        end
        
        function obj = fromVector(obj, state_vec)
            % Load from [x, y, theta, v] vector
            obj.x     = state_vec(1);
            obj.y     = state_vec(2);
            obj.theta = state_vec(3);
            obj.v     = state_vec(4);
        end
        
        function pos = getPosition(obj)
            % Get [x, y] position
            pos = [obj.x; obj.y];
        end
        
        function vel = getVelocity(obj)
            % Get [vx, vy] velocity
            vel = [obj.v * cos(obj.theta); obj.v * sin(obj.theta)];
        end
        
        function obj_new = kinematicUpdate(obj, dt, L)
            % Kinematic bicycle model update
            % 
            % ẋ = v cos(θ)
            % ẏ = v sin(θ)
            % θ̇ = (v/L) tan(δ)
            % v̇ = a
            %
            % Input:
            %   dt: time step (s)
            %   L:  wheelbase (m)
            %
            % Output: updated EgoState
            
            obj_new = obj;
            
            % Avoid division by zero
            if abs(obj.v) < 1e-6
                L_eff = L;
            else
                L_eff = L;
            end
            
            % Kinematics
            theta_dot = (obj.v / L_eff) * tan(obj.delta);
            
            % Update state (Euler integration)
            obj_new.x     = obj.x + dt * obj.v * cos(obj.theta);
            obj_new.y     = obj.y + dt * obj.v * sin(obj.theta);
            obj_new.theta = obj.theta + dt * theta_dot;
            obj_new.v     = obj.v + dt * obj.a;
            
            % Normalize theta to [-pi, pi]
            obj_new.theta = atan2(sin(obj_new.theta), cos(obj_new.theta));
            
            % Update time
            obj_new.t = obj.t + dt;
        end
        
        function obj_new = predictTrajectory(obj, dt, L, N_steps, a_seq, delta_seq)
            % Predict trajectory over N_steps
            %
            % Input:
            %   dt:       time step (s)
            %   L:        wheelbase (m)
            %   N_steps:  number of prediction steps
            %   a_seq:    acceleration sequence (N_steps x 1) or scalar
            %   delta_seq: steering sequence (N_steps x 1) or scalar
            %
            % Output:
            %   trajectory: (N_steps x 4) array of [x, y, theta, v]
            
            trajectory = zeros(N_steps, 4);
            state = obj;
            
            for k = 1:N_steps
                % Get control for this step
                if isscalar(a_seq)
                    a_k = a_seq;
                else
                    a_k = a_seq(k);
                end
                
                if isscalar(delta_seq)
                    delta_k = delta_seq;
                else
                    delta_k = delta_seq(k);
                end
                
                % Apply control and update
                state = state.setControl(a_k, delta_k);
                state = state.kinematicUpdate(dt, L);
                
                % Store
                trajectory(k, :) = state.toVector();
            end
            
            obj_new = state;
        end
        
        function display(obj)
            % Display state
            fprintf('EgoState: x=%.2f, y=%.2f, θ=%.3f rad, v=%.2f m/s\n', ...
                    obj.x, obj.y, obj.theta, obj.v);
            fprintf('  Control: a=%.2f m/s², δ=%.3f rad\n', obj.a, obj.delta);
            fprintf('  Time: %.2f s\n', obj.t);
        end
    end
end
