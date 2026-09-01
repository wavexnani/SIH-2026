classdef Agent
    % AGENT Dynamic agent in the scenario
    %
    % State: [x, y, vx, vy] (position + velocity)
    % Simplified: no heading stored separately (derived from velocity)
    %
    % Usage:
    %   agent = Agent(id, x, y, vx, vy);
    %   agent.setVelocity(vx, vy);
    %   agent_new = agent.constantVelocityUpdate(dt);
    %   agent_new = agent.predictTrajectory(dt, N_steps);
    
    properties
        % Identity
        id              % Agent ID (1, 2, 3, ...)
        
        % State
        x               % Position X (m)
        y               % Position Y (m)
        vx              % Velocity X (m/s)
        vy              % Velocity Y (m/s)
        
        % Uncertainty
        sigma           % Position uncertainty (m)
        
        % Time
        t               % Current time (s)
        
        % Dimensions (for collision checking)
        length          % Agent length (assume ~4m like vehicle)
        width           % Agent width (assume ~2m like vehicle)
    end
    
    methods
        function obj = Agent(id, x, y, vx, vy, varargin)
            % Constructor
            % Agent(id, x, y, vx, vy) or Agent(id, x, y, vx, vy, sigma)
            
            obj.id = id;
            obj.x = x;
            obj.y = y;
            obj.vx = vx;
            obj.vy = vy;
            
            % Default uncertainty
            if nargin > 5
                obj.sigma = varargin{1};
            else
                obj.sigma = 0.2;  % meters
            end
            
            obj.t = 0;
            
            % Default dimensions
            obj.length = 4.7;
            obj.width = 1.8;
        end
        
        function obj = setVelocity(obj, vx, vy)
            % Set velocity
            obj.vx = vx;
            obj.vy = vy;
        end
        
        function obj = setUncertainty(obj, sigma)
            % Set position uncertainty
            obj.sigma = sigma;
        end
        
        function pos = getPosition(obj)
            % Get [x, y] position
            pos = [obj.x; obj.y];
        end
        
        function vel = getVelocity(obj)
            % Get [vx, vy] velocity
            vel = [obj.vx; obj.vy];
        end
        
        function speed = getSpeed(obj)
            % Get speed magnitude
            speed = sqrt(obj.vx^2 + obj.vy^2);
        end
        
        function state_vec = toVector(obj)
            % Convert to [x, y, vx, vy] vector
            state_vec = [obj.x; obj.y; obj.vx; obj.vy];
        end
        
        function obj = fromVector(obj, state_vec)
            % Load from [x, y, vx, vy] vector
            obj.x = state_vec(1);
            obj.y = state_vec(2);
            obj.vx = state_vec(3);
            obj.vy = state_vec(4);
        end
        
        function obj_new = constantVelocityUpdate(obj, dt)
            % Constant velocity motion model
            % x(t+dt) = x(t) + vx * dt
            % y(t+dt) = y(t) + vy * dt
            
            obj_new = obj;
            obj_new.x = obj.x + dt * obj.vx;
            obj_new.y = obj.y + dt * obj.vy;
            obj_new.t = obj.t + dt;
        end
        
        function trajectory = predictTrajectory(obj, dt, N_steps)
            % Predict trajectory over N_steps using constant velocity
            %
            % Input:
            %   dt:       time step (s)
            %   N_steps:  number of prediction steps
            %
            % Output:
            %   trajectory: (N_steps x 4) array of [x, y, vx, vy]
            
            trajectory = zeros(N_steps, 4);
            state = obj;
            
            for k = 1:N_steps
                state = state.constantVelocityUpdate(dt);
                trajectory(k, :) = state.toVector();
            end
        end
        
        function d = distanceTo(obj, other)
            % Euclidean distance to another agent or position
            if isa(other, 'Agent')
                pos_other = other.getPosition();
            else
                % Assume it's [x, y] position
                pos_other = other;
            end
            
            pos_self = obj.getPosition();
            d = norm(pos_self - pos_other);
        end
        
        function display(obj)
            % Display state
            speed = obj.getSpeed();
            fprintf('Agent %d: pos=(%.2f, %.2f), vel=(%.2f, %.2f) m/s (speed=%.2f m/s)\n', ...
                    obj.id, obj.x, obj.y, obj.vx, obj.vy, speed);
            fprintf('  Uncertainty: σ=%.2f m, t=%.2f s\n', obj.sigma, obj.t);
        end
    end
end
