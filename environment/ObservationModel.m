classdef ObservationModel < handle
    % OBSERVATIONMODEL Perception Observation Pipeline for Dynamic Agents
    %
    % Purpose:
    %   Transforms ground-truth simulation state into observed state consumed by
    %   the ego vehicle's perception and planning pipeline. Supports deterministic
    %   noise and delay modes while preserving true ground truth separately.
    %
    % Modes:
    %   - 'ideal':   Identity mapping (Observed == Ground Truth, age = 0)
    %   - 'nominal': Deterministic zero-mean Gaussian noise with explicit RandStream seed
    %   - 'delayed': Deterministic perception delay buffer (tau_delay = 100 ms)
    
    properties
        mode            char = 'ideal'       % Mode: 'ideal', 'nominal', 'delayed'
        seed            double = 42          % Explicit seed for random generator
        sigma_pos       double = 0.15        % Position noise standard dev (m)
        sigma_vel       double = 0.20        % Velocity noise standard dev (m/s)
        sigma_theta     double = 0.02        % Heading noise standard dev (rad)
        tau_delay       double = 0.10        % Perception delay (s)
        
        rng_stream                           % Dedicated random stream object
        history_buffer  cell = {}            % Ground truth state buffer for delay
    end
    
    methods
        function obj = ObservationModel(mode, seed, varargin)
            if nargin >= 1 && ~isempty(mode), obj.mode = lower(mode); end
            if nargin >= 2 && ~isempty(seed), obj.seed = seed; end
            
            % Initialize dedicated RandStream for deterministic noise
            obj.rng_stream = RandStream('mt19937ar', 'Seed', obj.seed);
            
            p = inputParser;
            addParameter(p, 'sigma_pos', 0.15, @isnumeric);
            addParameter(p, 'sigma_vel', 0.20, @isnumeric);
            addParameter(p, 'sigma_theta', 0.02, @isnumeric);
            addParameter(p, 'tau_delay', 0.10, @isnumeric);
            if nargin > 2
                parse(p, varargin{:});
                obj.sigma_pos = p.Results.sigma_pos;
                obj.sigma_vel = p.Results.sigma_vel;
                obj.sigma_theta = p.Results.sigma_theta;
                obj.tau_delay = p.Results.tau_delay;
            end
        end
        
        function reset(obj)
            % Reset random generator stream and buffer
            obj.rng_stream = RandStream('mt19937ar', 'Seed', obj.seed);
            obj.history_buffer = {};
        end
        
        function setSeed(obj, new_seed)
            obj.seed = new_seed;
            obj.reset();
        end
        
        function [obs_world, obs_structs] = observe(obj, world, dt)
            % OBSERVE Transforms ground-truth world state into observed world state
            %
            % Input:
            %   world: Ground-truth WorldState object
            %   dt:    Simulation step size (s)
            %
            % Output:
            %   obs_world: Copy of WorldState containing observed dynamic agents
            %   obs_structs: Array of structured observation records with metadata
            
            if nargin < 3, dt = 0.1; end
            
            obs_world = world; % Shallow copy of WorldState
            obs_structs = struct('agent_id', {}, 'x', {}, 'y', {}, 'velocity', {}, ...
                                 'heading', {}, 'vx', {}, 'vy', {}, ...
                                 'length', {}, 'width', {}, 'timestamp', {}, 'age', {});
            
            if world.n_agents == 0
                return;
            end
            
            n_agents = world.n_agents;
            
            switch obj.mode
                case 'ideal'
                    for i = 1:n_agents
                        ag = world.agents(i);
                        if ag.id <= 0, continue; end
                        
                        speed = hypot(ag.vx, ag.vy);
                        heading = atan2(ag.vy, ag.vx + 1e-6);
                        
                        s.agent_id = ag.id;
                        s.x = ag.x;
                        s.y = ag.y;
                        s.velocity = speed;
                        s.heading = heading;
                        s.vx = ag.vx;
                        s.vy = ag.vy;
                        s.length = ag.length;
                        s.width = ag.width;
                        s.timestamp = world.t;
                        s.age = 0.0;
                        
                        obs_structs(i) = s;
                    end
                    % obs_world.agents remains identical to ground truth
                    
                case 'nominal'
                    for i = 1:n_agents
                        ag = world.agents(i);
                        if ag.id <= 0, continue; end
                        
                        % Draw deterministic noise from dedicated stream
                        n_pos = randn(obj.rng_stream, 1, 2) * obj.sigma_pos;
                        n_vel = randn(obj.rng_stream, 1, 1) * obj.sigma_vel;
                        n_th  = randn(obj.rng_stream, 1, 1) * obj.sigma_theta;
                        
                        true_speed = hypot(ag.vx, ag.vy);
                        true_heading = atan2(ag.vy, ag.vx + 1e-6);
                        
                        x_obs = ag.x + n_pos(1);
                        y_obs = ag.y + n_pos(2);
                        v_obs = max(0.0, true_speed + n_vel);
                        th_obs = atan2(sin(true_heading + n_th), cos(true_heading + n_th));
                        
                        vx_obs = v_obs * cos(th_obs);
                        vy_obs = v_obs * sin(th_obs);
                        
                        s.agent_id = ag.id;
                        s.x = x_obs;
                        s.y = y_obs;
                        s.velocity = v_obs;
                        s.heading = th_obs;
                        s.vx = vx_obs;
                        s.vy = vy_obs;
                        s.length = ag.length;
                        s.width = ag.width;
                        s.timestamp = world.t;
                        s.age = 0.0;
                        
                        obs_structs(i) = s;
                        
                        % Store observed Agent into obs_world
                        obs_ag = Agent(ag.id, x_obs, y_obs, vx_obs, vy_obs, ag.sigma);
                        obs_ag.length = ag.length;
                        obs_ag.width = ag.width;
                        obs_world.agents(i) = obs_ag;
                    end
                    
                case 'delayed'
                    % Buffer current ground truth agents
                    obj.history_buffer{end+1} = world.agents;
                    
                    n_delay_steps = max(1, round(obj.tau_delay / max(1e-4, dt)));
                    
                    if length(obj.history_buffer) > n_delay_steps
                        delayed_agents = obj.history_buffer{1};
                        obj.history_buffer(1) = [];
                        current_age = n_delay_steps * dt;
                    else
                        delayed_agents = obj.history_buffer{1};
                        current_age = (length(obj.history_buffer) - 1) * dt;
                    end
                    
                    obs_world.agents = delayed_agents;
                    
                    for i = 1:n_agents
                        ag = delayed_agents(i);
                        if ag.id <= 0, continue; end
                        
                        speed = hypot(ag.vx, ag.vy);
                        heading = atan2(ag.vy, ag.vx + 1e-6);
                        
                        s.agent_id = ag.id;
                        s.x = ag.x;
                        s.y = ag.y;
                        s.velocity = speed;
                        s.heading = heading;
                        s.vx = ag.vx;
                        s.vy = ag.vy;
                        s.length = ag.length;
                        s.width = ag.width;
                        s.timestamp = world.t - current_age;
                        s.age = current_age;
                        
                        obs_structs(i) = s;
                    end
                    
                case {'combined', 'combined_realistic'}
                    % Combined mode: Apply perception delay first, then apply Gaussian noise
                    obj.history_buffer{end+1} = world.agents;
                    n_delay_steps = max(1, round(obj.tau_delay / max(1e-4, dt)));
                    
                    if length(obj.history_buffer) > n_delay_steps
                        delayed_agents = obj.history_buffer{1};
                        obj.history_buffer(1) = [];
                        current_age = n_delay_steps * dt;
                    else
                        delayed_agents = obj.history_buffer{1};
                        current_age = (length(obj.history_buffer) - 1) * dt;
                    end
                    
                    for i = 1:n_agents
                        ag = delayed_agents(i);
                        if ag.id <= 0, continue; end
                        
                        n_pos = randn(obj.rng_stream, 1, 2) * obj.sigma_pos;
                        n_vel = randn(obj.rng_stream, 1, 1) * obj.sigma_vel;
                        n_th  = randn(obj.rng_stream, 1, 1) * obj.sigma_theta;
                        
                        true_speed = hypot(ag.vx, ag.vy);
                        true_heading = atan2(ag.vy, ag.vx + 1e-6);
                        
                        x_obs = ag.x + n_pos(1);
                        y_obs = ag.y + n_pos(2);
                        v_obs = max(0.0, true_speed + n_vel);
                        th_obs = atan2(sin(true_heading + n_th), cos(true_heading + n_th));
                        
                        vx_obs = v_obs * cos(th_obs);
                        vy_obs = v_obs * sin(th_obs);
                        
                        s.agent_id = ag.id;
                        s.x = x_obs;
                        s.y = y_obs;
                        s.velocity = v_obs;
                        s.heading = th_obs;
                        s.vx = vx_obs;
                        s.vy = vy_obs;
                        s.length = ag.length;
                        s.width = ag.width;
                        s.timestamp = world.t - current_age;
                        s.age = current_age;
                        
                        obs_structs(i) = s;
                        
                        obs_ag = Agent(ag.id, x_obs, y_obs, vx_obs, vy_obs, ag.sigma);
                        obs_ag.length = ag.length;
                        obs_ag.width = ag.width;
                        obs_world.agents(i) = obs_ag;
                    end
                    
                otherwise
                    error('Unknown ObservationModel mode: %s', obj.mode);
            end
        end
    end
end
