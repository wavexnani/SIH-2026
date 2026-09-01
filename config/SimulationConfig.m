classdef SimulationConfig
    % SIMULATIONCONFIG Global configuration for SIH simulation
    %
    % Stage 0: World model infrastructure
    % Contains all parameters used across stages
    %
    % Usage:
    %   cfg = SimulationConfig();
    %   dt = cfg.dt;
    %   T_horizon = cfg.T_horizon;
    
    properties
        % =====================================================
        % TIME PARAMETERS
        % =====================================================
        dt              % Simulation time step (0.1 s)
        T_horizon       % Prediction horizon (10 s)
        T_replan        % Replanning interval (0.5 s)
        T_sim_max       % Maximum simulation time (100 s)
        
        % =====================================================
        % VEHICLE PARAMETERS (Standard Sedan)
        % =====================================================
        
        % Dimensions (meters)
        vehicle_length      % L = 4.7 m
        vehicle_width       % W = 1.8 m
        wheelbase            % lr = 2.7 m (distance from rear axle to CoG)
        vehicle_mass        % m = 1500 kg (typical sedan)
        
        % Collision radii (used for safety checks)
        % Circular footprint approximation: r = sqrt(L^2 + W^2)/2
        collision_radius    % Half-diagonal of bounding box
        safety_margin       % Extra margin beyond collision radius
        
        % Speed limits (m/s)
        v_max               % Maximum speed = 20 m/s (~72 km/h)
        v_min               % Minimum speed = -5 m/s (reverse)
        
        % Acceleration limits (m/s²)
        a_max               % Max forward acceleration = 6 m/s²
        a_min               % Max reverse acceleration = -6 m/s²
        
        % Steering limits
        delta_max           % Max steering angle = ±35° (radians)
        delta_rate_max      % Max steering rate = 0.5 rad/s

        % Plant-only actuator dynamics.  These are deliberately kept out of
        % the frozen planner formulation: they describe how a commanded input
        % becomes an applied vehicle input in the simulation.
        steering_time_constant  % First-order steering actuator time constant (s)
        max_jerk                % Maximum longitudinal acceleration slew (m/s^3)
        
        % =====================================================
        % ROAD PARAMETERS
        % =====================================================
        
        road_length         % Road length = 100 m
        road_width          % Road width = 6 m
        road_center_y       % Road center (Y = 3 m)
        
        % =====================================================
        % SCENARIO PARAMETERS
        % =====================================================
        
        % Ego vehicle initial state [x, y, theta, v]
        ego_x_init          % Initial X position = 10 m
        ego_y_init          % Initial Y position = 3 m (center)
        ego_theta_init      % Initial heading = 0 rad (along road)
        ego_v_init          % Initial speed = 0 m/s
        
        % Reference velocity (for speed controller)
        v_ref               % Reference speed = 10 m/s
        
        % Number of agents and obstacles
        n_agents            % Number of dynamic agents = 3
        n_static_obs        % Number of static obstacles = 4
        
        % =====================================================
        % UNCERTAINTY & PREDICTION
        % =====================================================
        
        sigma_agent         % Agent position uncertainty (σ) = 0.3 m
        beta_uncertainty    % Uncertainty inflation factor = 1.5
        d_safe_threshold    % Minimum safe distance = 0.5 m
        
        % =====================================================
        % LOGGING & OUTPUT
        % =====================================================
        
        log_enabled         % Enable detailed logging (true)
        log_frequency       % Log every N steps (1 = every step)
        verbose             % Print progress to console (true)
        plot_enabled        % Enable visualization (true)
        
    end
    
    methods
        function obj = SimulationConfig()
            % Constructor: Initialize all parameters with default values
            
            % TIME
            obj.dt              = 0.1;           % seconds
            obj.T_horizon       = 10.0;          % seconds
            obj.T_replan        = 0.5;           % seconds
            obj.T_sim_max       = 100.0;         % seconds
            
            % VEHICLE DIMENSIONS
            obj.vehicle_length  = 4.7;           % meters
            obj.vehicle_width   = 1.8;           % meters
            obj.wheelbase        = 2.7;          % meters
            obj.vehicle_mass    = 1500;          % kg
            
            % Collision radius (half-diagonal of bounding box)
            obj.collision_radius = sqrt(obj.vehicle_length^2 + obj.vehicle_width^2) / 2;
            obj.safety_margin   = 0.2;           % extra safety margin (meters)
            
            % SPEED LIMITS
            obj.v_max           = 20.0;          % m/s (~72 km/h)
            obj.v_min           = -5.0;          % m/s (reverse)
            
            % ACCELERATION LIMITS
            obj.a_max           = 6.0;           % m/s²
            obj.a_min           = -6.0;          % m/s²
            
            % STEERING LIMITS
            obj.delta_max       = deg2rad(35);   % radians
            obj.delta_rate_max  = 0.5;           % rad/s
            obj.steering_time_constant = 0.15;   % s, representative EPS response
            obj.max_jerk        = 8.0;           % m/s^3, plant input continuity
            
            % ROAD
            obj.road_length     = 150.0;         % meters
            obj.road_width      = 6.0;           % meters
            obj.road_center_y   = 3.0;           % Y = 3 m (center of 6m wide road)
            
            % SCENARIO
            obj.ego_x_init      = 10.0;          % Start at 10m along road
            obj.ego_y_init      = 3.0;           % Start at center
            obj.ego_theta_init  = 0.0;           % Along road
            obj.ego_v_init      = 0.0;           % Start stopped
            
            obj.v_ref           = 10.0;          % Reference speed (m/s)
            
            obj.n_agents        = 3;             % 3 dynamic agents
            obj.n_static_obs    = 4;             % 4 static obstacles
            
            % UNCERTAINTY
            obj.sigma_agent     = 0.3;           % meters
            obj.beta_uncertainty = 1.5;          % inflation factor
            obj.d_safe_threshold = 0.5;          % meters
            
            % LOGGING
            obj.log_enabled     = true;
            obj.log_frequency   = 1;             % every step
            obj.verbose         = true;
            obj.plot_enabled    = true;
        end
        
        function N_horizon = getHorizonSteps(obj)
            % Number of steps in prediction horizon
            N_horizon = ceil(obj.T_horizon / obj.dt);
        end
        
        function N_replan = getReplanSteps(obj)
            % Number of steps between replanning cycles
            N_replan = ceil(obj.T_replan / obj.dt);
        end
        
        function N_total = getTotalSteps(obj)
            % Total steps in simulation
            N_total = ceil(obj.T_sim_max / obj.dt);
        end
        
        function r_safe = getSafeRadius(obj)
            % Total safe radius (collision + margin)
            r_safe = obj.collision_radius + obj.safety_margin;
        end
        
        function display(obj)
            % Display configuration
            fprintf('\n=== SIMULATION CONFIG ===\n');
            fprintf('Time step:          %.2f s\n', obj.dt);
            fprintf('Horizon:            %.1f s (%d steps)\n', obj.T_horizon, obj.getHorizonSteps());
            fprintf('Replan interval:    %.1f s (%d steps)\n', obj.T_replan, obj.getReplanSteps());
            fprintf('\nVehicle:\n');
            fprintf('  Dimensions:       %.1f m x %.1f m\n', obj.vehicle_length, obj.vehicle_width);
            fprintf('  Collision radius: %.2f m\n', obj.collision_radius);
            fprintf('  Speed limit:      %.1f m/s\n', obj.v_max);
            fprintf('  Accel limit:      %.1f m/s²\n', obj.a_max);
            fprintf('\nRoad:\n');
            fprintf('  Length x Width:   %.0f m x %.1f m\n', obj.road_length, obj.road_width);
            fprintf('  Center Y:         %.1f m\n', obj.road_center_y);
            fprintf('\nScenario:\n');
            fprintf('  Ego start:        (%.1f, %.1f, %.2f rad, %.1f m/s)\n', ...
                    obj.ego_x_init, obj.ego_y_init, obj.ego_theta_init, obj.ego_v_init);
            fprintf('  Agents:           %d\n', obj.n_agents);
            fprintf('  Static obs:       %d\n', obj.n_static_obs);
            fprintf('\n');
        end
    end
end
