classdef UncertaintyModel < handle
    % UNCERTAINTYMODEL Configurable Perception & Actuator Uncertainty Layer
    %
    % Purpose:
    %   Introduces switchable, deterministic feedback noise, latency delays, and
    %   actuator tracking offsets into the closed-loop evaluation pipeline to test controller robustness.
    %
    % Supported Modes:
    %   'ideal', 'nominal', 'noisy_perception', 'delayed_perception', 'actuator_uncertainty', 'combined_realistic'
    
    properties
        mode            char = 'ideal'
        seed            double = 42
        
        % Noise parameters
        sigma_pos       double = 0.0
        sigma_vel       double = 0.0
        sigma_theta     double = 0.0
        delay_steps     double = 0
        steer_bias      double = 0.0
        
        % State delay buffer
        state_queue     cell = {}
    end
    
    methods
        function obj = UncertaintyModel(mode_str, seed_val)
            if nargin >= 1 && ~isempty(mode_str), obj.mode = lower(mode_str); end
            if nargin >= 2 && ~isempty(seed_val), obj.seed = seed_val; end
            
            rng(obj.seed); % Enforce deterministic seed
            obj.configureMode();
        end
        
        function configureMode(obj)
            switch obj.mode
                case 'ideal'
                    obj.sigma_pos = 0.0; obj.sigma_vel = 0.0; obj.sigma_theta = 0.0;
                    obj.delay_steps = 0; obj.steer_bias = 0.0;
                case 'nominal'
                    obj.sigma_pos = 0.05; obj.sigma_vel = 0.10; obj.sigma_theta = 0.005;
                    obj.delay_steps = 0; obj.steer_bias = 0.0;
                case 'noisy_perception'
                    obj.sigma_pos = 0.20; obj.sigma_vel = 0.25; obj.sigma_theta = 0.030;
                    obj.delay_steps = 0; obj.steer_bias = 0.0;
                case 'delayed_perception'
                    obj.sigma_pos = 0.0; obj.sigma_vel = 0.0; obj.sigma_theta = 0.0;
                    obj.delay_steps = 1; obj.steer_bias = 0.0;
                case 'actuator_uncertainty'
                    obj.sigma_pos = 0.0; obj.sigma_vel = 0.0; obj.sigma_theta = 0.0;
                    obj.delay_steps = 0; obj.steer_bias = deg2rad(1.0); % 1 deg offset
                case 'combined_realistic'
                    obj.sigma_pos = 0.15; obj.sigma_vel = 0.20; obj.sigma_theta = 0.020;
                    obj.delay_steps = 1; obj.steer_bias = deg2rad(0.8);
                otherwise
                    error('UncertaintyModel: Unknown mode %s', obj.mode);
            end
        end
        
        function world_obs = observeWorld(obj, world_true)
            % OBSERVEWORLD Returns an observed WorldState with noise/delay applied
            world_obs = world_true;
            
            % Add delay to ego observation if configured
            if obj.delay_steps > 0
                obj.state_queue{end+1} = world_true.ego;
                if length(obj.state_queue) > obj.delay_steps
                    ego_apply = obj.state_queue{1};
                    obj.state_queue(1) = [];
                else
                    ego_apply = world_true.ego;
                end
                world_obs.ego = ego_apply;
            end
            
            % Add Gaussian noise to perception
            if obj.sigma_pos > 0 || obj.sigma_vel > 0 || obj.sigma_theta > 0
                world_obs.ego.x = world_obs.ego.x + obj.sigma_pos * randn();
                world_obs.ego.y = world_obs.ego.y + obj.sigma_pos * randn();
                world_obs.ego.v = max(0.0, world_obs.ego.v + obj.sigma_vel * randn());
                world_obs.ego.theta = world_obs.ego.theta + obj.sigma_theta * randn();
            end
        end
        
        function u_perturbed = perturbControl(obj, u_cmd)
            % PERTURBCONTROL Applies actuator bias/uncertainty to command [delta; a]
            u_perturbed = u_cmd;
            if obj.steer_bias ~= 0
                u_perturbed(1) = u_cmd(1) + obj.steer_bias;
            end
        end
    end
end
