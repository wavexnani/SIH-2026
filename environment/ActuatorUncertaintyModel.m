classdef ActuatorUncertaintyModel < handle
    % ACTUATORUNCERTAINTYMODEL Ego Vehicle Actuator Command Uncertainty Layer
    %
    % Purpose:
    %   Applies physically credible, deterministic actuator variations (e.g. steering bias)
    %   between the frozen planner/controller output and the BicycleModel plant input.
    %   Preserves physical saturation limits and existing plant lag/slew-rate bounds.
    %
    % Modes:
    %   - 'ideal':         Identity mapping (u_actual == u_cmd)
    %   - 'nominal':       Deterministic steering bias (+0.8 deg) with saturation limits
    %   - 'steering_bias': Deterministic steering bias (+0.8 deg) with saturation limits
    %   - 'combined':      Steering bias (+0.8 deg) + plant dynamics integration
    
    properties
        mode                char = 'ideal'       % Mode: 'ideal', 'nominal', 'steering_bias', 'combined'
        steering_bias_deg   double = 0.8         % Steering bias in degrees (+0.8 deg default)
        max_delta           double               % Steering angle limit (rad)
        max_accel           double = 3.0         % Acceleration limit (m/s^2)
        max_decel           double = -6.0        % Deceleration limit (m/s^2)
    end
    
    methods
        function obj = ActuatorUncertaintyModel(mode, varargin)
            if nargin >= 1 && ~isempty(mode), obj.mode = lower(mode); end
            
            p = inputParser;
            addParameter(p, 'steering_bias_deg', 0.8, @isnumeric);
            addParameter(p, 'max_delta', deg2rad(35), @isnumeric);
            addParameter(p, 'max_accel', 3.0, @isnumeric);
            addParameter(p, 'max_decel', -6.0, @isnumeric);
            if nargin > 1
                parse(p, varargin{:});
                obj.steering_bias_deg = p.Results.steering_bias_deg;
                obj.max_delta = p.Results.max_delta;
                obj.max_accel = p.Results.max_accel;
                obj.max_decel = p.Results.max_decel;
            else
                obj.max_delta = deg2rad(35);
            end
        end
        
        function reset(~)
            % Stateless deterministic model reset helper
        end
        
        function [u_actual, info] = process(obj, u_cmd, cfg)
            % PROCESS Process control command u_cmd = [delta_cmd, a_cmd]
            %
            % Inputs:
            %   u_cmd: 1x2 or 2x1 vector [delta_cmd, a_cmd] (rad, m/s^2)
            %   cfg:   Optional SimulationConfig object for limit overriding
            %
            % Outputs:
            %   u_actual: 1x2 vector [delta_actual, a_actual]
            %   info:     Struct containing command vs actual telemetry and errors
            
            if nargin >= 3 && ~isempty(cfg)
                obj.max_delta = cfg.delta_max;
                obj.max_accel = cfg.a_max;
                obj.max_decel = cfg.a_min;
            end
            
            delta_cmd = u_cmd(1);
            a_cmd = u_cmd(2);
            
            switch obj.mode
                case 'ideal'
                    delta_actual = delta_cmd;
                    a_actual = a_cmd;
                    
                case {'nominal', 'steering_bias', 'combined'}
                    bias_rad = deg2rad(obj.steering_bias_deg);
                    % Apply bias to commanded steering angle
                    delta_biased = delta_cmd + bias_rad;
                    % Saturate at physical steering limits
                    delta_actual = max(min(delta_biased, obj.max_delta), -obj.max_delta);
                    % Saturate acceleration at physical bounds
                    a_actual = max(min(a_cmd, obj.max_accel), obj.max_decel);
                    
                otherwise
                    error('Unknown ActuatorUncertaintyModel mode: %s', obj.mode);
            end
            
            u_actual = [delta_actual, a_actual];
            
            info.delta_cmd = delta_cmd;
            info.delta_actual = delta_actual;
            info.a_cmd = a_cmd;
            info.a_actual = a_actual;
            info.steering_error = delta_actual - delta_cmd;
            info.acceleration_error = a_actual - a_cmd;
        end
    end
end
