classdef DelayAwareSteeringFilter < handle
    % DELAYAWARESTEERINGFILTER Actuator-aware steering command stabilization.
    %
    % Concept adapted from RoboRacer's steering_policy.py and calibrated for
    % full-scale automotive electric power steering (EPS) dynamics.
    %
    % Purpose:
    %   Eliminates high-frequency relay-like steering chatter and sign reversals
    %   induced by tracking noise or discretized optimization in narrow corridors,
    %   while preserving instantaneous authority for genuine emergency evasive maneuvers.
    %
    % Key Features:
    %   1. Rate Limiting: Bounds rate of change |Delta delta| <= delta_rate_max * dt.
    %   2. Micro-Deadband Hold: If commanded change from last held setpoint is
    %      smaller than deadband_rad, the command is held constant.
    %   3. Delay Arrival Prediction: Evaluates first-order EPS lag response.
    %   4. Emergency Bypass: If error exceeds emergency_threshold_rad or
    %      emergency flag is set by SafetyFilter, deadband is immediately bypassed.
    
    properties
        enabled                 logical = true       % Active state
        policy                  char    = 'HOLD_DEADBAND' % 'RAW', 'RATE_LIMITED', 'HOLD_DEADBAND'
        delta_max               double  = 0.6109     % Max steering angle (rad) ~ 35 deg
        delta_rate_max          double  = 0.50       % Max steering rate (rad/s) ~ 28.6 deg/s
        deadband_rad            double  = 0.015      % Deadband hold threshold (rad) ~ 0.86 deg
        tau_actuator            double  = 0.15       % EPS first-order response time constant (s)
        emergency_threshold_rad double  = 0.080      % Emergency bypass threshold (rad) ~ 4.6 deg
        dt                      double  = 0.10       % Control cycle period (s)
        
        % State Memory
        last_cmd                double  = 0.0        % Last emitted steering command (rad)
        last_actual             double  = 0.0        % Last observed physical wheel angle (rad)
        delta_est               double  = 0.0        % Internal actuator model state estimate (rad)
        is_held                 logical = false      % True if last step was held by deadband
        last_rate_sign          int32   = 0          % Sign of last command delta (+1, -1, 0)
        reversal_count          int32   = 0          % Cumulative count of steering sign reversals
        emergency_count         int32   = 0          % Cumulative count of emergency bypass events
        step_count              int32   = 0          % Cumulative steps processed
    end
    
    methods
        function obj = DelayAwareSteeringFilter(varargin)
            % Constructor
            if nargin > 0 && isa(varargin{1}, 'SimulationConfig')
                cfg = varargin{1};
                obj.dt = cfg.dt;
                obj.delta_max = cfg.delta_max;
                obj.delta_rate_max = cfg.delta_rate_max;
                if isprop(cfg, 'steering_time_constant')
                    obj.tau_actuator = cfg.steering_time_constant;
                end
            end
        end
        
        function reset(obj, initial_delta)
            if nargin < 2, initial_delta = 0.0; end
            obj.last_cmd = initial_delta;
            obj.last_actual = initial_delta;
            obj.delta_est = initial_delta;
            obj.is_held = false;
            obj.last_rate_sign = 0;
            obj.reversal_count = 0;
            obj.emergency_count = 0;
            obj.step_count = 0;
        end
        
        function delta_pred = predictArrival(obj, delta_curr, delta_cmd_in, dt_step)
            % PREDICTARRIVAL Predicts wheel angle after nominal actuator delay
            if nargin < 4, dt_step = obj.dt; end
            alpha = 1.0 - exp(-dt_step / max(obj.tau_actuator, 1e-4));
            delta_pred = delta_curr + alpha * (delta_cmd_in - delta_curr);
        end
        
        function [cmd_out, info] = step(obj, raw_cmd, current_actual, dt_step, emergency_override, corridor_width)
            % STEP Applies steering stabilization filter to raw controller output.
            %
            % Inputs:
            %   raw_cmd:            Raw commanded steering angle (rad)
            %   current_actual:     Current physical wheel angle (rad)
            %   dt_step:            (optional) Sample time step (s)
            %   emergency_override: (optional) Flag forcing immediate bypass
            %   corridor_width:     (optional) Available lateral corridor width (m)
            
            if nargin < 4 || isempty(dt_step), dt_step = obj.dt; end
            if nargin < 5 || isempty(emergency_override), emergency_override = false; end
            if nargin < 6 || isempty(corridor_width), corridor_width = inf; end
            
            obj.last_actual = current_actual;
            obj.step_count = obj.step_count + 1;
            
            % If disabled or RAW policy, pass through clamped command
            if ~obj.enabled || strcmpi(obj.policy, 'RAW')
                cmd_out = max(-obj.delta_max, min(obj.delta_max, raw_cmd));
                obj.last_cmd = cmd_out;
                obj.is_held = false;
                info = struct('cmd_out', cmd_out, 'is_held', false, 'is_emergency', false);
                return;
            end
            
            % Saturation check
            clamped_cmd = max(-obj.delta_max, min(obj.delta_max, raw_cmd));
            
            % Check for supervisor emergency override requiring instantaneous raw bypass
            delta_error = abs(clamped_cmd - current_actual);
            is_emergency = emergency_override;
            
            if is_emergency
                obj.emergency_count = obj.emergency_count + 1;
                % Full Emergency Bypass: grant immediate raw steering authority to evade collision
                cmd_out = clamped_cmd;
                obj.is_held = false;
            else
                % 1. Slew Rate Limiting
                max_step = obj.delta_rate_max * dt_step;
                delta_diff = clamped_cmd - obj.last_cmd;
                slew_limited = obj.last_cmd + max(-max_step, min(max_step, delta_diff));
                
                % 2. Adaptive Deadband (scales down in narrow corridors < 2.5m)
                if strcmpi(obj.policy, 'HOLD_DEADBAND')
                    deadband_eff = obj.deadband_rad;
                    if corridor_width < 2.50
                        deadband_eff = obj.deadband_rad * max(0.0, min(1.0, (corridor_width - 1.80) / (2.50 - 1.80)));
                    end
                    
                    change_from_last = abs(slew_limited - obj.last_cmd);
                    % Hold if within deadband and tracking error has not exceeded emergency threshold
                    if (change_from_last < deadband_eff) && (delta_error < obj.emergency_threshold_rad)
                        % Within deadband: hold previous command to avoid relay chatter
                        cmd_out = obj.last_cmd;
                        obj.is_held = true;
                    else
                        cmd_out = slew_limited;
                        obj.is_held = false;
                    end
                else
                    cmd_out = slew_limited;
                    obj.is_held = false;
                end
            end
            
            % 3. Track command rate sign reversals
            d_cmd = cmd_out - obj.last_cmd;
            if abs(d_cmd) > 1e-5
                cur_sign = int32(sign(d_cmd));
                if obj.last_rate_sign ~= 0 && cur_sign ~= obj.last_rate_sign
                    obj.reversal_count = obj.reversal_count + 1;
                end
                obj.last_rate_sign = cur_sign;
            end
            
            % 4. Update Internal Actuator Model Estimate
            obj.delta_est = obj.predictArrival(obj.delta_est, cmd_out, dt_step);
            obj.last_cmd = cmd_out;
            
            % Diagnostics
            info = struct();
            info.cmd_out = cmd_out;
            info.raw_cmd = raw_cmd;
            info.is_held = obj.is_held;
            info.is_emergency = is_emergency;
            info.delta_est = obj.delta_est;
            info.reversal_count = obj.reversal_count;
        end
    end
end
