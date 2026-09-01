classdef VehicleController < handle
    % VEHICLECONTROLLER Closed-Loop Controller (PID Speed + Stanley Lateral)
    %
    % Controls:
    %   1. Longitudinal PID speed controller for acceleration command (a_cmd)
    %   2. Stanley method lateral controller for steering angle command (delta_cmd)
    
    properties
        % Speed PID Controller Parameters
        speed_kp            double = 1.5
        speed_ki            double = 0.05
        speed_kd            double = 0.2
        integral_error      double = 0.0
        prev_speed_error    double = 0.0
        
        % Stanley Controller Parameters
        stanley_k           double = 1.1   % Cross-track gain (re-validated & locked spec)
        stanley_softening   double = 1.0   % Softening velocity constant (m/s)
        
        % Control Limits
        max_accel           double = 3.0
        max_decel           double = -6.0
        max_steering        double = 0.6109  % deg2rad(35)
    end
    
    methods
        function obj = VehicleController(varargin)
            % Constructor
            if nargin > 0 && isa(varargin{1}, 'SimulationConfig')
                cfg = varargin{1};
                obj.max_accel = cfg.a_max;
                obj.max_decel = cfg.a_min;
                obj.max_steering = cfg.delta_max;
            else
                obj.max_steering = deg2rad(35);
            end
        end
        
        function accel_cmd = computeSpeedControl(obj, v_current, v_target, dt)
            % Compute longitudinal acceleration command using PID
            error = v_target - v_current;
            obj.integral_error = obj.integral_error + error * dt;
            
            % Prevent integral windup
            obj.integral_error = max(min(obj.integral_error, 10.0), -10.0);
            
            deriv = (error - obj.prev_speed_error) / max(dt, 1e-4);
            obj.prev_speed_error = error;
            
            accel_raw = obj.speed_kp * error + obj.speed_ki * obj.integral_error + obj.speed_kd * deriv;
            accel_cmd = max(min(accel_raw, obj.max_accel), obj.max_decel);
        end
        
        function [delta_cmd, crosstrack_error, heading_error] = computeStanleyControl(obj, ego_state, reference_path)
            % Compute lateral steering angle command using Stanley method
            % reference_path: [N x 2] or [N x 3] matrix [x, y, theta]
            
            x = ego_state.x;
            y = ego_state.y;
            theta = ego_state.theta;
            v = ego_state.v;
            
            ref_xy = reference_path(:, 1:2);
            
            % Find nearest point on reference path
            dists = (ref_xy(:, 1) - x).^2 + (ref_xy(:, 2) - y).^2;
            [~, nearest_idx] = min(dists);
            
            ref_x = reference_path(nearest_idx, 1);
            ref_y = reference_path(nearest_idx, 2);
            
            % Reference heading
            if size(reference_path, 2) >= 3
                ref_theta = reference_path(nearest_idx, 3);
            else
                if nearest_idx < size(reference_path, 1)
                    ref_theta = atan2(reference_path(nearest_idx+1, 2) - ref_y, ...
                                      reference_path(nearest_idx+1, 1) - ref_x);
                elseif nearest_idx > 1
                    ref_theta = atan2(ref_y - reference_path(nearest_idx-1, 2), ...
                                      ref_x - reference_path(nearest_idx-1, 1));
                else
                    ref_theta = 0;
                end
            end
            
            % Vector from reference point to ego position
            dx = x - ref_x;
            dy = y - ref_y;
            
            % Cross-track error (perpendicular to path direction)
            crosstrack_error = dy * cos(ref_theta) - dx * sin(ref_theta);
            
            % Heading error
            heading_error = theta - ref_theta;
            heading_error = atan2(sin(heading_error), cos(heading_error));
            
            % Stanley Control Law
            % delta = -heading_error + atan2(-k * e_y, v + v_soft)
            stanley_term = atan2(-obj.stanley_k * crosstrack_error, v + obj.stanley_softening);
            delta_raw = -heading_error + stanley_term;
            
            % Normalize and clamp steering command
            delta_raw = atan2(sin(delta_raw), cos(delta_raw));
            delta_cmd = max(min(delta_raw, obj.max_steering), -obj.max_steering);
        end
        
        function reset(obj)
            % Reset integral state
            obj.integral_error = 0;
            obj.prev_speed_error = 0;
        end
    end
end
