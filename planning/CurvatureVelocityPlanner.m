classdef CurvatureVelocityPlanner < handle
    % CURVATUREVELOCITYPLANNER Physics-consistent curvature-aware velocity profiling.
    %
    % Generates a smooth, physically feasible velocity profile along a reference
    % path based on local path curvature, lateral acceleration comfort limits,
    % forward acceleration limits, and backward pre-braking propagation.
    %
    % Mathematical Formulation:
    %   1. Parametric Curvature:
    %      kappa(s) = (x' * y'' - y' * x'') / (x'^2 + y'^2)^(3/2)
    %   2. Lateral Acceleration Limit:
    %      v_lat(s) = sqrt(a_lat_max / max(|kappa(s)|, eps_kappa))
    %   3. Backward Pre-Braking Propagation:
    %      v(s_i) <= sqrt(v(s_{i+1})^2 + 2 * a_decel_max * Delta_s_i)
    %   4. Forward Acceleration Smoothing:
    %      v(s_i) <= sqrt(v(s_{i-1})^2 + 2 * a_accel_max * Delta_s_{i-1})
    %
    % Default Parameters calibrated for full-scale passenger sedan:
    %   L = 2.7 m, mass = 1500 kg, comfort lateral accel = 2.5 m/s^2 (~0.25 g)
    
    properties
        a_lat_max       double = 2.50       % Maximum lateral acceleration (m/s^2) for passenger comfort
        a_accel_max     double = 2.00       % Maximum longitudinal acceleration (m/s^2)
        a_decel_max     double = 3.00       % Maximum service deceleration (m/s^2)
        v_max           double = 10.00      % Default upper speed limit (m/s)
        v_min           double = 1.50       % Minimum maneuvering speed through tight corners (m/s)
        eps_kappa       double = 1e-4       % Epsilon denominator to prevent div-by-zero on straights
        n_passes        int32 = 2           % Forward-backward smoothing iterations
    end
    
    methods
        function obj = CurvatureVelocityPlanner(varargin)
            % CURVATUREVELOCITYPLANNER Constructor
            % Accepts optional SimulationConfig or name-value pairs
            if nargin > 0 && isa(varargin{1}, 'SimulationConfig')
                cfg = varargin{1};
                obj.v_max = cfg.v_ref;
                if isprop(cfg, 'a_max'), obj.a_accel_max = min(2.5, cfg.a_max); end
                if isprop(cfg, 'a_min'), obj.a_decel_max = min(3.5, abs(cfg.a_min)); end
            end
        end
        
        function [kappa, s, headings] = computeCurvature(~, x, y)
            % COMPUTECURVATURE Computes cumulative arc length, heading, and curvature.
            %
            % Uses Menger curvature (circumscribed circle) for discrete waypoints.
            % Menger curvature is exact for circular arcs of any radius R, invariant
            % to rotation, and free of numerical gradient boundary attenuation.
            %
            % Inputs:
            %   x, y: Column vectors of path coordinates (meters)
            % Outputs:
            %   kappa:    Path curvature (1/m, rad/m)
            %   s:        Cumulative arc length (m)
            %   headings: Tangent heading angle theta (rad)
            
            x = x(:); y = y(:);
            N = length(x);
            if N < 3
                kappa = zeros(N, 1);
                s = zeros(N, 1);
                headings = zeros(N, 1);
                if N == 2
                    s = [0; hypot(x(2)-x(1), y(2)-y(1))];
                    headings = repmat(atan2(y(2)-y(1), x(2)-x(1)), 2, 1);
                end
                return;
            end
            
            % Arc length steps
            dx_step = diff(x);
            dy_step = diff(y);
            ds_step = hypot(dx_step, dy_step);
            ds_step = max(ds_step, 1e-6); % guard against identical consecutive points
            s = [0; cumsum(ds_step)];
            
            % Discrete Menger curvature
            kappa = zeros(N, 1);
            for i = 2:(N - 1)
                x1 = x(i-1); y1 = y(i-1);
                x2 = x(i);   y2 = y(i);
                x3 = x(i+1); y3 = y(i+1);
                
                a = hypot(x2 - x1, y2 - y1);
                b = hypot(x3 - x2, y3 - y2);
                c = hypot(x3 - x1, y3 - y1);
                
                cross_prod = (x2 - x1)*(y3 - y2) - (y2 - y1)*(x3 - x2);
                denom = a * b * c;
                if denom > 1e-9
                    kappa(i) = 2.0 * cross_prod / denom;
                end
            end
            kappa(1) = kappa(2);
            kappa(end) = kappa(end-1);
            
            % Tangent heading from centered differences
            headings = zeros(N, 1);
            headings(1) = atan2(y(2) - y(1), x(2) - x(1));
            headings(end) = atan2(y(end) - y(end-1), x(end) - x(end-1));
            for i = 2:(N - 1)
                headings(i) = atan2(y(i+1) - y(i-1), x(i+1) - x(i-1));
            end
        end
        
        function [aug_path, info] = planVelocityProfile(obj, path_in, v_cruise, v_end)
            % PLANVELOCITYPROFILE Generates full curvature-aware speed profile.
            %
            % Inputs:
            %   path_in:  [N x 2] (x, y), [N x 3] (x, y, theta), or [N x 5] matrix
            %   v_cruise: (optional) Maximum target cruising speed (m/s)
            %   v_end:    (optional) Terminal target speed (m/s, e.g. 0 for stop)
            %
            % Output:
            %   aug_path: [N x 5] matrix [x, y, theta, kappa, v_profile]
            %   info:     Diagnostic struct with curvature, limits, and profiles
            
            N = size(path_in, 1);
            if N == 0
                aug_path = zeros(0, 5);
                info = struct();
                return;
            end
            
            if nargin < 3 || isempty(v_cruise)
                v_cruise = obj.v_max;
            end
            if nargin < 4
                v_end = [];
            end
            
            x = path_in(:, 1);
            y = path_in(:, 2);
            
            % Compute or reuse curvature
            if size(path_in, 2) >= 4 && any(path_in(:, 4) ~= 0)
                kappa = path_in(:, 4);
                [~, s, headings] = obj.computeCurvature(x, y);
                if size(path_in, 2) >= 3
                    headings = path_in(:, 3);
                end
            else
                [kappa, s, headings] = obj.computeCurvature(x, y);
            end
            
            % 1. Pointwise Lateral Acceleration Constraint
            k_abs = abs(kappa);
            v_lat = sqrt(obj.a_lat_max ./ max(k_abs, obj.eps_kappa));
            
            % Unconstrained target speed profile clamped to [v_min, v_cruise]
            v_profile = min(v_cruise, max(obj.v_min, v_lat));
            
            % If curvature is negligible, use full cruise speed
            v_profile(k_abs < obj.eps_kappa * 5) = v_cruise;
            
            % Enforce terminal stop if requested
            if ~isempty(v_end)
                v_profile(end) = min(v_profile(end), v_end);
            end
            
            % Step distances
            ds = [diff(s); 0.1];
            ds = max(ds, 1e-4);
            
            % 2. Forward-Backward Integration Passes
            for pass = 1:double(obj.n_passes)
                % Backward Pre-Braking Pass:
                % Ensure vehicle begins braking on the straight before entering the curve
                for i = (N - 1):-1:1
                    ds_i = ds(i);
                    v_max_brake = sqrt(v_profile(i + 1)^2 + 2.0 * obj.a_decel_max * ds_i);
                    if v_profile(i) > v_max_brake
                        v_profile(i) = v_max_brake;
                    end
                end
                
                % Forward Acceleration Pass:
                % Ensure vehicle accelerates out of curves at a physically reasonable rate
                for i = 2:N
                    ds_prev = ds(i - 1);
                    v_max_accel = sqrt(v_profile(i - 1)^2 + 2.0 * obj.a_accel_max * ds_prev);
                    if v_profile(i) > v_max_accel
                        v_profile(i) = v_max_accel;
                    end
                end
            end
            
            % Assemble standard 5-column augmented reference path
            aug_path = zeros(N, 5);
            aug_path(:, 1) = x;
            aug_path(:, 2) = y;
            aug_path(:, 3) = headings;
            aug_path(:, 4) = kappa;
            aug_path(:, 5) = v_profile;
            
            % Compute achieved lateral acceleration profile
            a_lat_actual = (v_profile.^2) .* k_abs;
            
            % Diagnostic info
            info = struct();
            info.s = s;
            info.kappa = kappa;
            info.v_profile = v_profile;
            info.v_lat_raw = v_lat;
            info.a_lat_actual = a_lat_actual;
            info.min_speed = min(v_profile);
            info.max_speed = max(v_profile);
            info.max_a_lat = max(a_lat_actual);
            info.v_cruise_target = v_cruise;
            info.pre_braking_active = any(v_profile < v_cruise);
        end
    end
end
