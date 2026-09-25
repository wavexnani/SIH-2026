classdef RoadGeometry < handle
    % ROADGEOMETRY Physically Grounded Unstructured Village Road Geometry
    %
    % Represents:
    %   - Road Centerline with analytic/parametric curvature: y_c(x), theta_road(x), kappa(x)
    %   - Irregular and curved lateral boundaries: y_min(x), y_max(x)
    %   - Longitudinal elevation profile and grade: z(x), theta_grade(x) (uphill / downhill)
    %   - Physical road defects and potholes: [x, y, length, width, depth, severity]
    
    properties
        road_type           char = 'straight' % 'straight', 'curved', 'uphill', 'downhill', 'curved_grade', 'unstructured'
        road_length         double = 150.0   % Total length (m)
        road_width_nominal  double = 6.0     % Nominal width (m)
        y_center_base       double = 3.0     % Base lateral center (m)
        
        % Curve parameters: y_c(x) = y_center_base + A * sin(2*pi*(x - x_start)/lambda)
        curve_amp           double = 0.0     % Amplitude of curvature (m)
        curve_lambda        double = 80.0    % Wavelength of curve (m)
        curve_x_start       double = 20.0    % Longitudinal start of curve (m)
        
        % Longitudinal grade parameters (uphill / downhill)
        grade_slope         double = 0.0     % tan(theta_grade), e.g. +0.06 (6% uphill), -0.05 (5% downhill)
        grade_x_start       double = 15.0    % Start of grade incline/decline (m)
        
        % Irregular boundary variation (unstructured road edges)
        boundary_noise_amp  double = 0.0     % Lateral edge irregularity amplitude (m)
        
        % Physical potholes and road defects
        potholes            struct = struct('id', {}, 'x', {}, 'y', {}, 'length', {}, 'width', {}, 'depth', {}, 'severity', {})
    end
    
    methods
        function obj = RoadGeometry(road_type, varargin)
            if nargin >= 1 && ~isempty(road_type)
                obj.road_type = lower(road_type);
            end
            
            % 1. Set baseline defaults according to road_type
            switch obj.road_type
                case 'straight'
                    obj.curve_amp = 0.0;
                    obj.curve_lambda = 80.0;
                    obj.grade_slope = 0.0;
                    obj.boundary_noise_amp = 0.0;
                    
                case 'curved'
                    obj.curve_amp = 0.80; % Gentle, physically plausible curve
                    obj.curve_lambda = 80.0;
                    obj.grade_slope = 0.0;
                    obj.boundary_noise_amp = 0.08;
                    
                case 'uphill'
                    obj.curve_amp = 0.0;
                    obj.curve_lambda = 80.0;
                    obj.grade_slope = 0.05; % 5% uphill grade
                    obj.boundary_noise_amp = 0.05;
                    
                case 'downhill'
                    obj.curve_amp = 0.0;
                    obj.curve_lambda = 80.0;
                    obj.grade_slope = -0.05; % 5% downhill grade
                    obj.boundary_noise_amp = 0.05;
                    
                case 'curved_grade'
                    obj.curve_amp = 0.70;
                    obj.curve_lambda = 80.0;
                    obj.grade_slope = 0.04; % Curved + uphill
                    obj.boundary_noise_amp = 0.08;
                    
                case 'unstructured'
                    obj.curve_amp = 0.60;
                    obj.curve_lambda = 60.0;
                    obj.grade_slope = 0.02;
                    obj.boundary_noise_amp = 0.15;
            end
            
            % 2. Parse any explicit parameter overrides
            p = inputParser;
            addParameter(p, 'road_length', obj.road_length, @isnumeric);
            addParameter(p, 'road_width', obj.road_width_nominal, @isnumeric);
            addParameter(p, 'y_center', obj.y_center_base, @isnumeric);
            addParameter(p, 'curve_amp', obj.curve_amp, @isnumeric);
            addParameter(p, 'curve_lambda', obj.curve_lambda, @isnumeric);
            addParameter(p, 'grade_slope', obj.grade_slope, @isnumeric);
            if nargin > 1
                parse(p, varargin{:});
                obj.road_length = p.Results.road_length;
                obj.road_width_nominal = p.Results.road_width;
                obj.y_center_base = p.Results.y_center;
                obj.curve_amp = p.Results.curve_amp;
                obj.curve_lambda = p.Results.curve_lambda;
                obj.grade_slope = p.Results.grade_slope;
            end
        end
        
        function [y_c, theta_r, kappa] = getCenterline(obj, x)
            % GETCENTERLINE Returns centerline position y_c, tangential heading theta_r, and curvature kappa
            if obj.curve_amp == 0 || x < obj.curve_x_start
                y_c = obj.y_center_base;
                theta_r = 0.0;
                kappa = 0.0;
            else
                dx = x - obj.curve_x_start;
                k_w = 2 * pi / obj.curve_lambda;
                % Smooth ramp-in transition using sigmoid/cubic envelope
                s_ramp = min(1.0, dx / 15.0);
                envelope = 3.0 * s_ramp^2 - 2.0 * s_ramp^3;
                
                y_c = obj.y_center_base + envelope * obj.curve_amp * sin(k_w * dx);
                % Derivative dy/dx
                dy_dx = envelope * obj.curve_amp * k_w * cos(k_w * dx);
                theta_r = atan(dy_dx);
                % Second derivative d2y/dx2
                d2y_dx2 = -envelope * obj.curve_amp * (k_w^2) * sin(k_w * dx);
                kappa = d2y_dx2 / ((1 + dy_dx^2)^(1.5));
            end
        end
        
        function [y_min, y_max] = getBounds(obj, x)
            % GETBOUNDS Returns left and right physical drivable road bounds at longitudinal x
            [y_c, ~, ~] = obj.getCenterline(x);
            half_w = obj.road_width_nominal / 2.0;
            
            % Edge irregularity (realistic unstructured shoulder variations)
            if obj.boundary_noise_amp > 0
                delta_w_left = obj.boundary_noise_amp * sin(x / 7.0 + 0.4);
                delta_w_right = obj.boundary_noise_amp * cos(x / 9.0 + 1.2);
            else
                delta_w_left = 0.0;
                delta_w_right = 0.0;
            end
            
            y_min = y_c - half_w + delta_w_right;
            y_max = y_c + half_w + delta_w_left;
        end
        
        function [grade_angle, elevation] = getGrade(obj, x)
            % GETGRADE Returns longitudinal road incline angle theta_grade (rad) and cumulative elevation z (m)
            if obj.grade_slope == 0 || x < obj.grade_x_start
                grade_angle = 0.0;
                elevation = 0.0;
            else
                dx = x - obj.grade_x_start;
                elevation = obj.grade_slope * dx;
                grade_angle = atan(obj.grade_slope);
            end
        end
        
        function obj = addPothole(obj, id, x, y, length_p, width_p, depth_p, severity)
            % ADDPOTHOLE Adds a physical pothole hazard to the road surface
            if nargin < 5, length_p = 1.5; end
            if nargin < 6, width_p = 1.2; end
            if nargin < 7, depth_p = 0.08; end
            if nargin < 8, severity = 'moderate'; end
            
            p.id = id;
            p.x = x;
            p.y = y;
            p.length = length_p;
            p.width = width_p;
            p.depth = depth_p;
            p.severity = severity;
            
            obj.potholes(end+1) = p;
        end
        
        function [in_pothole, min_dist, closest_id] = checkPotholeProximity(obj, x, y, L, W)
            % CHECKPOTHOLEPROXIMITY Checks if a vehicle footprint overlaps or nears any pothole
            in_pothole = false;
            min_dist = inf;
            closest_id = -1;
            
            if isempty(obj.potholes)
                return;
            end
            
            for i = 1:length(obj.potholes)
                p = obj.potholes(i);
                dx = abs(x - p.x);
                dy = abs(y - p.y);
                
                % Radial/OBB gap
                gap_x = max(0.0, dx - (L + p.length)/2);
                gap_y = max(0.0, dy - (W + p.width)/2);
                dist = hypot(gap_x, gap_y);
                
                if dist < min_dist
                    min_dist = dist;
                    closest_id = p.id;
                end
                
                if gap_x == 0 && gap_y == 0
                    in_pothole = true;
                end
            end
        end
        
        function is_in_bounds = isFootprintInBounds(obj, x, y, theta, L, W, margin)
            % ISFOOTPRINTINBOUNDS Evaluates whether an oriented box stays strictly inside road boundaries
            if nargin < 7, margin = 0.0; end
            
            half_L = L / 2 + margin;
            half_W = W / 2 + margin;
            
            % Corner coordinates relative to center
            corners_local = [ half_L,  half_W;
                              half_L, -half_W;
                             -half_L, -half_W;
                             -half_L,  half_W ];
            
            rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            corners_world = (rot * corners_local')' + [x, y];
            
            is_in_bounds = true;
            for c = 1:4
                px = corners_world(c, 1);
                py = corners_world(c, 2);
                [y_min, y_max] = obj.getBounds(px);
                if py < y_min || py > y_max
                    is_in_bounds = false;
                    return;
                end
            end
        end
    end
end
