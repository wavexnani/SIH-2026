classdef FreeSpaceBoundProvider < AbstractBoundProvider
    % FREESPACEBOUNDPROVIDER 2D Free-Space Lateral Bound Provider
    %
    % Purpose:
    %   Derives per-horizon step left and right lateral bounds (y_min_vec, y_max_vec)
    %   from a hand-authored 2D drivable-space map (FreeSpaceMap).
    %
    % Preserves exact downstream representation expected by existing QP.
    
    properties
        map             FreeSpaceMap
        half_W_bound    double = 0.98; % Safety margin accounting for length-heading projection (0.90m + 0.05m heading + 0.03m QP margin)
    end
    
    methods
        function obj = FreeSpaceBoundProvider(map_obj, half_W)
            if nargin >= 1 && ~isempty(map_obj)
                obj.map = map_obj;
            else
                obj.map = FreeSpaceMap.createCorridorMap(0.0, 6.0);
            end
            if nargin >= 2 && ~isempty(half_W)
                obj.half_W_bound = half_W;
            else
                obj.half_W_bound = 0.98; % 0.90m vehicle half-width + 0.05m heading + 0.03m QP margin
            end
        end
        
        function [y_min_vec, y_max_vec] = getBounds(obj, world, X_ref, N_p, vehicle_width)
            nx = 4;
            if nargin < 4 || isempty(N_p), N_p = 20; end
            
            x_vec = zeros(N_p, 1);
            for k = 1:N_p
                idx_x = (k - 1) * nx + 1;
                x_vec(k) = X_ref(idx_x);
            end
            
            if ismethod(obj.map, 'extractLocalBounds')
                [y_min_vec, y_max_vec] = obj.map.extractLocalBounds(x_vec, world, obj.half_W_bound);
            else
                y_min_vec = zeros(N_p, 1);
                y_max_vec = zeros(N_p, 1);
                for k = 1:N_p
                    [y_min_road, y_max_road] = obj.map.getRoadBoundsAt(x_vec(k));
                    y_min_vec(k) = y_min_road + obj.half_W_bound;
                    y_max_vec(k) = y_max_road - obj.half_W_bound;
                end
            end
        end
    end
end
