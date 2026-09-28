classdef CorridorBoundProvider < AbstractBoundProvider
    % CORRIDORBOUNDPROVIDER Fixed Road Corridor Lateral Bound Provider
    %
    % Purpose:
    %   Extracts constant per-horizon lateral bounds from world.getRoadBounds().
    %   Matches exact baseline behavior of Stage 4 and Stage 5.1.
    
    properties
        half_W_bound double = 1.05; % Safety margin accounting for length-heading projection (0.90m + 0.15m)
    end
    
    methods
        function obj = CorridorBoundProvider(half_W)
            if nargin >= 1 && ~isempty(half_W)
                obj.half_W_bound = half_W;
            end
        end
        
        function [y_min_vec, y_max_vec] = getBounds(obj, world, X_ref, N_p, vehicle_width)
            nx = 4;
            if nargin < 4 || isempty(N_p), N_p = 20; end
            
            if isprop(world, 'road_geometry') && ~isempty(world.road_geometry) && (world.road_geometry.curve_amp > 0 || world.road_geometry.boundary_noise_amp > 0)
                y_min_vec = zeros(N_p, 1);
                y_max_vec = zeros(N_p, 1);
                for k = 1:N_p
                    idx_x = (k - 1) * nx + 1;
                    px_k = X_ref(idx_x);
                    [y_min_road, y_max_road] = world.road_geometry.getBounds(px_k);
                    y_min_vec(k) = y_min_road + obj.half_W_bound;
                    y_max_vec(k) = y_max_road - obj.half_W_bound;
                end
            else
                bounds = world.getRoadBounds();
                y_min_safe = bounds(3) + obj.half_W_bound;
                y_max_safe = bounds(4) - obj.half_W_bound;
                
                y_min_vec = repmat(y_min_safe, N_p, 1);
                y_max_vec = repmat(y_max_safe, N_p, 1);
            end
        end
    end
end
