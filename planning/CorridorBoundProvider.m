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
            
            bounds = world.getRoadBounds();
            y_min_safe = bounds(3) + obj.half_W_bound;
            y_max_safe = bounds(4) - obj.half_W_bound;
            
            y_min_vec = repmat(y_min_safe, N_p, 1);
            y_max_vec = repmat(y_max_safe, N_p, 1);
        end
    end
end
