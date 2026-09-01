classdef AbstractBoundProvider < handle
    % ABSTRACTBOUNDPROVIDER Abstract interface for lateral bound providers.
    %
    % Purpose:
    %   Provides a unified interface for extracting horizon lateral bounds
    %   y_min_vec and y_max_vec for the CA-CRC QP planner.
    
    methods (Abstract)
        [y_min_vec, y_max_vec] = getBounds(obj, world, X_ref, N_p, vehicle_width)
    end
end
