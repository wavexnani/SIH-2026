classdef UncertaintyPredictor < handle
    % UNCERTAINTYPREDICTOR Bounded Velocity-Uncertainty Dynamic Occupancy Predictor
    %
    % Purpose:
    %   Computes physically and statistically realistic dynamic agent lateral 
    %   occupancy envelopes over horizon tau under perception velocity noise sigma_v.
    
    properties
        k_sigma     double = 2.0        % Coverage factor (2.0 = ~95% confidence interval)
        alpha_unc   double = 1.0        % Uncertainty scaling multiplier
    end
    
    methods
        function obj = UncertaintyPredictor(varargin)
            if nargin > 0 && isstruct(varargin{1})
                if isfield(varargin{1}, 'k_sigma'), obj.k_sigma = varargin{1}.k_sigma; end
                if isfield(varargin{1}, 'alpha_unc'), obj.alpha_unc = varargin{1}.alpha_unc; end
            end
        end
        
        function [y_blo_lo, y_blo_hi, delta_y_unc] = predictAgentOccupancy(obj, ag, tau, sigma_v, r_ego)
            % PREDICTAGENTOCCUPANCY Computes uncertainty-expanded blocked interval.
            if nargin < 4 || isempty(sigma_v), sigma_v = 0.10; end
            if nargin < 5 || isempty(r_ego), r_ego = 0.90; end
            
            % 1. Physical agent footprint radius
            r_ag = min(ag.width / 2.0, 0.35);
            if r_ag <= 0, r_ag = 0.30; end
            
            % 2. Dynamic mean prediction
            pred_y = ag.y + ag.vy * tau;
            
            % 3. Bounded velocity uncertainty growth over horizon tau
            ag_sigma = 0.05;
            if isprop(ag, 'sigma') && ~isempty(ag.sigma), ag_sigma = ag.sigma; end
            
            delta_y_unc = obj.alpha_unc * (obj.k_sigma * (ag_sigma + sigma_v * tau));
            
            % 4. Total safety clearance (ego footprint + agent footprint + uncertainty margin)
            d_safe = r_ego + r_ag + delta_y_unc + 0.10;
            
            y_blo_lo = pred_y - d_safe;
            y_blo_hi = pred_y + d_safe;
        end
    end
end
