classdef BaselinePlannerSimple < handle
    % BASELINEPLANNERSIMPLE Lateral-Offset Candidate Baseline Planner (Stage 2 - IMMUTABLE)
    %
    % Purpose:
    %   Generates 7 candidate trajectories offset laterally from a reference path,
    %   hard-rejects candidates that violate road boundaries or exceed static obstacle
    %   safety threshold, then scores surviving safe candidates using offset, switching,
    %   and soft proximity costs.
    %
    % Candidate Offset Revision Note:
    %   Revised from nominal +/-2.0m to +/-1.5m after geometric feasibility analysis:
    %   - Road width W_road = 6.0m, centerline y_ref = 3.0m (bounds y in [0, 6.0]m).
    %   - Vehicle width W_ego = 1.8m (half-width 0.9m) + safety margin 0.2m = 1.10m.
    %   - Safe vehicle center range: y_center in [1.10m, 4.90m].
    %   - Relative offset from centerline y=3.0m: Dy in [-1.90m, +1.90m].
    %   - Nominal +/-2.0m places center at 1.0m and 5.0m (violates bounds).
    %   - +/-1.5m derived set: [-1.5, -1.0, -0.5, 0.0, +0.5, +1.0, +1.5] meters.
    %
    % Collision vs. Rejection Threshold Distinction:
    %   - Ground Truth Physical Collision: d_safe <= 0.0m
    %   - Planner Trajectory Hard Rejection: d_safe <= d_threshold (d_threshold = 0.05m)
    %
    % Frozen Stage 2 Baseline Configuration:
    %   - num_candidates  = 7
    %   - offsets         = [-1.5, -1.0, -0.5, 0.0, +0.5, +1.0, +1.5] m
    %   - horizon_length  = 22 (11.0m lookahead)
    %   - w_offset        = 15.0
    %   - w_switch        = 1.0
    %   - w_prox          = 5.0
    %   - d_threshold     = 0.05 m
    %   - collision_model = circular (d_safe = d_center - r_ego - r_obs)
    %   - dynamic_agents  = ignored
    %   - uncertainty     = none
    %
    % Freeze Enforcement Rule:
    %   These parameters are frozen for all comparative experiments. Changes are permitted
    %   only if a previously undiscovered implementation defect is demonstrated, in which
    %   case the affected stage must be revalidated and the corresponding experimental
    %   results regenerated.
    
    properties
        num_candidates      int32 = 7
        offsets             double               % [-1.5, ..., 0.0, ..., +1.5] m
        horizon_length      int32 = 22           % Lookahead horizon (22 pts = 11.0m)
        
        % Safety threshold (planner rejection margin, separate from physical collision 0m)
        d_threshold         double = 0.05        % 0.05m safety threshold above physical footprint
        lambda_prox         double = 1.0         % Proximity cost decay parameter (m)
        
        % Cost Weights (soft, for scoring safe candidates only)
        w_offset            double = 15.0        % Strong preference for road centerline (offset 0)
        w_switch            double = 1.0         % Low hysteresis: allow prompt return to centerline
        w_prox              double = 5.0         % Soft proximity penalty for near-clearance candidates
        
        % State memory for hysteresis
        prev_selected_idx   double = 4           % Default to center path (index 4 of 7)
    end
    
    methods
        function obj = BaselinePlannerSimple(varargin)
            % Constructor - Frozen Stage 2 Candidate Offsets [-1.5m, +1.5m]
            obj.offsets = linspace(-1.5, 1.5, obj.num_candidates);
        end
        
        function [best_path, best_idx, candidates, costs] = plan(obj, world, reference_path, cfg)
            ego = world.ego;
            
            % 1. Find nearest point on reference path
            ref_xy = reference_path(:, 1:2);
            dists = (ref_xy(:, 1) - ego.x).^2 + (ref_xy(:, 2) - ego.y).^2;
            [~, nearest_idx] = min(dists);
            
            % Extract forward horizon segment
            end_idx = min(nearest_idx + obj.horizon_length, size(reference_path, 1));
            ref_segment = reference_path(nearest_idx:end_idx, :);
            num_pts = size(ref_segment, 1);
            
            candidates = cell(obj.num_candidates, 1);
            costs = inf(obj.num_candidates, 1);   % Default Inf = rejected
            
            % Vehicle footprint radius for Stage 2 lateral clearance (half-width)
            r_ego = cfg.vehicle_width / 2;  % 0.9m lateral half-width
            
            % Safety-margined footprint for road boundary check
            half_L_bound = cfg.vehicle_length / 2 + cfg.safety_margin;
            half_W_bound = cfg.vehicle_width / 2 + cfg.safety_margin;
            
            % Road bounds
            bounds = world.getRoadBounds();
            x_min = bounds(1); x_max = bounds(2);
            y_min = bounds(3); y_max = bounds(4);
            
            % 2. Generate and evaluate candidate paths
            for i = 1:obj.num_candidates
                target_offset = obj.offsets(i);
                cand_path = zeros(num_pts, 3);
                
                for k = 1:num_pts
                    x_ref = ref_segment(k, 1);
                    y_ref = ref_segment(k, 2);
                    theta_ref = ref_segment(k, 3);
                    
                    nx = -sin(theta_ref);
                    ny = cos(theta_ref);
                    
                    cand_path(k, 1) = x_ref + target_offset * nx;
                    cand_path(k, 2) = y_ref + target_offset * ny;
                    cand_path(k, 3) = theta_ref;
                end
                
                candidates{i} = cand_path;
                
                % --- HARD REJECTION PHASE ---
                is_rejected = false;
                prox_cost_sum = 0;
                
                for k = 1:num_pts
                    px = cand_path(k, 1);
                    py = cand_path(k, 2);
                    ptheta = cand_path(k, 3);
                    
                    % (a) Road boundary rejection (oriented footprint + safety margin)
                    r_xb = abs(half_L_bound * cos(ptheta)) + abs(half_W_bound * sin(ptheta));
                    r_yb = abs(half_L_bound * sin(ptheta)) + abs(half_W_bound * cos(ptheta));
                    
                    if (px - r_xb < x_min) || (px + r_xb > x_max) || ...
                       (py - r_yb < y_min) || (py + r_yb > y_max)
                        is_rejected = true;
                        break;
                    end
                    
                    % (b) Static obstacle collision rejection (Stage 2: no uncertainty term)
                    %     Formula: d_safe = d_center - r_ego - r_obstacle
                    for obs_i = 1:world.n_static_obs
                        obs = world.static_obs(obs_i, :);
                        if obs(1) > -50  % Active static obstacle
                            obs_x = obs(1); obs_y = obs(2);
                            obs_W = obs(4);
                            r_obstacle = obs_W / 2;  % 1.0m lateral half-width
                            
                            d_center = norm([px - obs_x, py - obs_y]);
                            d_safe = d_center - r_ego - r_obstacle;
                            
                            % Hard safety rejection threshold (0.3m clearance)
                            if d_safe < obj.d_threshold
                                is_rejected = true;
                                break;
                            end
                            
                            % Soft proximity cost accumulator for surviving candidates
                            prox_cost_sum = prox_cost_sum + exp(-d_safe / obj.lambda_prox);
                        end
                    end
                    
                    if is_rejected
                        break;
                    end
                end
                
                % If rejected, cost stays Inf — skip scoring
                if is_rejected
                    continue;
                end
                
                % --- SCORING PHASE (safe candidates only) ---
                
                % Lateral offset penalty (prefer centerline when clear)
                offset_cost = abs(target_offset) * obj.w_offset;
                
                % Hysteresis switching cost (smoothness)
                switch_cost = abs(i - obj.prev_selected_idx) * obj.w_switch;
                
                % Soft proximity cost
                prox_cost = obj.w_prox * prox_cost_sum;
                
                costs(i) = offset_cost + switch_cost + prox_cost;
            end
            
            % 3. Select candidate with minimum cost
            [min_cost, best_idx] = min(costs);
            
            if isinf(min_cost)
                % Fallback if all candidates are rejected
                best_idx = obj.prev_selected_idx;
            end
            
            best_path = candidates{best_idx};
            obj.prev_selected_idx = best_idx;
        end
        
        function reset(obj)
            obj.prev_selected_idx = 4;  % Center of 7 candidates
        end
    end
end
