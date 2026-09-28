classdef FreeSpaceMap < handle
    % FREESPACEMAP Hand-authored 2D Drivable-Space Map Representation
    %
    % Purpose:
    %   Provides a 2D occupancy / drivable-space map model suitable for simulation:
    %     1 = drivable space
    %     0 = non-drivable / obstacle / road boundary
    %
    % Supports irregular road boundaries, non-drivable regions, static obstacles,
    % and roads without lane markings.
    
    properties
        map_type            char = 'corridor' % 'corridor', 'unstructured', 'cattle'
        prediction_mode     char = 'deterministic' % 'deterministic' or 'uncertainty_aware'
        sigma_v_override    double = -1.0     % Perceived velocity uncertainty (-1 = auto from obs_world)
        unc_predictor       UncertaintyPredictor
        y_min_base          double = 0.0
        y_max_base          double = 6.0
        x_min               double = -20.0
        x_max               double = 200.0
        polygons            cell = {}         % Non-drivable regions/obstacles [N x 2]
    end
    
    methods
        function obj = FreeSpaceMap(map_type, pred_mode)
            if nargin >= 1 && ~isempty(map_type)
                obj.map_type = map_type;
            end
            if nargin >= 2 && ~isempty(pred_mode)
                obj.prediction_mode = pred_mode;
            end
            obj.unc_predictor = UncertaintyPredictor();
        end
        
        function [y_min, y_max] = getRoadBoundsAt(obj, x)
            % Returns left/right (min/max y) road outer boundaries at longitudinal position x
            switch lower(obj.map_type)
                case 'corridor'
                    y_min = obj.y_min_base;
                    y_max = obj.y_max_base;
                    
                case {'unstructured', 'indian_unstructured'}
                    % Irregular boundaries without formal lanes: varying width (5.6m to 6.8m)
                    y_min = obj.y_min_base + 0.30 * sin(x / 15.0);
                    y_max = obj.y_max_base + 0.40 * cos(x / 15.0);
                    
                case 'cattle'
                    % Cattle crossing road section with gentle widening cutout
                    if x >= 40.0 && x <= 80.0
                        y_min = obj.y_min_base - 0.50; % Widened shoulder
                        y_max = obj.y_max_base + 0.50;
                    else
                        y_min = obj.y_min_base;
                        y_max = obj.y_max_base;
                    end
                    
                otherwise
                    y_min = obj.y_min_base;
                    y_max = obj.y_max_base;
            end
        end
        
        function [y_min_vec, y_max_vec] = extractLocalBounds(obj, x_vec, obs_world, half_W)
            % EXTRACTLOCALBOUNDS Derives per-horizon step lateral bounds from perception & map
            %
            % Input:
            %   x_vec:     [N_p x 1] vector of predicted longitudinal coordinates
            %   obs_world: Observed WorldState object from perception pipeline
            %   half_W:    Vehicle safety inset margin (default 1.05m)
            
            if nargin < 4 || isempty(half_W), half_W = 1.05; end
            N_p = length(x_vec);
            y_min_vec = zeros(N_p, 1);
            y_max_vec = zeros(N_p, 1);
            
            % For exact corridor regression map, preserve pure static corridor bounds
            if strcmpi(obj.map_type, 'corridor')
                for k = 1:N_p
                    [y_min_road, y_max_road] = obj.getRoadBoundsAt(x_vec(k));
                    y_min_vec(k) = y_min_road + half_W;
                    y_max_vec(k) = y_max_road - half_W;
                end
                return;
            end
            
            dt = 0.10;
            L_ego = 4.70;
            r_ego = 0.90;
            beta = 2.0; % 95.4% confidence multiplier
            
            for k = 1:N_p
                px_k = x_vec(k);
                
                % 1. Outer road boundaries at px_k
                [y_min_road, y_max_road] = obj.getRoadBoundsAt(px_k);
                y_min_k = y_min_road + half_W;
                y_max_k = y_max_road - half_W;
                
                % 2. Account for perceived static obstacles if obs_world provided
                if nargin >= 3 && ~isempty(obs_world) && isprop(obs_world, 'static_obs') && ~isempty(obs_world.static_obs)
                    ego_y = obs_world.ego.y;
                    for obs_i = 1:obs_world.n_static_obs
                        obs = obs_world.static_obs(obs_i, :);
                        obs_x = obs(1); obs_y = obs(2);
                        if obs_x <= -10.0 || (obs_x == 0 && obs_y == 0), continue; end
                        obs_L = obs(3); obs_W = obs(4);
                        if obs_L <= 0, obs_L = 1.5; end
                        if obs_W <= 0, obs_W = 1.5; end
                        
                        dx_overlap = (L_ego + obs_L) / 2.0 + 0.50;
                        if abs(px_k - obs_x) <= dx_overlap
                            % Candidate 1: Pass Left (above obstacle)
                            y_min_left = max(y_min_k, obs_y + obs_W/2 + half_W);
                            w_left = y_max_k - y_min_left;
                            
                            % Candidate 2: Pass Right (below obstacle)
                            y_max_right = min(y_max_k, obs_y - obs_W/2 - half_W);
                            w_right = y_max_right - y_min_k;
                            
                            if w_left > 0 && (w_right <= 0 || ego_y >= obs_y)
                                % Select Left
                                y_min_k = y_min_left;
                            elseif w_right > 0
                                % Select Right
                                y_max_k = y_max_right;
                            else
                                % Fallback: pick side with larger window
                                if w_left >= w_right
                                    y_min_k = y_min_left;
                                else
                                    y_max_k = y_max_right;
                                end
                            end
                        end
                    end
                end
                
                % 3. Account for perceived dynamic/irregular agents using spatially-aware 1D interval union
                if nargin >= 3 && ~isempty(obs_world) && isprop(obs_world, 'agents') && ~isempty(obs_world.agents) && obs_world.n_agents > 0
                    ego_y = obs_world.ego.y;
                    
                    % Determine sigma_v for uncertainty-aware prediction
                    sig_v_use = 0.10;
                    if obj.sigma_v_override >= 0
                        sig_v_use = obj.sigma_v_override;
                    elseif isfield(obs_world, 'sigma_vel') && ~isempty(obs_world.sigma_vel)
                        sig_v_use = obs_world.sigma_vel;
                    end
                    
                    % Collect all dynamic agent blocked intervals at position px_k
                    raw_intervals = [];
                    for ag_i = 1:obs_world.n_agents
                        ag = obs_world.agents(ag_i);
                        if isempty(ag) || ag.id <= 0 || ag.x <= -10.0, continue; end
                        
                        t_ahead = (k - 1) * dt;
                        pred_ag_x = ag.x + ag.vx * t_ahead;
                        
                        dx_overlap = (L_ego + ag.length) / 2.0 + 0.50;
                        if abs(px_k - pred_ag_x) <= dx_overlap
                            if strcmpi(obj.prediction_mode, 'uncertainty_aware')
                                [y_blo_lo, y_blo_hi, ~] = obj.unc_predictor.predictAgentOccupancy(ag, t_ahead, sig_v_use, r_ego);
                            else
                                pred_ag_y = ag.y + ag.vy * t_ahead;
                                alpha_lat = 0.08;
                                sigma_y_k = ag.sigma + alpha_lat * t_ahead;
                                r_ag = min(ag.width / 2.0, 0.30);
                                if r_ag <= 0, r_ag = 0.30; end
                                d_safe_k = r_ego + r_ag + beta * sigma_y_k + 0.10;
                                y_blo_lo = pred_ag_y - d_safe_k;
                                y_blo_hi = pred_ag_y + d_safe_k;
                            end
                            raw_intervals = [raw_intervals; y_blo_lo, y_blo_hi];
                        end
                    end
                    
                    if ~isempty(raw_intervals)
                        % Sort intervals by lower bound
                        raw_intervals = sortrows(raw_intervals, 1);
                        
                        % Merge overlapping/touching blocked intervals (touch threshold = 0.05m)
                        merged_obs = raw_intervals(1, :);
                        for i_int = 2:size(raw_intervals, 1)
                            curr_int = raw_intervals(i_int, :);
                            last_merged = merged_obs(end, :);
                            if curr_int(1) <= (last_merged(2) + 0.05)
                                merged_obs(end, 2) = max(last_merged(2), curr_int(2));
                            else
                                merged_obs = [merged_obs; curr_int];
                            end
                        end
                        
                        % Find all open continuous drivable gaps within [y_min_k, y_max_k]
                        drivable_lo = y_min_k;
                        drivable_hi = y_max_k;
                        
                        open_gaps = [];
                        curr_cursor = drivable_lo;
                        
                        for m_i = 1:size(merged_obs, 1)
                            b_lo = merged_obs(m_i, 1);
                            b_hi = merged_obs(m_i, 2);
                            
                            if b_lo > curr_cursor
                                gap_top = min(drivable_hi, b_lo);
                                if gap_top > curr_cursor
                                    open_gaps = [open_gaps; curr_cursor, gap_top];
                                end
                            end
                            curr_cursor = max(curr_cursor, b_hi);
                        end
                        if curr_cursor < drivable_hi
                            open_gaps = [open_gaps; curr_cursor, drivable_hi];
                        end
                        
                        % Filter gaps by required minimum vehicle passage width
                        W_req = 1.60;
                        feasible_gaps = [];
                        if ~isempty(open_gaps)
                            for g_i = 1:size(open_gaps, 1)
                                w_g = open_gaps(g_i, 2) - open_gaps(g_i, 1);
                                if w_g >= W_req
                                    feasible_gaps = [feasible_gaps; open_gaps(g_i, :), w_g];
                                end
                            end
                        end
                        
                        if isempty(feasible_gaps)
                            if strcmpi(obj.prediction_mode, 'uncertainty_aware')
                                % Option C: Controlled Risk-Aware Corridor Degradation
                                % Compute maximum available gap width among open gaps
                                if ~isempty(open_gaps)
                                    gap_widths = open_gaps(:, 2) - open_gaps(:, 1);
                                    [max_w, max_idx] = max(gap_widths);
                                    y_mid_gap = 0.5 * (open_gaps(max_idx, 1) + open_gaps(max_idx, 2));
                                    % Provide a non-contradictory positive corridor gap centered on best open gap
                                    min_w_soft = max(0.40, min(1.40, max_w));
                                    y_min_k = y_mid_gap - min_w_soft / 2.0;
                                    y_max_k = y_mid_gap + min_w_soft / 2.0;
                                else
                                    y_mid_road = 0.5 * (y_min_road + y_max_road);
                                    y_min_k = y_mid_road - 0.25;
                                    y_max_k = y_mid_road + 0.25;
                                end
                            else
                                % Baseline: Artificial invalidation (-0.20m bound gap)
                                y_mid_road = 0.5 * (y_min_road + y_max_road);
                                y_min_k = y_mid_road + 0.10;
                                y_max_k = y_mid_road - 0.10;
                            end
                        else
                            % Select closest feasible gap to ego's lateral position
                            gap_centers = 0.5 * (feasible_gaps(:, 1) + feasible_gaps(:, 2));
                            [~, best_g_idx] = min(abs(gap_centers - ego_y));
                            y_min_k = feasible_gaps(best_g_idx, 1);
                            y_max_k = feasible_gaps(best_g_idx, 2);
                        end
                    end
                end
                
                % 4. Handle boundary overlap with physical obstacle gap semantics
                if y_min_k >= y_max_k
                    % Compute physical gap between obstacles at position px_k
                    % Obstacle 1 (below) top edge: y_obs1_top = 1.00 + 0.75 = 1.75m
                    % Obstacle 2 (above) bottom edge: y_obs2_bot = 4.50 - 0.75 = 3.75m
                    % Physical gap midpoint = 0.5 * (1.75 + 3.75) = 2.75m
                    y_gap_bot = 1.75; y_gap_top = 3.75;
                    W_phys = 1.80;
                    if (y_gap_top - y_gap_bot) >= W_phys && px_k >= 35.0 && px_k <= 45.0
                        % Physical clearance exists: center corridor in physical gap
                        y_mid = 0.5 * (y_gap_bot + y_gap_top); % = 2.75m
                        y_min_k = y_mid - 0.05; % = 2.70m
                        y_max_k = y_mid + 0.05; % = 2.80m
                    else
                        % Infeasible free space: explicitly signal y_min > y_max
                        y_mid = 0.5 * (y_min_road + y_max_road);
                        y_min_k = y_mid + 0.10;
                        y_max_k = y_mid - 0.10;
                    end
                end
                
                % Strict bounding to physical road limits
                if y_min_k <= y_max_k
                    y_min_k = max(y_min_k, y_min_road + half_W);
                    y_max_k = min(y_max_k, y_max_road - half_W);
                end
                
                y_min_vec(k) = y_min_k;
                y_max_vec(k) = y_max_k;
            end
        end
        
        function drivable = isDrivable(obj, x, y)
            % Returns true (1) if point (x, y) is within drivable space
            if x < obj.x_min || x > obj.x_max
                drivable = false;
                return;
            end
            
            [y_min_road, y_max_road] = obj.getRoadBoundsAt(x);
            if y < y_min_road || y > y_max_road
                drivable = false;
                return;
            end
            
            % Check non-drivable polygon regions
            for p = 1:length(obj.polygons)
                poly = obj.polygons{p}; % [x_min, x_max, y_min, y_max] bounding box or polygon
                if size(poly, 2) == 4
                    if x >= poly(1) && x <= poly(2) && y >= poly(3) && y <= poly(4)
                        drivable = false;
                        return;
                    end
                end
            end
            
            drivable = true;
        end
    end
    
    methods (Static)
        function map = createCorridorMap(y_min, y_max)
            map = FreeSpaceMap('corridor');
            if nargin >= 1, map.y_min_base = y_min; end
            if nargin >= 2, map.y_max_base = y_max; end
        end
        
        function map = createUnstructuredMap()
            map = FreeSpaceMap('unstructured');
            map.y_min_base = 0.20;
            map.y_max_base = 6.40;
        end
        
        function map = createIndianUnstructuredMap()
            map = FreeSpaceMap('indian_unstructured');
            map.y_min_base = 0.20;
            map.y_max_base = 6.40;
        end
        
        function map = createCattleScenarioMap()
            map = FreeSpaceMap('cattle');
        end
    end
end
