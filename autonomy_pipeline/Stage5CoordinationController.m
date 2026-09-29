classdef Stage5CoordinationController < handle
    % STAGE5COORDINATIONCONTROLLER Top-Level Stage 5 Multi-Vehicle Controller
    %
    % Integrates Multi-Vehicle Detection, Interaction Classification, Risk Prediction,
    % and Coordination Decision Layer with the FROZEN Stage 4 CA-CRC Planner and Safety Filter.
    
    properties
        config              % SimulationConfig object
        cacrc_planner       % Frozen Stage 4 CACRCPlanner object
        safety_filter       % Frozen Stage 4 SafetyFilter object
        decision_layer      % Stage 5 CoordinationDecisionLayer object
        x_overtake_start    % Longitudinal position where overtake transition began (-1 if inactive)
        y_overtake_start    % Lateral position where overtake transition began (-1 if inactive)
        x_recenter_start    % Longitudinal position where re-centering transition began (-1 if inactive)
        y_recenter_start    % Lateral position where re-centering transition began (-1 if inactive)
        prev_intent         % Previous step macro intent
        overtake_clear_count% Step counter for persistent overtake completion confirmation
        in_overtake_maneuver% Boolean latch for active overtake maneuver
        target_overtake_id  % ID of target agent currently being overtaken (-1 if none)
        use_curvature_velocity logical = false % Proposal upgrade: curvature-aware speed profiling
        use_steering_filter    logical = false % Proposal upgrade: actuator-aware steering stabilization
        curvature_planner                      % CurvatureVelocityPlanner instance
        steering_filter                        % DelayAwareSteeringFilter instance
    end
    
    methods
        function obj = Stage5CoordinationController(cfg)
            % Constructor: Initialize Stage 5 Controller wrapper
            obj.config = cfg;
            obj.cacrc_planner = CACRCPlanner(cfg);
            obj.safety_filter = SafetyFilter(cfg);
            obj.decision_layer = CoordinationDecisionLayer(8.0, 1.5, 5.0);
            obj.x_overtake_start = -1;
            obj.y_overtake_start = -1;
            obj.x_recenter_start = -1;
            obj.y_recenter_start = -1;
            obj.prev_intent = 'MAINTAIN';
            obj.overtake_clear_count = 0;
            obj.in_overtake_maneuver = false;
            obj.target_overtake_id = -1;
            obj.use_curvature_velocity = false;
            obj.use_steering_filter = false;
            obj.curvature_planner = CurvatureVelocityPlanner(cfg);
            obj.steering_filter = DelayAwareSteeringFilter(cfg);
        end
        
        function reset(obj)
            % Reset controller internal state
            obj.cacrc_planner.reset();
            obj.decision_layer.reset();
            obj.x_overtake_start = -1;
            obj.y_overtake_start = -1;
            obj.x_recenter_start = -1;
            obj.y_recenter_start = -1;
            obj.prev_intent = 'MAINTAIN';
            obj.overtake_clear_count = 0;
            obj.in_overtake_maneuver = false;
            obj.target_overtake_id = -1;
            if ~isempty(obj.steering_filter)
                obj.steering_filter.reset();
            end
        end

        function setOvertakeEnabled(obj, enabled)
            % Scenario policy only: this never bypasses Stage 4 authority.
            obj.decision_layer.overtake_enabled = logical(enabled);
            if ~enabled
                obj.decision_layer.active_overtake_id = -1;
                obj.in_overtake_maneuver = false;
                obj.target_overtake_id = -1;
            end
        end
        
        function [u_cmd, pred_states, status, info] = step(obj, world, ref_path, v_des_nominal)
            if nargin < 4, v_des_nominal = obj.config.ego_v_init; end
            if ~isempty(v_des_nominal) && isprop(obj.decision_layer, 'v_des_nominal')
                obj.decision_layer.v_des_nominal = v_des_nominal;
            end
            t_step_start = tic;
            
            % 1. Perception & Relative State Detection
            detections = MultiVehicleDetector.detect(world, world.ego);
            
            % 2. Dynamic Trajectory Prediction & Risk Estimation (Time-To-Conflict)
            preds = RiskPredictor.predictTrajectories(detections, 10, 0.10);
            [ttc_vector, min_ttc] = RiskPredictor.predictTTC(detections, obj.config.vehicle_length);
            
            % 3. Interaction Classification
            interactions = InteractionClassifier.classify(detections, world.ego, ttc_vector);
            
            % 4. Coordination Decision Layer (Macro-Intent Selection)
            decision = obj.decision_layer.evaluate(detections, interactions, world.ego);
            
            % 5. Build Intent-Conditioned Reference Trajectory
            if strcmp(decision.macro_intent, 'OVERTAKE')
                obj.in_overtake_maneuver = true;
                if obj.decision_layer.active_overtake_id > 0
                    obj.target_overtake_id = obj.decision_layer.active_overtake_id;
                end
            end

            % Track specific active overtaken vehicle target_id to eliminate premature de-latching when lead is >15m ahead
            % 7.50 m = existing engineering completion threshold (vehicle length 4.70m + safety buffer 2.50m + margin 0.30m)
            % 5 steps x 0.10 s = 0.50 s (temporal debounce/hysteresis choice)
            lead_cleared = false;
            target_id = obj.target_overtake_id;
            if target_id <= 0 && obj.decision_layer.active_overtake_id > 0
                target_id = obj.decision_layer.active_overtake_id;
                obj.target_overtake_id = target_id;
            end

            target_found = false;
            if obj.in_overtake_maneuver && target_id > 0
                if ~isempty(detections)
                    for i = 1:length(detections)
                        if detections(i).id == target_id
                            target_found = true;
                            ego_front_minus_lead_front = -detections(i).dx;
                            if ego_front_minus_lead_front >= 5.00
                                lead_cleared = true;
                            end
                            break;
                        end
                    end
                end
                % If active target vehicle was passed and is now behind ego / absent from forward detections:
                if ~target_found && ~strcmp(decision.macro_intent, 'OVERTAKE')
                    lead_cleared = true;
                end
            end

            if lead_cleared && obj.in_overtake_maneuver
                obj.overtake_clear_count = obj.overtake_clear_count + 1;
            else
                obj.overtake_clear_count = 0;
            end
            is_physically_complete = (obj.overtake_clear_count >= 3) || ...
                                     (~target_found && ~strcmp(decision.macro_intent, 'OVERTAKE')) || ...
                                     (strcmp(decision.macro_intent, 'YIELD') || strcmp(decision.macro_intent, 'FOLLOW'));

            % Transition state logic: remain in overtake maneuver until physically complete
            was_in_maneuver = obj.in_overtake_maneuver;
            if obj.in_overtake_maneuver && is_physically_complete
                obj.in_overtake_maneuver = false;
                obj.target_overtake_id = -1;
                obj.overtake_clear_count = 0;
            end

            is_overtake_active = obj.in_overtake_maneuver;
            if is_overtake_active
                obj.cacrc_planner.locked_side = 'left';
                if obj.x_overtake_start < 0
                    obj.x_overtake_start = world.ego.x;
                    obj.y_overtake_start = max(ref_path(1, 2), world.ego.y);
                    obj.x_recenter_start = -1;
                    obj.y_recenter_start = -1;
                end
            else
                obj.cacrc_planner.locked_side = 'none';
                
                [~, nearest_r0] = min(abs(ref_path(:, 1) - world.ego.x));
                y_nom_curr = ref_path(nearest_r0, 2);
                has_obs_ahead = false;
                if isprop(world, 'static_obs') && ~isempty(world.static_obs)
                    for obs_i = 1:size(world.static_obs, 1)
                        ox = world.static_obs(obs_i, 1);
                        if ox > world.ego.x && (ox - world.ego.x) < 18.0
                            has_obs_ahead = true;
                            break;
                        end
                    end
                end
                
                if was_in_maneuver && obj.x_recenter_start < 0
                    obj.x_recenter_start = world.ego.x;
                    obj.y_recenter_start = world.ego.y;
                    obj.x_overtake_start = -1;
                    obj.y_overtake_start = -1;
                end
                
                % Reset recenter state once ego has returned to right lane centerline (|y - y_ref| <= 0.15m and aligned)
                if obj.x_recenter_start > 0
                    [~, nearest_r] = min(abs(ref_path(:, 1) - world.ego.x));
                    y_centerline = ref_path(nearest_r, 2);
                    if abs(world.ego.y - y_centerline) <= 0.15 && abs(world.ego.theta) <= 0.05
                        obj.x_recenter_start = -1;
                        obj.y_recenter_start = -1;
                    end
                end
            end
            obj.prev_intent = decision.macro_intent;
            
            % Construct augmented world model for CA-CRC planning using state predictions:
            world_coord = world;
            if isprop(world_coord, 'agents') && ~isempty(world_coord.agents)
                valid_agents = world_coord.agents;
                keep_mask = true(1, length(valid_agents));
                for i = 1:length(valid_agents)
                    ag = valid_agents(i);
                    if isempty(ag) || ag.id <= 0 || (isprop(ag, 'is_active') && ~ag.is_active)
                        keep_mask(i) = false;
                    end
                end
                world_coord.agents = valid_agents(keep_mask);
                world_coord.n_agents = length(world_coord.agents);
            end
            
            % Filter cleared static obstacles (footprint-aware clearance: rear bumper clears obstacle front edge)
            if isprop(world_coord, 'static_obs') && ~isempty(world_coord.static_obs)
                obs_active = world_coord.static_obs;
                for obs_i = 1:size(obs_active, 1)
                    obs_x = obs_active(obs_i, 1);
                    obs_L = 2.0;
                    if size(obs_active, 2) >= 3 && obs_active(obs_i, 3) > 0
                        obs_L = obs_active(obs_i, 3);
                    end
                    d_clear_thresh = obs_L / 2.0 + 4.50 / 2.0 + 0.60;
                    if obs_x > 0 && world.ego.x > (obs_x + d_clear_thresh)
                        obs_active(obs_i, 1) = -100.0; % Mark obstacle as cleared
                    end
                end
                world_coord.static_obs = obs_active;
            end
            
            % Dynamic agents are handled by RiskPredictor and CACRCPlanner via velocity predictions.
            % No static obstacle augmentation required.
            
            % 5. Bounded Lateral Reference Path Construction
            bp = [];
            if ~isempty(obj.cacrc_planner.bound_provider) && isprop(obj.cacrc_planner.bound_provider, 'map') && ~isempty(obj.cacrc_planner.bound_provider.map)
                bp = obj.cacrc_planner.bound_provider;
            end
            
            is_unstructured = (~isempty(bp) && isprop(bp, 'map') && ~isempty(bp.map) && ismethod(bp.map, 'extractLocalBounds') && ...
               (strcmpi(bp.map.map_type, 'unstructured') || strcmpi(bp.map.map_type, 'indian_unstructured'))) || ...
               (isprop(world, 'road_geometry') && ~isempty(world.road_geometry) && (strcmpi(world.road_geometry.road_type, 'unstructured') || world.road_geometry.curve_amp > 0));

            ref_path_is_default_flat = all(abs(ref_path(:, 2) - ref_path(1, 2)) < 1e-4) && abs(ref_path(1, 2) - 3.0) < 0.05;
            if is_unstructured && ref_path_is_default_flat
                % Unstructured Road: derive reference trajectory directly from FreeSpaceMap
                ref_path_coord = ref_path;
                x_pts = ref_path_coord(:, 1);
                [y_lo_vec, y_hi_vec] = bp.map.extractLocalBounds(x_pts, world_coord, bp.half_W_bound);
                y_center_vec = 0.5 * (y_lo_vec + y_hi_vec);
                
                % 31-point (~10m) moving average smoothing over reference corridor center
                N_pts = length(y_center_vec);
                w_size = 15; % Window half-size
                y_center_smooth = y_center_vec;
                for r = (w_size + 1):(N_pts - w_size)
                    y_center_smooth(r) = mean(y_center_vec((r - w_size):(r + w_size)));
                end
                
                ref_path_coord(:, 2) = y_center_smooth;
                
                % Compute matching C1 reference heading theta_ref
                for r = 1:N_pts
                    r1 = max(1, r - 1); r2 = min(N_pts, r + 1);
                    dx_p = ref_path_coord(r2, 1) - ref_path_coord(r1, 1);
                    dy_p = ref_path_coord(r2, 2) - ref_path_coord(r1, 2);
                    ref_path_coord(r, 3) = atan2(dy_p, dx_p);
                end
            elseif is_overtake_active && obj.x_overtake_start > 0
                ref_path_coord = ref_path;
                d_obs_ahead = 20.0;
                if isprop(world, 'static_obs') && ~isempty(world.static_obs)
                    for obs_i = 1:world.n_static_obs
                        obs_x = world.static_obs(obs_i, 1);
                        if obs_x > obj.x_overtake_start
                            d_obs_ahead = min(d_obs_ahead, obs_x - obj.x_overtake_start);
                        end
                    end
                end
                y_centerline = ref_path(1, 2);
                if isprop(world_coord, 'road_geometry') && ~isempty(world_coord.road_geometry)
                    [y_lo_ego, y_hi_ego] = world_coord.road_geometry.getBounds(world.ego.x);
                else
                    y_lo_ego = 0.0; y_hi_ego = 6.0;
                end
                y_pass_center = y_centerline + 0.25 * (y_hi_ego - y_lo_ego);
                L_lc = max(8.0, min(14.0, d_obs_ahead - 2.5));
                target_y_ov = min(y_hi_ego - 1.10, max(y_centerline + 1.20, y_pass_center));
                y_start_ov = obj.y_overtake_start;
                if y_start_ov < 0, y_start_ov = y_centerline; end
                dy_ov = target_y_ov - y_start_ov;
                
                for r = 1:size(ref_path_coord, 1)
                    px = ref_path_coord(r, 1);
                    s_norm = min(1.0, max(0.0, (px - obj.x_overtake_start) / L_lc));
                    smooth_s = 3.0 * s_norm^2 - 2.0 * s_norm^3;
                    ds_dsnorm = 6.0 * s_norm - 6.0 * s_norm^2;
                    dy_dx = (dy_ov * ds_dsnorm) / L_lc;
                    ref_path_coord(r, 2) = y_start_ov + dy_ov * smooth_s;
                    ref_path_coord(r, 3) = ref_path(r, 3) + atan(dy_dx);
                end
            elseif obj.x_recenter_start > 0
                L_lc_recenter = max(24.0, 4.0 * max(world.ego.v, 2.0));
                [~, nearest_r] = min(abs(ref_path(:, 1) - world.ego.x));
                y_centerline = ref_path(nearest_r, 2);
                
                if world.ego.x > (obj.x_recenter_start + L_lc_recenter)
                    if abs(world.ego.y - y_centerline) > 0.20
                        obj.x_recenter_start = world.ego.x;
                        obj.y_recenter_start = world.ego.y;
                    else
                        obj.x_recenter_start = 0;
                        ref_path_coord = ref_path;
                        return;
                    end
                end
                
                ref_path_coord = ref_path;
                y_start = obj.y_recenter_start;
                if y_start < 0, y_start = world.ego.y; end
                for r = 1:size(ref_path_coord, 1)
                    px = ref_path_coord(r, 1);
                    s_norm = min(1.0, max(0.0, (px - obj.x_recenter_start) / L_lc_recenter));
                    smooth_s = 3.0 * s_norm^2 - 2.0 * s_norm^3;
                    ds_dsnorm = 6.0 * s_norm - 6.0 * s_norm^2;
                    y_target_lane = ref_path(r, 2);
                    dy_total = y_target_lane - y_start;
                    dy_dx = (dy_total * ds_dsnorm) / L_lc_recenter;
                    ref_path_coord(r, 2) = y_start + dy_total * smooth_s;
                    ref_path_coord(r, 3) = ref_path(r, 3) + atan(dy_dx);
                end
            else
                ref_path_coord = ref_path;
            end
            
            % Perception-Driven Roadside Livestock Flock Defense:
            % Scans perceived detections dynamically for active roadside grazing herds (sheep or cattle)
            has_roadside_herd = false;
            herd_x_list = [];
            herd_y_list = [];
            for d_i = 1:length(detections)
                det_vru = detections(d_i);
                if ismember(det_vru.type, {'sheep', 'cattle'})
                    if det_vru.dx > -5.0 && det_vru.dx < 35.0
                        has_roadside_herd = true;
                        herd_x_list(end+1) = det_vru.x;
                        herd_y_list(end+1) = det_vru.y;
                    end
                end
            end
            
            if has_roadside_herd && length(herd_x_list) >= 2
                x_herd_min = min(herd_x_list) - 5.0;
                x_herd_max = max(herd_x_list) + 8.0;
                span_x = max(18.0, x_herd_max - x_herd_min);
                y_herd_mean = mean(herd_y_list);
                
                % Query road centerline at flock location to determine occupied verge side
                y_c_herd = 3.0;
                if isprop(world_coord, 'road_geometry') && ~isempty(world_coord.road_geometry)
                    y_c_herd = world_coord.road_geometry.getCenterline(mean(herd_x_list));
                end
                
                % Defensive nudge: away from occupied verge towards open corridor
                nudge_dir = 1.0;
                if y_herd_mean > y_c_herd, nudge_dir = -1.0; end
                
                for r = 1:size(ref_path_coord, 1)
                    px_r = ref_path_coord(r, 1);
                    if px_r >= x_herd_min && px_r <= x_herd_max
                        s_norm = (px_r - x_herd_min) / span_x;
                        nudge_s = sin(pi * s_norm)^2;
                        ref_path_coord(r, 2) = ref_path_coord(r, 2) + nudge_dir * 0.60 * nudge_s;
                    end
                end
                
                % Recompute smooth reference heading
                for r = 1:size(ref_path_coord, 1)
                    r1 = max(1, r - 1); r2 = min(size(ref_path_coord, 1), r + 1);
                    dx_p = ref_path_coord(r2, 1) - ref_path_coord(r1, 1);
                    dy_p = ref_path_coord(r2, 2) - ref_path_coord(r1, 2);
                    ref_path_coord(r, 3) = atan2(dy_p, dx_p);
                end
            end
            
            if ~isempty(bp)
                for r = 1:size(ref_path_coord, 1)
                    px_r = ref_path_coord(r, 1);
                    if isprop(world_coord, 'road_geometry') && ~isempty(world_coord.road_geometry)
                        [y_lo, y_hi] = world_coord.road_geometry.getBounds(px_r);
                    else
                        [y_lo, y_hi] = bp.map.getRoadBoundsAt(px_r);
                    end
                    margin = bp.half_W_bound;
                    ref_path_coord(r, 2) = min(y_hi - margin, max(y_lo + margin, ref_path_coord(r, 2)));
                end
            end
            
            target_v_exec = decision.target_v;
            if is_unstructured
                target_v_exec = min(target_v_exec, 4.50);
            end
            
            % Speed Breaker Regulation: Slow down to 2.0 m/s (~7.2 km/h) for IRC bump
            if isprop(world, 'road_geometry') && ~isempty(world.road_geometry) && ...
               isprop(world.road_geometry, 'speed_breakers') && ~isempty(world.road_geometry.speed_breakers)
                for sb_idx = 1:length(world.road_geometry.speed_breakers)
                    sb = world.road_geometry.speed_breakers(sb_idx);
                    if world.ego.x >= (sb.x - 12.0) && world.ego.x <= (sb.x + 3.0)
                        target_v_exec = min(target_v_exec, 2.0);
                    end
                end
            end
            
            % Rural Livestock Speed Regulation: Safe passing crawl when alongside livestock
            if has_roadside_herd && ~isempty(herd_x_list) && ...
               world.ego.x >= (min(herd_x_list) - 3.0) && world.ego.x <= (max(herd_x_list) + 3.0)
                target_v_exec = min(target_v_exec, 3.0);
            end
            
            % Curvature-aware velocity profiling (automatically active for curved roads and blind bends)
            has_curve = isprop(world, 'road_geometry') && ~isempty(world.road_geometry) && ...
                        (world.road_geometry.curve_amp > 0 || ...
                        (isprop(world.road_geometry, 'blind_bend_active') && world.road_geometry.blind_bend_active));
            if (obj.use_curvature_velocity || has_curve) && ~isempty(obj.curvature_planner)
                obj.cacrc_planner.use_curvature_velocity = true;
                ref_path_coord = obj.curvature_planner.planVelocityProfile(ref_path_coord, target_v_exec);
            else
                obj.cacrc_planner.use_curvature_velocity = false;
            end
            
            t_plan_start = tic;
            [u_mpc, pred_states, status, cacrc_info] = obj.cacrc_planner.plan(world_coord, ref_path_coord, target_v_exec);
            t_plan_ms = toc(t_plan_start) * 1000;
            
            % 6. Delay-Aware Steering Filter (if enabled)
            % Stabilizes raw MPC command prior to Layer 2 safety verification
            t_steer_start = tic;
            steer_info = struct();
            if obj.use_steering_filter && ~isempty(obj.steering_filter)
                dt_step = obj.config.dt;
                corr_w = inf;
                if is_unstructured && ~isempty(bp) && ismethod(bp.map, 'getRoadBoundsAt')
                    [y_lo_ego, y_hi_ego] = bp.map.getRoadBoundsAt(world.ego.x);
                    corr_w = y_hi_ego - y_lo_ego;
                end
                [steer_filtered, s_info] = obj.steering_filter.step(u_mpc(1), world.ego.delta, dt_step, false, corr_w);
                u_mpc(1) = steer_filtered;
                steer_info = s_info;
            end
            t_steer_ms = toc(t_steer_start) * 1000;
            
            % 7. Apply Layer 2 Safety Filter Authority
            % Evaluates the filtered candidate command; provides final uncompromised safety authority
            t_safety_start = tic;
            [u_cmd, filter_active, filter_reason] = obj.safety_filter.filter(u_mpc, status, world_coord, pred_states, obj.cacrc_planner.bound_provider);
            t_safety_ms = toc(t_safety_start) * 1000;
            
            t_step_total_ms = toc(t_step_start) * 1000;
            t_ctrl_logic_ms = max(0, t_step_total_ms - t_plan_ms - t_safety_ms);
            
            % Assemble Stage 5 Diagnostics
            info = cacrc_info;
            info.macro_intent = decision.macro_intent;
            info.target_v_coord = decision.target_v;
            info.allow_overtake = decision.allow_overtake;
            info.coord_reason = decision.reason;
            info.min_ttc = min_ttc;
            info.n_detected_vehicles = length(detections);
            info.num_observed_agents = world.n_agents;
            info.num_detected_agents = length(detections);
            info.num_predicted_agents = length(preds);
            info.num_interacting_agents = length(interactions);
            info.num_planner_relevant = world_coord.n_agents;
            info.planner_relevant_ids = [];
            if world_coord.n_agents > 0 && isprop(world_coord.agents(1), 'id')
                info.planner_relevant_ids = [world_coord.agents.id];
            end
            info.filter_active = filter_active;
            info.filter_reason = filter_reason;
            info.steer_info = steer_info;
            
            % Sub-system Latency Breakdown (ms)
            info.timing = struct();
            info.timing.planner_ms = t_plan_ms;
            info.timing.controller_ms = t_ctrl_logic_ms;
            info.timing.steering_filter_ms = t_steer_ms;
            info.timing.safety_filter_ms = t_safety_ms;
            info.timing.total_ctrl_step_ms = t_step_total_ms;
            
            % --- Pipeline Instrumentation (logging-only, no algorithm change) ---
            info.detections = detections;
            info.predictions = preds;
            info.ttc_vector = ttc_vector;
            info.interactions = interactions;
            info.decision = decision;
            info.pred_ego_traj = pred_states;
            info.locked_side = obj.cacrc_planner.locked_side;
            info.in_overtake_maneuver = obj.in_overtake_maneuver;
            info.target_overtake_id = obj.target_overtake_id;
            info.u_mpc = u_mpc;
            info.u_safe = u_cmd;
        end
    end
end
