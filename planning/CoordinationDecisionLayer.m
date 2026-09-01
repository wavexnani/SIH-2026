classdef CoordinationDecisionLayer < handle
    % COORDINATIONDECISIONLAYER Multi-Vehicle Macro-Intent Decision Layer
    %
    % Evaluates high-level interaction intent (MAINTAIN, FOLLOW, YIELD, OVERTAKE)
    % and determines target speed and overtaking permissions for Stage 4 CA-CRC.
    
    properties
        v_des_nominal       % Nominal desired cruising speed (m/s) [Default 8.0 m/s]
        time_gap            % Target longitudinal time gap for car-following (s) [Default 1.5s]
        d_min_gap           % Minimum standstill gap distance (m) [Default 5.0m]
        active_overtake_id  % ID of agent currently being overtaken (-1 if none)
        overtake_enabled    % Scenario-level authority; false for FOLLOW-only tests
    end
    
    methods
        function obj = CoordinationDecisionLayer(v_des, time_gap, d_min_gap)
            if nargin < 1, v_des = 8.0; end
            if nargin < 2, time_gap = 1.5; end
            if nargin < 3, d_min_gap = 5.0; end
            
            obj.v_des_nominal = v_des;
            obj.time_gap = time_gap;
            obj.d_min_gap = d_min_gap;
            obj.active_overtake_id = -1;
            obj.overtake_enabled = true;
        end
        
        function reset(obj)
            obj.active_overtake_id = -1;
        end
        
        function decision = evaluate(obj, detections, interactions, ego)
            decision.macro_intent = 'MAINTAIN';
            decision.target_v = obj.v_des_nominal;
            decision.allow_overtake = true;
            decision.reason = 'Nominal cruise path clear';
            
            % Phase 15A Safety Gate Telemetry
            decision.overtake_candidate = false;
            decision.overtake_allowed = true;
            decision.available_passing_gap_m = 6.00;
            decision.required_passing_gap_m = 4.10; % W_ego(1.8) + W_other(1.8) + Total_Margin(0.50)
            decision.overtake_block_reason = 'NO_CANDIDATE';
            decision.oncoming_ttc_s = inf;
            
            if isempty(detections) || isempty(interactions)
                return;
            end
            
            % Check for oncoming vehicle conflicts (active until fully passed behind ego rear bumper dx < -7.5m)
            has_oncoming_threat = false;
            min_oncoming_ttc = inf;
            for i = 1:length(interactions)
                if strcmp(interactions(i).class_name, 'ONCOMING_VEHICLE')
                    det_dx = 100.0;
                    for d = 1:length(detections)
                        if detections(d).id == interactions(i).id
                            det_dx = detections(d).dx;
                            break;
                        end
                    end
                    if det_dx > -7.5
                        has_oncoming_threat = true;
                        if interactions(i).ttc < min_oncoming_ttc
                            min_oncoming_ttc = interactions(i).ttc;
                        end
                    end
                end
            end
            decision.oncoming_ttc_s = min_oncoming_ttc;
            
            % 1. Evaluate Latched Active Overtake State
            if obj.active_overtake_id > 0 && ~obj.overtake_enabled
                obj.active_overtake_id = -1;
            end
            if obj.active_overtake_id > 0
                found_target = false;
                for i = 1:length(detections)
                    if detections(i).id == obj.active_overtake_id
                        found_target = true;
                        if detections(i).dx < -7.5
                            % Overtake completed (ego is > 7.5m ahead of overtaken vehicle rear bumper)
                            obj.active_overtake_id = -1;
                        elseif has_oncoming_threat && detections(i).dx > 5.0
                            % Abort overtake if oncoming threat appears and ego hasn't passed target yet
                            obj.active_overtake_id = -1;
                        else
                            % Maintain active latched OVERTAKE intent
                            decision.macro_intent = 'OVERTAKE';
                            decision.allow_overtake = true;
                            decision.target_v = min(obj.v_des_nominal, max(4.0, ego.v + 2.0));
                            decision.reason = sprintf('Executing latched overtake on Agent %d (dx=%.1fm)', obj.active_overtake_id, detections(i).dx);
                            decision.overtake_candidate = true;
                            decision.overtake_allowed = true;
                            decision.overtake_block_reason = 'SPATIAL_PASS_FEASIBLE';
                            return;
                        end
                    end
                end
                if ~found_target
                    obj.active_overtake_id = -1;
                end
            end
            
            % Check for lead vehicle in same lane
            lead_idx = -1;
            min_lead_dx = inf;
            for i = 1:length(detections)
                det = detections(i);
                if det.is_ahead && det.is_same_lane && det.dx < min_lead_dx
                    min_lead_dx = det.dx;
                    lead_idx = i;
                end
            end
            
            % Priority 1: YIELD/FOLLOW behind lead vehicle when oncoming threat is active or gap is tight
            if lead_idx > 0
                lead_det = detections(lead_idx);
                d_safe_headway = 15.0;
                headway_err = lead_det.dx - d_safe_headway;
                relative_v = lead_det.dvx;
                v_pd = lead_det.v + 0.40 * headway_err + 0.60 * relative_v;
                v_pd_clamped = max(0.0, min(obj.v_des_nominal, v_pd));
                
                % Evaluate Spatial/TTC Feasibility Gate before OVERTAKE initiation
                if lead_det.dx <= 28.0 && lead_det.dx >= 6.0
                    decision.overtake_candidate = true;
                end
                
                [is_feasible, avail_gap, req_gap, block_reason, gate_ttc] = obj.is_spatial_overtake_feasible(lead_det, detections, interactions, ego);
                decision.available_passing_gap_m = avail_gap;
                decision.required_passing_gap_m = req_gap;
                decision.overtake_block_reason = block_reason;
                decision.oncoming_ttc_s = gate_ttc;
                decision.overtake_allowed = is_feasible;
                
                if has_oncoming_threat || ~is_feasible
                    decision.macro_intent = 'YIELD';
                    decision.allow_overtake = false;
                    d_standstill = 15.0;
                    err_d = lead_det.dx - d_standstill;
                    v_yield = max(0.0, min(v_pd_clamped, 0.50 * err_d));
                    decision.target_v = v_yield;
                    decision.reason = sprintf('Yielding behind lead vehicle (Gate blocked: %s, Avail=%.2fm, Req=%.2fm, TTC=%.2fs)', ...
                        block_reason, avail_gap, req_gap, gate_ttc);
                    return;
                else
                    if obj.overtake_enabled && is_feasible && lead_det.dx <= 28.0 && lead_det.dx >= 6.0
                        % Corridor clear, speed sufficient, & gap reachable -> Initiate latched overtake
                        obj.active_overtake_id = lead_det.id;
                        decision.macro_intent = 'OVERTAKE';
                        decision.allow_overtake = obj.overtake_enabled;
                        decision.target_v = min(obj.v_des_nominal, max(4.0, ego.v + 2.0));
                        decision.reason = sprintf('Initiating latched overtake on Agent %d (Gate Passed)', lead_det.id);
                        return;
                    elseif lead_det.dx > 28.0
                        % Maintain car-following at safe headway until closing within overtake window
                        decision.macro_intent = 'FOLLOW';
                        decision.allow_overtake = true;
                        decision.target_v = v_pd_clamped;
                        decision.reason = sprintf('Following lead vehicle at safe headway (gap=%.1fm, v_tgt=%.1fm/s)', lead_det.dx, v_pd_clamped);
                        return;
                    else
                        % Maneuver not yet feasible (e.g. accelerating from standstill or gap too tight) -> FOLLOW / YIELD
                        decision.macro_intent = 'FOLLOW';
                        decision.allow_overtake = false;
                        if ego.v < 2.0
                            decision.target_v = max(v_pd_clamped, min(4.0, obj.v_des_nominal));
                        else
                            decision.target_v = v_pd_clamped;
                        end
                        decision.reason = sprintf('Following/Accelerating behind lead vehicle (gap=%.1fm, v_ego=%.1fm/s, v_tgt=%.1fm/s)', lead_det.dx, ego.v, decision.target_v);
                        return;
                    end
                end
            end

            % An oncoming conflict without a same-lane lead still prohibits
            % overtaking. The physical obstacle remains under Stage 4 control.
            if has_oncoming_threat
                decision.macro_intent = 'YIELD';
                decision.allow_overtake = false;
                decision.target_v = max(0.0, min(obj.v_des_nominal, ...
                    obj.v_des_nominal * min_oncoming_ttc / 6.0));
                decision.reason = sprintf('Yielding for oncoming conflict (TTC=%.2fs)', min_oncoming_ttc);
                decision.overtake_allowed = false;
                decision.overtake_block_reason = 'ONCOMING_CONFLICT_PRESENT';
            end
        end
        
        function [is_feasible, avail_gap, req_gap, block_reason, min_ttc] = is_spatial_overtake_feasible(obj, lead_det, detections, interactions, ego)
            % IS_SPATIAL_OVERTAKE_FEASIBLE Derives physical corridor width & TTC bounds prior to OVERTAKE initiation.
            %
            % Geometric Derivation:
            %   W_road = 6.00 m (standard 2-lane road)
            %   W_ego = 1.80 m (ego vehicle footprint width)
            %   W_other = 1.80 m (conflicting vehicle footprint width)
            %   total_margin = 0.50 m (0.25m lateral clearance per vehicle boundary)
            %   W_req = W_ego + W_other + total_margin = 4.10 m
            %
            % Feasibility checks:
            %   1. Oncoming TTC check: min_oncoming_ttc >= 6.0s (in conflict window dx > -5m)
            %   2. Available passing corridor width check: W_avail >= W_req (4.10m)
            %   3. Ego velocity gate: v_ego >= 1.5 m/s (kinematic steering authority)
            %   4. Longitudinal gap gate: lead_det.dx >= 7.0 m (lane change space)
            
            W_ego = 1.80;
            W_other = 1.80;
            total_margin = 0.50; % Explicit 0.25m per side safety clearance
            req_gap = W_ego + W_other + total_margin; % 4.10 m
            
            avail_gap = 6.00; % Default road width when passing corridor is completely clear
            min_ttc = inf;
            block_reason = 'SPATIAL_PASS_FEASIBLE';
            is_feasible = true;
            
            % 1. Oncoming Threat & TTC Evaluation
            for i = 1:length(interactions)
                if strcmp(interactions(i).class_name, 'ONCOMING_VEHICLE')
                    det_dx = 100.0;
                    for d = 1:length(detections)
                        if detections(d).id == interactions(i).id
                            det_dx = detections(d).dx;
                            break;
                        end
                    end
                    if det_dx > -7.5 && det_dx < 75.0
                        if interactions(i).ttc < min_ttc
                            min_ttc = interactions(i).ttc;
                        end
                    end
                end
            end
            
            % 2. Calculate Available Passing Corridor Width (W_avail)
            % Scan detections for obstacles or oncoming vehicles occupying the passing lane (y in [2.5, 6.0] m)
            left_lane_blockage = false;
            for d = 1:length(detections)
                det = detections(d);
                if det.dx > -7.5 && det.dx < 50.0
                    % Check if vehicle or static obstacle is in passing corridor
                    if det.y > 2.50
                        left_lane_blockage = true;
                        % Net available gap is squeezed by vehicle width in left lane
                        w_obs = 1.80;
                        if isfield(det, 'width') && ~isempty(det.width), w_obs = det.width; end
                        % Net clear gap between right-lane vehicle (y~2.7) and left-lane obstacle (y~4.2)
                        avail_gap = min(avail_gap, max(0.0, (det.y - w_obs/2) - (ego.y + W_ego/2)));
                    end
                end
            end
            
            % 3. Evaluate Feasibility Conditions
            if min_ttc <= 6.0
                is_feasible = false;
                block_reason = 'TTC_TOO_LOW';
                return;
            end
            
            if left_lane_blockage && avail_gap < req_gap
                is_feasible = false;
                block_reason = 'INSUFFICIENT_SPATIAL_GAP';
                return;
            end
            
            if ego.v < 1.5
                is_feasible = false;
                block_reason = 'SPEED_TOO_LOW';
                return;
            end
            
            if lead_det.dx < 7.0
                is_feasible = false;
                block_reason = 'DISTANCE_TOO_SHORT';
                return;
            end
        end
    end
end
