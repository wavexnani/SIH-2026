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
                    if det_dx > -7.5 && interactions(i).ttc <= 6.0
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
                w_target = 1.80;
                if isfield(det, 'width') && ~isempty(det.width) && det.width > 0
                    w_target = det.width;
                end
                % Lateral conflict limit: physical corridor overlap (half vehicle + half obstacle + buffer)
                lateral_conflict_limit = (1.80 + w_target) / 2.0 + 0.35;
                is_lane_target = det.is_same_lane || (det.dx <= 35.0 && abs(det.dy) <= lateral_conflict_limit);
                if ~ismember(det.type, {'pothole', 'static_obstacle', 'pedestrian', 'sheep', 'cattle'}) && ...
                   det.is_ahead && ~det.is_oncoming && is_lane_target && det.dx < min_lead_dx
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
                
                min_dx_gate = 3.5;
                if lead_det.v > 1.0, min_dx_gate = 6.0; end
                max_dx_gate = max(24.0, 4.0 * max(ego.v, 2.0));
                if lead_det.dx <= max_dx_gate && lead_det.dx >= min_dx_gate
                    decision.overtake_candidate = true;
                end
                
                [is_feasible, avail_gap, req_gap, block_reason, gate_ttc] = obj.is_spatial_overtake_feasible(lead_det, detections, interactions, ego);
                decision.available_passing_gap_m = avail_gap;
                decision.required_passing_gap_m = req_gap;
                decision.overtake_block_reason = block_reason;
                decision.oncoming_ttc_s = gate_ttc;
                decision.overtake_allowed = is_feasible;
                
                gate_blocked_by_threat_or_gap = has_oncoming_threat || ...
                    strcmp(block_reason, 'TTC_TOO_LOW') || ...
                    strcmp(block_reason, 'INSUFFICIENT_SPATIAL_GAP');
                
                if gate_blocked_by_threat_or_gap
                    decision.macro_intent = 'YIELD';
                    decision.allow_overtake = false;
                    
                    % Determine if the yield is caused by an oncoming conflict in the bottleneck
                    has_oncoming_conflict = has_oncoming_threat || ...
                        strcmp(block_reason, 'TTC_TOO_LOW');
                    if ~has_oncoming_conflict
                        for d = 1:length(detections)
                            if detections(d).is_oncoming && detections(d).dx > -7.5 && detections(d).dx < 45.0
                                has_oncoming_conflict = true;
                                break;
                            end
                        end
                    end
                    
                    if has_oncoming_conflict
                        % Standstill Hold at bottleneck entrance:
                        % When yielding the narrow corridor to opposing traffic, ego must hold position
                        % (or decelerate to stop before the bottleneck) until the opposing vehicle has cleared.
                        % If ego is already stopped/crawling (v <= 1.2 m/s) or within safe distance of the lead obstacle,
                        % command complete standstill (target_v = 0.0).
                        d_hold_lead = 8.0;
                        if ego.v <= 1.2 || lead_det.dx <= d_hold_lead
                            v_yield = 0.0;
                        else
                            err_hold = lead_det.dx - d_hold_lead;
                            v_yield = max(0.0, min(v_pd_clamped, 0.40 * err_hold));
                        end
                        decision.reason = sprintf('Yielding for oncoming bottleneck conflict (Standstill Hold: Avail=%.2fm, Req=%.2fm, TTC=%.2fs)', ...
                            avail_gap, req_gap, gate_ttc);
                    else
                        % Standard car-following queue creep when no oncoming vehicle is in conflict
                        d_standstill = 5.0;
                        if lead_det.v > 1.0, d_standstill = 10.0; end
                        err_d = lead_det.dx - d_standstill;
                        v_yield = max(0.0, min(v_pd_clamped, 0.50 * err_d));
                        decision.reason = sprintf('Yielding behind lead vehicle (Queue follow: Avail=%.2fm, Req=%.2fm)', ...
                            avail_gap, req_gap);
                    end
                    
                    decision.target_v = v_yield;
                    return;
                else
                    if obj.overtake_enabled && is_feasible && lead_det.dx <= max_dx_gate && lead_det.dx >= min_dx_gate
                        % Corridor clear, speed sufficient, & gap reachable -> Initiate latched overtake
                        obj.active_overtake_id = lead_det.id;
                        decision.macro_intent = 'OVERTAKE';
                        decision.allow_overtake = obj.overtake_enabled;
                        decision.target_v = min(obj.v_des_nominal, max(4.0, ego.v + 2.0));
                        decision.reason = sprintf('Initiating latched overtake on Agent %d (Gate Passed)', lead_det.id);
                        return;
                    elseif lead_det.dx > max_dx_gate
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

            % An oncoming conflict without a same-lane lead prohibits overtaking.
            % If the oncoming vehicle is cleanly in the opposing lane (|dy| >= 1.25m),
            % ego maintains its lane at nominal cruise speed rather than stopping.
            if has_oncoming_threat
                is_headon_encroachment = false;
                for i = 1:length(interactions)
                    if strcmp(interactions(i).class_name, 'ONCOMING_VEHICLE')
                        for d = 1:length(detections)
                            if detections(d).id == interactions(i).id
                                if abs(detections(d).dy) < 1.25
                                    is_headon_encroachment = true;
                                end
                                break;
                            end
                        end
                    end
                end
                
                decision.allow_overtake = false;
                if is_headon_encroachment
                    decision.macro_intent = 'YIELD';
                    if min_oncoming_ttc <= 3.0 || ego.v <= 1.2
                        decision.target_v = 0.0;
                    else
                        decision.target_v = max(0.0, min(obj.v_des_nominal, ...
                            obj.v_des_nominal * min_oncoming_ttc / 6.0));
                    end
                    decision.reason = sprintf('Yielding for head-on oncoming encroachment (TTC=%.2fs)', min_oncoming_ttc);
                else
                    decision.macro_intent = 'MAINTAIN';
                    decision.target_v = min(obj.v_des_nominal, 5.0);
                    decision.reason = sprintf('Maintaining lane while oncoming traffic passes in opposing lane (TTC=%.2fs)', min_oncoming_ttc);
                end
                decision.overtake_allowed = false;
                decision.overtake_block_reason = 'ONCOMING_CONFLICT_PRESENT';
                return;
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
                if strcmp(det.type, 'pothole') || det.id == lead_det.id, continue; end
                
                % Longitudinal conflict window:
                % Oncoming vehicles close rapidly -> require 50m clear horizon
                % Same-direction vehicles -> conflict window is the active passing envelope
                if det.is_oncoming
                    max_dx_conflict = 50.0;
                else
                    max_dx_conflict = max(18.0, lead_det.dx + 8.0);
                end
                
                if det.dx > -7.5 && det.dx < max_dx_conflict
                    % Check if vehicle or static obstacle is in passing corridor (occupying left passing lane)
                    if det.dy > 0.40
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
            
            if ego.v < 1.5 && lead_det.v > 1.0
                is_feasible = false;
                block_reason = 'SPEED_TOO_LOW';
                return;
            end
            
            min_dx_req = 3.5;
            if lead_det.v > 1.0, min_dx_req = 7.0; end
            if lead_det.dx < min_dx_req
                is_feasible = false;
                block_reason = 'DISTANCE_TOO_SHORT';
                return;
            end
        end
    end
end
