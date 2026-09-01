classdef DynamicMetrics < handle
    properties
        scenario_id
        trial_id
        scenario_name
        
        % Timeline vectors
        t_vec
        ego_x_vec
        ego_y_vec
        ego_v_vec
        ego_th_vec
        y_min_vec
        y_max_vec
        corridor_w_vec
        mpc_status_vec
        sf_active_vec
        scenario_clearance_vec
        env_clearance_vec
        global_clearance_vec
        
        % Event Timestamps & Telemetry
        detection_time_s = NaN
        emergency_stop_time_s = NaN
        stop_position_m = NaN
        minimum_herd_clearance_m = inf
        min_environment_clearance_m = inf
        min_global_clearance_m = inf
        herd_clear_time_s = NaN
        corridor_reopen_time_s = NaN
        safety_filter_release_time_s = NaN
        resume_time_s = NaN
        cruising_resumed_time_s = NaN
        
        % Latency Breakdown Metrics
        recovery_latency_s = NaN
        full_stop_recovery_latency_s = NaN
        yield_recovery_latency_s = NaN
        
        % Event Flags & Classification Mode
        blockage_detected = false
        safe_stop = false
        safe_deceleration = false
        obstacle_cleared = false
        corridor_reopened = false
        controller_released = false
        vehicle_resumed = false
        cruising_resumed = false
        
        % Categorical Recovery Mode ('FULL_STOP', 'DYNAMIC_YIELD', or 'NONE')
        recovery_mode = 'NONE'
        min_ego_v_m_s = inf
        mean_pos_rmse = 0.0
        mean_vel_rmse = 0.0
        
        % Collision & Success Outcomes
        scenario_obstacle_collision = false
        unrelated_agent_collision = false
        global_collision = false
        post_recovery_collision = false
        recovery_success = false
        success = false
        
        % Counter telemetry
        planner_prevented_count = 0
        safety_filter_saved_count = 0
    end
    
    methods
        function obj = DynamicMetrics(scen_id, tr_id, scen_name)
            if nargin > 0
                obj.scenario_id = scen_id;
                obj.trial_id = tr_id;
                obj.scenario_name = scen_name;
            end
        end
        
        function decel = getMaxDeceleration(obj)
            if length(obj.ego_v_vec) < 2 || length(obj.t_vec) < 2
                decel = 0.0; return;
            end
            dv = diff(obj.ego_v_vec);
            dt = diff(obj.t_vec);
            decel = max(0.0, max(-dv ./ max(1e-4, dt)));
        end
        
        function pct = getPlannerActivePct(obj)
            pct = obj.getMPCFeasibleRate();
        end
        
        function pct = getMPCFeasibleRate(obj)
            if isempty(obj.mpc_status_vec)
                pct = 0.0; return;
            end
            pct = sum(obj.mpc_status_vec == 1) / max(1, length(obj.mpc_status_vec)) * 100;
        end
        
        function pct = getSafetyFilterActivePct(obj)
            pct = obj.getSafetyFilterInterventionRate();
        end
        
        function pct = getSafetyFilterInterventionRate(obj)
            if isempty(obj.sf_active_vec)
                pct = 0.0; return;
            end
            pct = sum(obj.sf_active_vec == 1) / max(1, length(obj.sf_active_vec)) * 100;
        end
    end
end
