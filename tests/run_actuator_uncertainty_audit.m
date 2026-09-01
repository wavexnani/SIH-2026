function run_actuator_uncertainty_audit()
    % RUN_ACTUATOR_UNCERTAINTY_AUDIT Closed-Loop Actuator Uncertainty Benchmark Audit
    %
    % Purpose:
    %   Evaluates Demos 1, 2, 4, 5, 6 under ideal, steering_bias (+0.8 deg), and combined
    %   actuator modes to quantitatively measure closed-loop vehicle behavior, tracking errors,
    %   motion stability, safety margins, and solver performance.
    
    addpath('environment', 'core', 'vehicle', 'planning', 'config', 'stages', 'visualization');
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 12C: ACTUATOR UNCERTAINTY & CLOSED-LOOP REALISM BENCHMARK AUDIT           \n');
    fprintf('========================================================================================\n\n');
    
    scenarios = {
        'clear', 'Demo 1: Nominal';
        'static', 'Demo 2: Static Obstacle';
        'multi_obstacle_sequence', 'Demo 4: Dynamic Road Narrowing';
        'multi_vehicle_oncoming_conflict', 'Demo 5: Multi-Vehicle Oncoming Conflict';
        'impassable_center', 'Demo 6: Total Blockage'
    };
    
    modes = {'ideal', 'steering_bias', 'combined'};
    
    results = struct();
    
    for s_i = 1:size(scenarios, 1)
        scen_id = scenarios{s_i, 1};
        scen_label = scenarios{s_i, 2};
        
        fprintf('----------------------------------------------------------------------------------------\n');
        fprintf('  Auditing Scenario: %s (%s)\n', scen_label, scen_id);
        fprintf('----------------------------------------------------------------------------------------\n');
        
        for m_i = 1:length(modes)
            mode_name = modes{m_i};
            act_model = ActuatorUncertaintyModel(mode_name);
            obs_model = ObservationModel('ideal'); % Keep perception ideal to isolate actuator effect
            
            [~, met, hist] = stage5_multivehicle_coordination(...
                'scenario', scen_id, ...
                'obs_model', obs_model, ...
                'actuator_model', act_model, ...
                'verbose', false);
            
            % Compute Motion Metrics
            dt = 0.1;
            N = length(hist.t);
            dtheta = diff(hist.ego_theta); dtheta = atan2(sin(dtheta), cos(dtheta));
            yaw_rate = [0; dtheta / dt];
            lat_accel = abs(hist.ego_v .* yaw_rate);
            
            ddelta = diff(hist.delta_plant);
            steering_rate = [0; ddelta / dt];
            
            d_lat_accel = diff(lat_accel);
            lat_jerk = [0; d_lat_accel / dt];
            
            a_plant = hist.a_plant;
            da = diff(a_plant);
            long_jerk = [0; da / dt];
            
            max_yaw_rate_deg = rad2deg(max(abs(yaw_rate)));
            max_lat_accel_g = max(lat_accel) / 9.81;
            max_lat_jerk = max(abs(lat_jerk));
            max_long_accel = max(a_plant);
            max_long_decel = min(a_plant);
            max_long_jerk = max(abs(long_jerk));
            max_steer_rate_deg = rad2deg(max(abs(steering_rate)));
            
            % Compute Tracking Metrics
            y_ref = 1.80; % Nominal center line
            lat_rmse = sqrt(mean((hist.ego_y - y_ref).^2));
            heading_rmse = sqrt(mean(hist.ego_theta.^2));
            v_ref = 8.0;
            speed_rmse = sqrt(mean((hist.ego_v - v_ref).^2));
            
            % Classification Outcome
            if met.collision_steps > 0
                outcome = 'COLLISION';
            elseif met.bounds_steps < N
                outcome = 'UNSAFE_FAILURE';
            elseif sum(hist.solver_status == 0) > 5
                outcome = 'PLANNER_INFEASIBLE';
            elseif met.emergency_braking_count > 0 && met.final_v < 0.5
                outcome = 'SAFE_STOP';
            elseif lat_rmse > 0.5 || met.safety_rejected_count > 0
                outcome = 'DEGRADED_SAFE';
            else
                outcome = 'SUCCESS';
            end
            
            % Store Results
            res.scenario = scen_label;
            res.mode = mode_name;
            res.outcome = outcome;
            res.lat_rmse = lat_rmse;
            res.heading_rmse = heading_rmse;
            res.speed_rmse = speed_rmse;
            res.min_clr = met.min_clr;
            res.collision_steps = met.collision_steps;
            res.bounds_steps = met.bounds_steps;
            res.emergency_count = met.emergency_braking_count;
            res.hard_qp_count = met.hard_qp_count;
            res.soft_qp_count = met.soft_qp_count;
            res.qp_infeas = sum(hist.solver_status == 0);
            res.max_yaw_rate_deg = max_yaw_rate_deg;
            res.max_lat_accel_g = max_lat_accel_g;
            res.max_long_accel = max_long_accel;
            res.max_long_decel = max_long_decel;
            res.max_long_jerk = max_long_jerk;
            res.max_steer_rate_deg = max_steer_rate_deg;
            res.rms_steer_err_deg = rad2deg(met.rms_steering_error);
            res.max_steer_err_deg = rad2deg(met.max_steering_error);
            
            results.(strrep(scen_id, 'scenario_', '')).(mode_name) = res;
            
            fprintf('  [%-13s] Outcome: %-15s | Lat RMSE: %.3fm | Steer Err: %4.2f deg | Min Clr: %5.2fm | Emerg: %d\n', ...
                mode_name, outcome, lat_rmse, res.rms_steer_err_deg, met.min_clr, met.emergency_braking_count);
        end
        fprintf('\n');
    end
    
    fprintf('========================================================================================\n');
    fprintf('        SUMMARY COMPARISON ACROSS SCENARIOS (IDEAL vs STEERING_BIAS vs COMBINED)        \n');
    fprintf('========================================================================================\n');
    fprintf('| Scenario / Mode        | Outcome         | Lat RMSE (m) | Steer Err (deg) | Min Clr (m) | Max Lat Acc (g) |\n');
    fprintf('|------------------------|-----------------|--------------|-----------------|-------------|-----------------|\n');
    
    scen_keys = fieldnames(results);
    for k = 1:length(scen_keys)
        sk = scen_keys{k};
        for m_i = 1:length(modes)
            mn = modes{m_i};
            r = results.(sk).(mn);
            fprintf('| %-14s (%-7s) | %-15s | %12.4f | %15.2f | %11.4f | %15.2f |\n', ...
                sk, mn, r.outcome, r.lat_rmse, r.rms_steer_err_deg, r.min_clr, r.max_lat_accel_g);
        end
    end
    fprintf('========================================================================================\n\n');
end
