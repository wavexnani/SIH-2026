classdef RealismMetrics
    % REALISMMETRICS High-Fidelity Closed-Loop Performance & Realism Metrics Aggregator
    
    methods (Static)
        function met = compute(history, world, cfg)
            dt = cfg.dt;
            N = length(history.t);
            
            % 1. Motion Metrics
            dtheta = diff(history.ego_theta); dtheta = atan2(sin(dtheta), cos(dtheta));
            yaw_rate = [0; dtheta / dt];
            lat_accel = abs(history.ego_v .* yaw_rate);
            
            ddelta = diff(history.delta_plant);
            steering_rate = [0; ddelta / dt];
            
            d_lat_accel = diff(lat_accel);
            lat_jerk = [0; d_lat_accel / dt];
            
            a_plant = history.a_plant;
            da = diff(a_plant);
            long_jerk = [0; da / dt];
            
            met.motion.max_yaw_rate_deg = rad2deg(max(abs(yaw_rate)));
            met.motion.max_lat_accel_g = max(lat_accel) / 9.81;
            met.motion.rms_lat_accel_g = sqrt(mean(lat_accel.^2)) / 9.81;
            met.motion.max_lat_jerk = max(abs(lat_jerk));
            met.motion.max_long_accel = max(a_plant);
            met.motion.max_long_decel = min(a_plant);
            met.motion.max_long_jerk = max(abs(long_jerk));
            met.motion.max_steering_rate_deg = rad2deg(max(abs(steering_rate)));
            
            % 2. Tracking Metrics (Ego reference center y)
            ego_y_v = history.ego_y(:);
            if isfield(history, 'ref_y')
                y_ref_v = history.ref_y(:);
            else
                y_ref_v = 1.80;
            end
            met.tracking.lat_rmse = sqrt(mean((ego_y_v - y_ref_v).^2));
            met.tracking.heading_rmse = sqrt(mean(history.ego_theta.^2));
            v_target = 8.0;
            met.tracking.speed_rmse = sqrt(mean((history.ego_v - v_target).^2));
            met.tracking.dist_traveled = history.ego_x(end) - history.ego_x(1);
            
            % 3. Safety Metrics
            met.safety.min_obstacle_clearance = min(history.min_clearance);
            met.safety.collision_steps = sum(history.is_collision);
            met.safety.boundary_violations = sum(~history.inside_bounds);
            met.safety.emergency_interventions = sum(history.solver_status == 0);
            
            % 4. Planner Metrics
            met.planner.hard_qp_solves = sum(history.solver_status == 1 & ~history.filter_active & ~strcmp(history.selected_topology, 'FALLBACK'));
            met.planner.soft_fallbacks = sum(history.solver_status == 1 & strcmp(history.selected_topology, 'FALLBACK'));
            met.planner.qp_infeasibilities = sum(history.solver_status == 0);
            met.planner.mean_solve_time_ms = mean(history.solve_time_ms);
            met.planner.max_solve_time_ms = max(history.solve_time_ms);
            
            % 5. Perception Metrics
            met.perception.pos_rmse = sqrt(mean(history.perception_e_pos.^2));
            met.perception.vel_rmse = sqrt(mean(history.perception_e_vel.^2));
            met.perception.heading_rmse = sqrt(mean(history.perception_e_heading.^2));
            met.perception.mean_obs_age = mean(history.perception_age);
            
            % 6. Actuator Metrics
            met.actuator.rms_steering_error_deg = rad2deg(sqrt(mean(history.steering_error.^2)));
            met.actuator.max_steering_error_deg = rad2deg(max(abs(history.steering_error)));
            met.actuator.rms_acceleration_error = sqrt(mean(history.acceleration_error.^2));
            met.actuator.max_acceleration_error = max(abs(history.acceleration_error));
            
            % 7. Traffic & TTC Metrics
            if isfield(history, 'ego_agent_clearance')
                met.traffic.min_ego_agent_clearance = min(history.ego_agent_clearance);
            else
                met.traffic.min_ego_agent_clearance = met.safety.min_obstacle_clearance;
            end
            
            if isfield(history, 'min_ttc')
                valid_ttc = history.min_ttc(~isinf(history.min_ttc));
                if ~isempty(valid_ttc)
                    met.traffic.min_ttc = min(valid_ttc);
                else
                    met.traffic.min_ttc = Inf;
                end
            elseif isfield(history, 'ttc')
                valid_ttc = history.ttc(~isinf(history.ttc));
                if ~isempty(valid_ttc)
                    met.traffic.min_ttc = min(valid_ttc);
                else
                    met.traffic.min_ttc = Inf;
                end
            else
                met.traffic.min_ttc = Inf;
            end
        end
    end
end
