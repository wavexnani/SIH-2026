classdef SafetyFilter < handle
    % SAFETYFILTER Emergency Safety Filter & Trajectory Safeguard (Stage 4)
    %
    % Purpose:
    %   Validates planned control inputs and trajectories against physical vehicle
    %   limits, road bounds, and collision hazards. If primary MPC fails, it triggers
    %   Layer 2 Controlled Emergency Deceleration (a_cmd = -3.0 m/s^2, delta_cmd = 0.0).
    
    properties
        max_decel       double = -3.0       % Emergency braking deceleration (m/s^2)
        min_clearance   double = 0.05       % Safety threshold (m)
        vehicle_width   double = 1.80       % Vehicle width (m)
        vehicle_length  double = 4.70       % Vehicle length (m)
        wheelbase       double = 2.70       % Wheelbase (m)
    end
    
    methods
        function obj = SafetyFilter(varargin)
            if nargin > 0 && isa(varargin{1}, 'SimulationConfig')
                cfg = varargin{1};
                obj.vehicle_width = cfg.vehicle_width;
                obj.vehicle_length = cfg.vehicle_length;
                obj.wheelbase = cfg.wheelbase;
            end
        end
        
        function [u_safe, filter_active, filter_reason] = filter(obj, u_mpc, status, world, pred_states, bound_provider)
            % FILTER Independent verification of trajectory safety and emergency override.
            
            if nargin < 6, bound_provider = []; end
            
            filter_active = false;
            filter_reason = 'none';
            u_safe = u_mpc;
            
            % 1. Local Road Bounds Verification at Current Ego Position
            half_width = obj.vehicle_width / 2 + 0.15; % Physical vehicle half-width (0.90m) + 0.15m safety margin (1.05m)
            [y_min_curr, y_max_curr] = obj.getRoadBoundsAt(world, world.ego.x, bound_provider);
            y_min_safe_curr = y_min_curr + half_width;
            y_max_safe_curr = y_max_curr - half_width;
            
            % Active proportional PD re-centering steering calculation for emergency interventions
            k_p = 0.15; k_d = 0.50;
            if world.ego.y > y_max_safe_curr
                delta_steer_safe = -k_p * (world.ego.y - y_max_safe_curr) - k_d * world.ego.theta;
            elseif world.ego.y < y_min_safe_curr
                delta_steer_safe = k_p * (y_min_safe_curr - world.ego.y) - k_d * world.ego.theta;
            else
                delta_steer_safe = -k_d * world.ego.theta;
            end
            delta_steer_safe = max(-0.15, min(0.15, delta_steer_safe));
            
            % 2. Check Primary MPC Status
            if status == 0
                filter_active = true;
                filter_reason = 'mpc_infeasible_layer2_emergency_braking';
                u_safe = [delta_steer_safe; obj.max_decel]; % Full Layer 2 Controlled Emergency Deceleration (-3.0 m/s^2)
                return;
            end
            
            if ~isempty(pred_states)
                % 3. Check Predicted Road-Bound Compliance across Horizon
                Np = size(pred_states, 1);
                for k = 1:Np
                    px_k = pred_states(k, 1);
                    py_k = pred_states(k, 2);
                    
                    [y_min_k, y_max_k] = obj.getRoadBoundsAt(world, px_k, bound_provider);
                    y_min_safe_k = y_min_k + half_width;
                    y_max_safe_k = y_max_k - half_width;
                    
                    if py_k < y_min_safe_k || py_k > y_max_safe_k
                        filter_active = true;
                        filter_reason = 'predicted_road_bound_violation';
                        u_safe = [delta_steer_safe; -0.20];
                        return;
                    end
                end
                
                % 4. Independent 2D Bounding-Box Footprint Collision Verification (Static Obstacles & Dynamic Agents)
                obs_list = []; % [x, y, L, W, vx, vy]
                if isprop(world, 'n_static_obs') && world.n_static_obs > 0
                    for obs_i = 1:world.n_static_obs
                        obs_x = world.static_obs(obs_i, 1);
                        obs_y = world.static_obs(obs_i, 2);
                        if obs_x < -10.0 || obs_x < (world.ego.x - 3.0), continue; end
                        obs_L = 1.0; obs_W = 1.0;
                        if size(world.static_obs, 2) >= 4
                            obs_L = world.static_obs(obs_i, 3);
                            obs_W = world.static_obs(obs_i, 4);
                        end
                        obs_list = [obs_list; obs_x, obs_y, obs_L, obs_W, 0.0, 0.0];
                    end
                end
                
                if isprop(world, 'agents') && ~isempty(world.agents) && isprop(world, 'n_agents') && world.n_agents > 0
                    for ag_i = 1:world.n_agents
                        ag = world.agents(ag_i);
                        if isempty(ag) || ag.id <= 0 || ag.x < (world.ego.x - 3.0) || ag.x > (world.ego.x + 35.0), continue; end
                        ag_L = 0.8; ag_W = 0.5;
                        if isprop(ag, 'length') && ag.length > 0, ag_L = ag.length; end
                        if isprop(ag, 'width') && ag.width > 0, ag_W = ag.width; end
                        obs_list = [obs_list; ag.x, ag.y, ag_L, ag_W, ag.vx, ag.vy];
                    end
                end
                
                if ~isempty(obs_list)
                    body_center_offset = obj.wheelbase / 2.0; % 1.35m
                    dt_step = 0.10; % MPC discretization step
                    for o_i = 1:size(obs_list, 1)
                        o_x = obs_list(o_i, 1);
                        o_y = obs_list(o_i, 2);
                        o_L = obs_list(o_i, 3);
                        o_W = obs_list(o_i, 4);
                        o_vx = obs_list(o_i, 5);
                        o_vy = obs_list(o_i, 6);
                        
                        dx_overlap = (obj.vehicle_length + o_L) / 2.0;
                        dy_req = (obj.vehicle_width + o_W) / 2.0 + obj.min_clearance;
                        
                        for k = 1:Np
                            px_k = pred_states(k, 1);
                            py_k = pred_states(k, 2);
                            pth_k = 0.0;
                            if size(pred_states, 2) >= 3, pth_k = pred_states(k, 3); end
                            
                            p_bc_x = px_k + body_center_offset * cos(pth_k);
                            p_bc_y = py_k + body_center_offset * sin(pth_k);
                            
                            % Forward project dynamic agent position at step k
                            t_k = k * dt_step;
                            pred_o_x = o_x + o_vx * t_k;
                            pred_o_y = o_y + o_vy * t_k;
                            
                            if abs(p_bc_x - pred_o_x) <= dx_overlap && abs(p_bc_y - pred_o_y) < dy_req
                                filter_active = true;
                                filter_reason = 'predicted_obstacle_clearance_violation';
                                u_safe = [0.0; obj.max_decel];
                                return;
                            end
                        end
                    end
                end
            end
            
            % 5. Check Velocity Limits
            ego = world.ego;
            if ego.v <= 0.1 && u_safe(2) < 0
                u_safe(2) = 0.0;
            end
        end
    end
    
    methods (Access = private)
        function [y_min, y_max] = getRoadBoundsAt(obj, world, x, bound_provider)
            % GETROADBOUNDSAT Dynamically queries position-dependent road bounds.
            if ~isempty(bound_provider)
                if isprop(bound_provider, 'map') && ~isempty(bound_provider.map)
                    [y_min, y_max] = bound_provider.map.getRoadBoundsAt(x);
                    return;
                end
            end
            if isprop(world, 'map') && ~isempty(world.map)
                [y_min, y_max] = world.map.getRoadBoundsAt(x);
            else
                bounds = world.getRoadBounds();
                y_min = bounds(3);
                y_max = bounds(4);
            end
        end
    end
end
