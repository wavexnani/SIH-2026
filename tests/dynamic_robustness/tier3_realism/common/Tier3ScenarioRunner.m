classdef Tier3ScenarioRunner < handle
    % TIER3SCENARIORUNNER Simulation runner for Tier-3 Realism & Uncertainty Evaluation
    %
    % Preserves exact frozen core planning & safety stack while allowing
    % configurable perception uncertainty (position noise, velocity noise, delay)
    % via the existing ObservationModel abstraction.
    
    methods (Static)
        function metric = runTrial(scenario_id, trial_id, scenario_builder_fn, cfg_opt, perception_cfg)
            if nargin < 4 || isempty(cfg_opt)
                cfg = SimulationConfig();
            else
                cfg = cfg_opt;
            end
            
            if nargin < 5 || isempty(perception_cfg)
                perception_cfg = struct('mode', 'ideal', 'sigma_pos', 0.0, 'sigma_vel', 0.0, ...
                    'sigma_theta', 0.0, 'tau_delay', 0.0);
            end
            
            dt = cfg.dt;
            N_steps = 250;
            
            % Generate deterministic seed for this trial
            trial_seed = uint32(42 + scenario_id * 1000 + trial_id);
            rng(trial_seed);
            
            % Initialize world using scenario builder
            [world, custom_updater] = scenario_builder_fn(cfg, trial_seed);
            
            % Identify scenario agents vs background environment agents
            scenario_agent_ids = get_scenario_agent_ids(world, scenario_id);
            
            metric = DynamicMetrics(scenario_id, trial_id, get_scenario_title(scenario_id));
            metric.t_vec = zeros(N_steps, 1);
            metric.ego_x_vec = zeros(N_steps, 1);
            metric.ego_y_vec = zeros(N_steps, 1);
            metric.ego_v_vec = zeros(N_steps, 1);
            metric.ego_th_vec = zeros(N_steps, 1);
            metric.y_min_vec = zeros(N_steps, 1);
            metric.y_max_vec = zeros(N_steps, 1);
            metric.corridor_w_vec = zeros(N_steps, 1);
            metric.mpc_status_vec = zeros(N_steps, 1);
            metric.sf_active_vec = zeros(N_steps, 1);
            metric.scenario_clearance_vec = zeros(N_steps, 1);
            metric.env_clearance_vec = zeros(N_steps, 1);
            metric.global_clearance_vec = zeros(N_steps, 1);
            
            % Tier-3 Perception Tracking
            pos_rmse_vec = zeros(N_steps, 1);
            vel_rmse_vec = zeros(N_steps, 1);
            
            % Initialize Frozen Baseline Components
            map_obj = FreeSpaceMap('unstructured');
            map_obj.y_min_base = 0.0;
            map_obj.y_max_base = 6.0;
            
            bp = FreeSpaceBoundProvider(map_obj, cfg.vehicle_width / 2.0);
            planner = CACRCPlanner(cfg);
            planner.bound_provider = bp;
            sf = SafetyFilter(cfg);
            vehicle = BicycleModel(cfg);
            
            % Instantiate ObservationModel with Tier-3 perception configuration
            obs_model = ObservationModel(perception_cfg.mode, trial_seed, ...
                'sigma_pos', perception_cfg.sigma_pos, ...
                'sigma_vel', perception_cfg.sigma_vel, ...
                'sigma_theta', perception_cfg.sigma_theta, ...
                'tau_delay', perception_cfg.tau_delay);
            
            ref_path = zeros(900, 5);
            ref_path(:,1) = linspace(0, 200, 900)';
            ref_path(:,2) = 2.5;
            ref_path(:,5) = 5.0;
            
            sf_active_prev = false;
            
            for k = 1:N_steps
                t = (k-1)*dt;
                metric.t_vec(k) = t;
                metric.ego_x_vec(k) = world.ego.x;
                metric.ego_y_vec(k) = world.ego.y;
                metric.ego_v_vec(k) = world.ego.v;
                metric.ego_th_vec(k) = world.ego.theta;
                
                % Step custom dynamic scene updater if present
                if ~isempty(custom_updater)
                    world = custom_updater(world, t, k, dt);
                end
                
                % Observation Transformation (Ground Truth World -> Observed World)
                [obs_world, ~] = obs_model.observe(world, dt);
                
                % Compute Perception Position & Velocity RMSE (Observed vs Ground Truth)
                pos_err_sq = 0.0;
                vel_err_sq = 0.0;
                n_eval = 0;
                for ai = 1:world.n_agents
                    if world.agents(ai).id > 0 && world.agents(ai).x > -10.0
                        dx = obs_world.agents(ai).x - world.agents(ai).x;
                        dy = obs_world.agents(ai).y - world.agents(ai).y;
                        dvx = obs_world.agents(ai).vx - world.agents(ai).vx;
                        dvy = obs_world.agents(ai).vy - world.agents(ai).vy;
                        pos_err_sq = pos_err_sq + (dx^2 + dy^2);
                        vel_err_sq = vel_err_sq + (dvx^2 + dvy^2);
                        n_eval = n_eval + 1;
                    end
                end
                if n_eval > 0
                    pos_rmse_vec(k) = sqrt(pos_err_sq / n_eval);
                    vel_rmse_vec(k) = sqrt(vel_err_sq / n_eval);
                else
                    pos_rmse_vec(k) = 0.0;
                    vel_rmse_vec(k) = 0.0;
                end
                
                % Corridor bound extraction at Ego & obstacle horizon from OBSERVED world
                [ymn_ego, ymx_ego] = map_obj.extractLocalBounds(world.ego.x, obs_world, cfg.vehicle_width / 2.0);
                w_corr_ego = ymx_ego(1) - ymn_ego(1);
                metric.corridor_w_vec(k) = w_corr_ego;
                metric.y_min_vec(k) = ymn_ego(1);
                metric.y_max_vec(k) = ymx_ego(1);
                
                % Evaluate scenario obstacle bounds ahead from OBSERVED world
                obs_x = get_obstacle_x_location(world, scenario_agent_ids);
                [ymn_obs, ymx_obs] = map_obj.extractLocalBounds(obs_x, obs_world, cfg.vehicle_width / 2.0);
                w_corr_obs = ymx_obs(1) - ymn_obs(1);
                
                % Event: Blockage Detection
                if (w_corr_obs < 1.60 || ymn_obs(1) > ymx_obs(1)) && ~metric.blockage_detected
                    metric.blockage_detected = true;
                    metric.detection_time_s = t;
                end
                
                % Planner Execution on OBSERVED world
                [u_mpc, pred_states, status, info] = planner.plan(obs_world, ref_path, 5.0);
                metric.mpc_status_vec(k) = status;
                
                % Safety Filter Execution (Ground-Truth Supervisor using true world)
                [u_cmd, filter_active, filter_reason] = sf.filter(u_mpc, status, world, pred_states, bp);
                metric.sf_active_vec(k) = filter_active;
                
                % Track SafetyFilter Activation / Release
                if filter_active && ~sf_active_prev && isnan(metric.detection_time_s)
                    metric.blockage_detected = true;
                    metric.detection_time_s = t;
                end
                
                if sf_active_prev && ~filter_active && isnan(metric.safety_filter_release_time_s)
                    metric.controller_released = true;
                    metric.safety_filter_release_time_s = t;
                end
                sf_active_prev = filter_active;
                
                if status == 1 && ~filter_active
                    metric.planner_prevented_count = metric.planner_prevented_count + 1;
                elseif filter_active
                    metric.safety_filter_saved_count = metric.safety_filter_saved_count + 1;
                end
                
                % Track minimum velocity during interaction
                if world.ego.v < metric.min_ego_v_m_s
                    metric.min_ego_v_m_s = world.ego.v;
                end

                % Event: Safe Stop (Standstill v <= 0.05 m/s)
                if metric.blockage_detected && world.ego.v <= 0.05 && isnan(metric.emergency_stop_time_s) && world.ego.x < obs_x
                    metric.safe_stop = true;
                    metric.emergency_stop_time_s = t;
                    metric.stop_position_m = world.ego.x;
                    metric.recovery_mode = 'FULL_STOP';
                end

                % Event: Safe Deceleration / Dynamic Yield (v < 4.5 m/s while blockage exists)
                if metric.blockage_detected && world.ego.v < 4.5 && ~metric.safe_stop
                    metric.safe_deceleration = true;
                end
                
                % Event: Obstacle Cleared & Corridor Reopened
                if metric.blockage_detected && w_corr_obs >= 1.60 && ymn_obs(1) <= ymx_obs(1)
                    if isnan(metric.corridor_reopen_time_s)
                        metric.corridor_reopened = true;
                        metric.corridor_reopen_time_s = t;
                        metric.herd_clear_time_s = t;
                        metric.obstacle_cleared = true;
                    end
                end
                
                % Separate Footprint Clearance Calculations (Evaluated against GROUND TRUTH world)
                [c_scen, c_env, c_glob] = compute_separated_clearance(world, scenario_agent_ids, cfg);
                metric.scenario_clearance_vec(k) = c_scen;
                metric.env_clearance_vec(k) = c_env;
                metric.global_clearance_vec(k) = c_glob;
                
                if c_scen < metric.minimum_herd_clearance_m, metric.minimum_herd_clearance_m = c_scen; end
                if c_env < metric.min_environment_clearance_m, metric.min_environment_clearance_m = c_env; end
                if c_glob < metric.min_global_clearance_m, metric.min_global_clearance_m = c_glob; end
                
                if c_scen < 0.0, metric.scenario_obstacle_collision = true; end
                if c_env < 0.0, metric.unrelated_agent_collision = true; end
                if c_glob < 0.0, metric.global_collision = true; end
                
                % Step World Dynamics (Kinematic Ego + Constant Velocity Agents)
                world.ego = vehicle.stepKinematic(world.ego, u_cmd(2), u_cmd(1), dt);
                for ai = 1:world.n_agents
                    if world.agents(ai).id > 0
                        world.agents(ai).x = world.agents(ai).x + world.agents(ai).vx * dt;
                        world.agents(ai).y = world.agents(ai).y + world.agents(ai).vy * dt;
                    end
                end
            end
            
            % Compute empirical trial-wide mean position & velocity RMSE
            metric.mean_pos_rmse = mean(pos_rmse_vec);
            metric.mean_vel_rmse = mean(vel_rmse_vec);
            
            % Evaluate Overall Trial Success based on GROUND TRUTH collision
            if ~metric.scenario_obstacle_collision
                metric.success = true;
            else
                metric.success = false;
            end
        end
    end
end

function ids = get_scenario_agent_ids(world, scenario_id)
ids = [];
for i = 1:length(world.agents)
    ag = world.agents(i);
    if isempty(ag) || ag.id <= 0, continue; end
    if scenario_id == 1 || scenario_id == 2
        if ag.x >= 40.0 && ag.x <= 75.0
            ids = [ids, ag.id];
        end
    else
        ids = [ids, ag.id];
    end
end
if isempty(ids)
    ids = 1:length(world.agents);
end
end

function obs_x = get_obstacle_x_location(world, scenario_agent_ids)
x_vals = [];
for i = 1:length(world.agents)
    ag = world.agents(i);
    if ismember(ag.id, scenario_agent_ids)
        x_vals = [x_vals, ag.x];
    end
end
if isempty(x_vals)
    obs_x = world.ego.x + 10.0;
else
    obs_x = mean(x_vals);
end
end

function [c_scen, c_env, c_glob] = compute_separated_clearance(world, scenario_agent_ids, cfg)
c_scen = inf; c_env = inf; c_glob = inf;
ego = world.ego;
ego_L = cfg.vehicle_length; ego_W = cfg.vehicle_width;

for ai = 1:length(world.agents)
    ag = world.agents(ai);
    if isempty(ag) || ag.id <= 0, continue; end
    ag_L = 0.8; ag_W = 0.5;
    if isprop(ag, 'length') && ag.length > 0, ag_L = ag.length; end
    if isprop(ag, 'width') && ag.width > 0, ag_W = ag.width; end
    
    dx = abs(ego.x - ag.x) - (ego_L + ag_L)/2.0;
    dy = abs(ego.y - ag.y) - (ego_W + ag_W)/2.0;
    
    if dx < 0 && dy < 0
        dist = max(dx, dy);
    elseif dx >= 0 && dy >= 0
        dist = sqrt(dx^2 + dy^2);
    else
        dist = max(dx, dy);
    end
    
    if dist < c_glob, c_glob = dist; end
    if ismember(ag.id, scenario_agent_ids)
        if dist < c_scen, c_scen = dist; end
    else
        if dist < c_env, c_env = dist; end
    end
end
end

function name = get_scenario_title(scenario_id)
titles = {'Sudden Herd Entry', 'Herd Clears Recovery', 'Partial Gap Transition', ...
          'Opposite Gap Opens', 'Side Switch Topology Stress', 'Crossing Dynamic Agent'};
if scenario_id >= 1 && scenario_id <= 6
    name = titles{scenario_id};
else
    name = 'Custom Scenario';
end
end
