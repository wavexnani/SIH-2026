function audit_horizon_and_sensitivity()
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common', 'tests/dynamic_robustness');

out_dir = 'tests/dynamic_robustness/results/scenario_02_audit_v2';
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

fprintf('\n========================================================================================\n');
fprintf('         STEP 4 & 8: HORIZON AUDIT & SENSITIVITY EXPERIMENT (25s, 30s, 35s, 40s)        \n');
fprintf('========================================================================================\n');

cfg = SimulationConfig();
n_trials = 50;

% 1. Physics Audit of Herd Vy vs Clear Time across all 50 trials
vy_vec = zeros(n_trials, 1);
t_clear_theo_vec = zeros(n_trials, 1);
t_clear_obs_vec = zeros(n_trials, 1);

for tr = 1:n_trials
    trial_seed = uint32(42 + 2 * 1000 + tr);
    rng(trial_seed);
    herd_x = 50.0 + (rand() - 0.5) * 4.0;
    herd_vy = 0.50 + 0.10 * (rand() - 0.5);
    
    % Herd geometry: lowest goat starts at y = -1.0m. Corridor upper bound is y = 4.0m.
    % Distance to clear upper bound = 4.0 - (-1.0) = 5.0m.
    % Required clear time = 5.0 / herd_vy.
    t_clear_theo = 5.0 / herd_vy;
    
    vy_vec(tr) = herd_vy;
    t_clear_theo_vec(tr) = t_clear_theo;
end

% Plot Vy vs Theoretical Clear Time
fig1 = figure('Visible', 'off', 'Position', [100, 100, 800, 500]);
scatter(vy_vec, t_clear_theo_vec, 60, 'b', 'filled');
hold on;
yline(25.0, 'r--', '25s Benchmark Horizon Cutoff', 'LineWidth', 2);
xlabel('Herd Crossing Speed v_y (m/s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Theoretical Herd Clear Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
title('Scenario 02: Herd Crossing Speed vs Physical Clearance Time', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
saveas(fig1, fullfile(out_dir, 'herd_vy_vs_cleartime.png'));
close(fig1);

% 2. Horizon Sensitivity Experiment for 25s, 30s, 35s, 40s
horizons = [25, 30, 35, 40];
sens_results = struct([]);

fprintf('\n%-10s | %-23s | %-10s | %-12s | %-18s | %-10s\n', ...
    'Horizon', 'Scenario Collision-Free', 'Safe Stop', 'Herd Cleared', 'Recovery Completed', 'Timeout');
fprintf('---------------------------------------------------------------------------------------------------\n');

for hi = 1:length(horizons)
    H_sec = horizons(hi);
    N_steps_H = round(H_sec / cfg.dt);
    
    n_scen_free = 0;
    n_safe_stop = 0;
    n_cleared = 0;
    n_recovered = 0;
    n_timeout = 0;
    
    for tr = 1:n_trials
        trial_seed = uint32(42 + 2 * 1000 + tr);
        rng(trial_seed);
        
        builder_fn = @(cfg_in, seed_in) build_scenario_02_helper(cfg_in, seed_in);
        
        % Run custom trial with horizon cutoff H_sec
        m = run_custom_horizon_trial(2, tr, builder_fn, cfg, N_steps_H);
        
        if ~m.scenario_obstacle_collision, n_scen_free = n_scen_free + 1; end
        if m.safe_stop, n_safe_stop = n_safe_stop + 1; end
        if m.corridor_reopened, n_cleared = n_cleared + 1; end
        if m.recovery_success, n_recovered = n_recovered + 1; end
        if ~m.corridor_reopened, n_timeout = n_timeout + 1; end
    end
    
    sens_results(hi).horizon = H_sec;
    sens_results(hi).scen_free_pct = (n_scen_free / n_trials) * 100;
    sens_results(hi).safe_stop_pct = (n_safe_stop / n_trials) * 100;
    sens_results(hi).cleared_pct   = (n_cleared / n_trials) * 100;
    sens_results(hi).recovered_pct = (n_recovered / n_trials) * 100;
    sens_results(hi).timeout_pct   = (n_timeout / n_trials) * 100;
    
    fprintf('%-10s | %-23s | %-10s | %-12s | %-18s | %-10s\n', ...
        sprintf('%d s', H_sec), ...
        sprintf('%.1f%% (%d/%d)', sens_results(hi).scen_free_pct, n_scen_free, n_trials), ...
        sprintf('%.1f%%', sens_results(hi).safe_stop_pct), ...
        sprintf('%.1f%%', sens_results(hi).cleared_pct), ...
        sprintf('%.1f%% (%d/%d)', sens_results(hi).recovered_pct, n_recovered, n_trials), ...
        sprintf('%.1f%% (%d/%d)', sens_results(hi).timeout_pct, n_timeout, n_trials));
end
fprintf('================================================================================-------------------\n\n');

% Plot Sensitivity Bar Chart
fig2 = figure('Visible', 'off', 'Position', [100, 100, 800, 500]);
bar_data = [[sens_results.recovered_pct]', [sens_results.timeout_pct]'];
b = bar(horizons, bar_data, 'stacked');
b(1).FaceColor = [0.1, 0.7, 0.3];
b(2).FaceColor = [0.85, 0.4, 0.1];
xlabel('Simulation Horizon Cutoff (seconds)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Percentage of 50 Monte Carlo Trials (%)', 'FontSize', 12, 'FontWeight', 'bold');
title('Scenario 02: Recovery Completion vs Horizon Cutoff Duration', 'FontSize', 14, 'FontWeight', 'bold');
legend({'Recovery Completed (Herd Cleared & Ego Resumed)', 'Scenario Timeout (Herd Still Crossing Road at Cutoff)'}, 'Location', 'east');
grid on;
saveas(fig2, fullfile(out_dir, 'horizon_sensitivity_plot.png'));
close(fig2);

save(fullfile(out_dir, 'horizon_sensitivity_data.mat'), 'sens_results', 'vy_vec', 't_clear_theo_vec');

end

function metric = run_custom_horizon_trial(scenario_id, trial_id, scenario_builder_fn, cfg, N_steps)
trial_seed = uint32(42 + scenario_id * 1000 + trial_id);
rng(trial_seed);

[world, custom_updater] = scenario_builder_fn(cfg, trial_seed);
scenario_agent_ids = 1:world.n_agents;

metric = DynamicMetrics(scenario_id, trial_id, 'Herd Clears Recovery');
metric.t_vec = zeros(N_steps, 1);
metric.ego_x_vec = zeros(N_steps, 1);
metric.ego_y_vec = zeros(N_steps, 1);
metric.ego_v_vec = zeros(N_steps, 1);
metric.corridor_w_vec = zeros(N_steps, 1);
metric.mpc_status_vec = zeros(N_steps, 1);
metric.sf_active_vec = zeros(N_steps, 1);
metric.scenario_clearance_vec = zeros(N_steps, 1);

map_obj = FreeSpaceMap('unstructured');
map_obj.y_min_base = 0.0; map_obj.y_max_base = 6.0;

bp = FreeSpaceBoundProvider(map_obj, cfg.vehicle_width / 2.0);
planner = CACRCPlanner(cfg); planner.bound_provider = bp;
sf = SafetyFilter(cfg); vehicle = BicycleModel(cfg);
obs_model = ObservationModel('ideal', trial_seed);

ref_path = zeros(900, 5);
ref_path(:,1) = linspace(0, 200, 900)'; ref_path(:,2) = 2.5; ref_path(:,5) = 5.0;

sf_active_prev = false; dt = cfg.dt;

for k = 1:N_steps
    t = (k-1)*dt;
    metric.t_vec(k) = t; metric.ego_x_vec(k) = world.ego.x; metric.ego_y_vec(k) = world.ego.y; metric.ego_v_vec(k) = world.ego.v;
    
    if ~isempty(custom_updater), world = custom_updater(world, t, k, dt); end
    [obs_world, ~] = obs_model.observe(world, dt);
    
    obs_x = mean([world.agents.x]);
    [ymn_obs, ymx_obs] = map_obj.extractLocalBounds(obs_x, obs_world, cfg.vehicle_width / 2.0);
    w_corr_obs = ymx_obs(1) - ymn_obs(1);
    
    if (w_corr_obs < 1.60 || ymn_obs(1) > ymx_obs(1)) && ~metric.blockage_detected
        metric.blockage_detected = true; metric.detection_time_s = t;
    end
    
    [u_mpc, pred_states, status, info] = planner.plan(obs_world, ref_path, 5.0);
    [u_cmd, filter_active, filter_reason] = sf.filter(u_mpc, status, world, pred_states, bp);
    
    if filter_active && ~sf_active_prev && isnan(metric.detection_time_s)
        metric.blockage_detected = true; metric.detection_time_s = t;
    end
    if sf_active_prev && ~filter_active && isnan(metric.safety_filter_release_time_s)
        metric.controller_released = true; metric.safety_filter_release_time_s = t;
    end
    sf_active_prev = filter_active;
    
    if metric.blockage_detected && world.ego.v <= 0.05 && isnan(metric.emergency_stop_time_s) && world.ego.x < obs_x
        metric.safe_stop = true; metric.emergency_stop_time_s = t; metric.stop_position_m = world.ego.x;
    end
    
    if metric.blockage_detected && w_corr_obs >= 1.60 && ymn_obs(1) <= ymx_obs(1)
        if isnan(metric.corridor_reopen_time_s)
            metric.corridor_reopened = true; metric.corridor_reopen_time_s = t;
            metric.herd_clear_time_s = t; metric.obstacle_cleared = true;
        end
    end
    
    if metric.safe_stop && metric.corridor_reopened && world.ego.v >= 1.0 && isnan(metric.resume_time_s)
        metric.vehicle_resumed = true; metric.resume_time_s = t; metric.recovery_success = true;
    end
    
    c_scen = compute_scen_clearance_helper(world, cfg);
    metric.scenario_clearance_vec(k) = c_scen;
    if c_scen < metric.minimum_herd_clearance_m, metric.minimum_herd_clearance_m = c_scen; end
    if c_scen < 0.0, metric.scenario_obstacle_collision = true; end
    
    world.ego = vehicle.stepKinematic(world.ego, u_cmd(2), u_cmd(1), dt);
    for ai = 1:world.n_agents
        world.agents(ai).x = world.agents(ai).x + world.agents(ai).vx * dt;
        world.agents(ai).y = world.agents(ai).y + world.agents(ai).vy * dt;
    end
end
end

function [world, custom_updater] = build_scenario_02_helper(cfg, trial_seed)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);

n_goats = 40;
world.n_agents = n_goats;

rng(trial_seed);
herd_x_center = 50.0 + (rand() - 0.5) * 4.0;
vy_fast = 0.50 + 0.10 * (rand() - 0.5);

for gi = 1:n_goats
    gx = herd_x_center + (rand() - 0.5) * 4.0;
    gy = -1.0 + 0.2 * mod(gi-1, 8);
    gvy = vy_fast;
    gvx = 0.0;
    world = world.setAgentState(gi, gx, gy, gvx, gvy, max(0.30, cfg.sigma_agent));
    world.agents(gi).length = 0.8; world.agents(gi).width = 0.5;
end

custom_updater = [];
end

function c_min = compute_scen_clearance_helper(world, cfg)
c_min = inf;
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
    
    if dist < c_min, c_min = dist; end
end
end
