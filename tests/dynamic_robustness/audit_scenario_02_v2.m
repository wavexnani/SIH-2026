function audit_scenario_02_v2()
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common', 'tests/dynamic_robustness');

out_dir = 'tests/dynamic_robustness/results/scenario_02_audit_v2';
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

fprintf('\n========================================================================================\n');
fprintf('         STEP 1 & 2: AUDITING ALL 50 TRIALS OF SCENARIO 02 (V2)                        \n');
fprintf('========================================================================================\n');

cfg = SimulationConfig();
n_trials = 50;

% Load existing benchmark results if available
mat_file = 'tests/dynamic_robustness/results/dynamic_robustness_benchmark_results.mat';
if exist(mat_file, 'file')
    data = load(mat_file);
    metrics_cell = data.all_metrics{2};
else
    [~, metrics_cell] = run_scenario_02_herd_clears(n_trials);
end

% 1. Collect trial-by-trial detailed metrics
trial_table = struct([]);
n_recovered = 0;
n_timeout = 0;
n_ctrl_fail = 0;

for tr = 1:n_trials
    m = metrics_cell{tr};
    trial_seed = uint32(42 + 2 * 1000 + tr);
    
    % Reconstruct initial herd parameters for exact physics audit
    rng(trial_seed);
    herd_x_center = 50.0 + (rand() - 0.5) * 4.0;
    herd_vy = 0.50 + 0.10 * (rand() - 0.5);
    
    % Geometry calculation: Goats start at y ∈ [-1.0, 0.4]. Road bounds y ∈ [0.0, 6.0].
    % Upper lane boundary is y = 4.0m. Highest goat y_init is ~0.4m.
    % Required vertical travel distance Δy ≈ 4.0 - (-1.0) = 5.0m max, or 4.0 - 0.4 = 3.6m for trailing goat.
    % Theoretical time to clear upper boundary: t_clear_theo = (4.0 - y_start_lowest) / herd_vy.
    % y_start_lowest = -1.0m => t_clear_theo = 5.0 / herd_vy.
    
    t_clear_theo = 5.0 / herd_vy;
    
    % Event timestamps
    t_block = m.detection_time_s;
    t_sf_act = m.detection_time_s; % SafetyFilter activates at detection
    t_stop = m.emergency_stop_time_s;
    x_stop = m.stop_position_m;
    t_clear = m.herd_clear_time_s;
    t_reopen = m.corridor_reopen_time_s;
    t_sf_rel = m.safety_filter_release_time_s;
    t_resume = m.resume_time_s;
    
    final_x = m.ego_x_vec(end);
    final_v = m.ego_v_vec(end);
    min_clr = m.minimum_herd_clearance_m;
    scen_coll = m.scenario_obstacle_collision;
    glob_coll = m.global_collision;
    rec_succ = m.recovery_success;
    
    % Categorize failure / outcome
    fail_reason = 'NONE (SUCCESS)';
    category = 'SUCCESS';
    
    if scen_coll
        fail_reason = 'A — Scenario Collision';
        category = 'COLLISION';
        n_ctrl_fail = n_ctrl_fail + 1;
    elseif ~m.safe_stop
        fail_reason = 'A — Vehicle Never Stopped Safely';
        category = 'CTRL_FAIL';
        n_ctrl_fail = n_ctrl_fail + 1;
    elseif isnan(t_clear) || isnan(t_reopen)
        fail_reason = sprintf('B — Herd Never Cleared Before Cutoff (vy=%.3fm/s, req t_clear=%.2fs > 25s cutoff)', herd_vy, t_clear_theo);
        category = 'TIMEOUT';
        n_timeout = n_timeout + 1;
    elseif isnan(t_sf_rel)
        fail_reason = 'D — Corridor Reopened but SafetyFilter Never Released';
        category = 'CTRL_FAIL';
        n_ctrl_fail = n_ctrl_fail + 1;
    elseif isnan(t_resume)
        fail_reason = 'E — SafetyFilter Released but Vehicle Never Resumed';
        category = 'CTRL_FAIL';
        n_ctrl_fail = n_ctrl_fail + 1;
    else
        n_recovered = n_recovered + 1;
    end
    
    row.trial = tr;
    row.seed = trial_seed;
    row.herd_x = herd_x_center;
    row.herd_vy = herd_vy;
    row.t_clear_theo = t_clear_theo;
    row.t_block = t_block;
    row.t_stop = t_stop;
    row.x_stop = x_stop;
    row.t_clear = t_clear;
    row.t_reopen = t_reopen;
    row.t_sf_rel = t_sf_rel;
    row.t_resume = t_resume;
    row.final_x = final_x;
    row.final_v = final_v;
    row.min_clr = min_clr;
    row.scen_coll = scen_coll;
    row.glob_coll = glob_coll;
    row.rec_succ = rec_succ;
    row.category = category;
    row.fail_reason = fail_reason;
    
    trial_table(tr).data = row;
end

% 2. Print Latency breakdown for recovered trials (Step 5)
fprintf('\n--- STEP 5: RECOVERY LATENCY BREAKDOWN FOR RECOVERED TRIALS ---\n');
fprintf('%-6s | %-12s | %-12s | %-12s | %-16s\n', 'Trial', 'Reopen Time', 'SF Release', 'Resume Time', 'Recovery Latency');
fprintf('-----------------------------------------------------------------------------------\n');
latencies = [];
for tr = 1:n_trials
    r = trial_table(tr).data;
    if r.rec_succ
        lat = r.t_resume - r.t_reopen;
        latencies = [latencies; lat];
        fprintf('%-6d | %-12.2f | %-12.2f | %-12.2f | %16.2f s\n', tr, r.t_reopen, r.t_sf_rel, r.t_resume, lat);
    end
end

fprintf('\n--- LATENCY STATISTICAL ANALYSIS ---\n');
fprintf('Count: %d trials\n', length(latencies));
fprintf('Mean:   %.4f s\n', mean(latencies));
fprintf('Median: %.4f s\n', median(latencies));
fprintf('Min:    %.4f s\n', min(latencies));
fprintf('Max:    %.4f s\n', max(latencies));
fprintf('Std:    %.4f s\n', std(latencies));

% Save table to mat
save(fullfile(out_dir, 'scenario_02_audit_table.mat'), 'trial_table', 'latencies');

end
