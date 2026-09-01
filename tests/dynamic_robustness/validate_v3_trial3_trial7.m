function validate_v3_trial3_trial7()
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common', 'tests/dynamic_robustness');

cfg = SimulationConfig();
trials_to_run = [3, 4, 7];
seeds = [uint32(2045), uint32(2046), uint32(2049)];

fprintf('\n========================================================================================\n');
fprintf('            STEP 7: DETERMINISTIC VALIDATION FOR TRIAL 3 & TRIAL 7 (V3)                 \n');
fprintf('========================================================================================\n');
fprintf('%-6s | %-15s | %-17s | %-22s | %-10s | %-17s | %-8s | %-16s\n', ...
    'Trial', 'Recovery Mode', 'Blockage Detected', 'Min Scenario Clearance', 'Herd Clear', 'Corridor Reopened', 'Resume', 'Recovery Success');
fprintf('-----------------------------------------------------------------------------------------------------------------------------------\n');

for i = 1:length(trials_to_run)
    tr_id = trials_to_run(i);
    sd = seeds(i);
    
    m = DynamicScenarioRunner.runTrial(2, tr_id, @(cfg_in, seed_in) build_scenario_02_helper(cfg_in, seed_in), cfg);
    
    bd_str = sprintf('%d (t=%.2fs)', m.blockage_detected, m.detection_time_s);
    hc_str = sprintf('%d (t=%.2fs)', m.obstacle_cleared, m.herd_clear_time_s);
    cr_str = sprintf('%d (t=%.2fs)', m.corridor_reopened, m.corridor_reopen_time_s);
    
    res_t = m.resume_time_s;
    if isnan(res_t), res_t = m.cruising_resumed_time_s; end
    res_str = sprintf('%d (t=%.2fs)', m.vehicle_resumed, res_t);
    
    fprintf('%-6d | %-15s | %-17s | %-22.4f | %-10s | %-17s | %-8s | %-16d\n', ...
        tr_id, m.recovery_mode, bd_str, m.minimum_herd_clearance_m, hc_str, cr_str, res_str, m.recovery_success);
end
fprintf('================================================================================---------------------------------------------------\n\n');
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
