function [summary, metrics_cell] = run_scenario_01_sudden_herd(n_trials)
if nargin < 1, n_trials = 50; end
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common');

metrics_cell = cell(n_trials, 1);
fprintf('[Scenario 01: Sudden Herd Entry] Running %d Monte Carlo trials...\n', n_trials);

for trial_id = 1:n_trials
    scenario_builder_fn = @(cfg, trial_seed) build_scenario_01(cfg, trial_seed);
    metrics_cell{trial_id} = DynamicScenarioRunner.runTrial(1, trial_id, scenario_builder_fn);
end

summary = MetricsEvaluator.summarizeScenario('Sudden Herd Entry', metrics_cell);
end

function [world, custom_updater] = build_scenario_01(cfg, trial_seed)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);

n_goats = 60;
world.n_agents = n_goats;

rng(trial_seed);
herd_x_center = 65.0 + (rand() - 0.5) * 6.0;
vy_base = 0.25 + 0.08 * (rand() - 0.5);

for gi = 1:n_goats
    gx = herd_x_center + (rand() - 0.5) * 5.0;
    gy = -3.0 + 0.3 * mod(gi-1, 10) + 0.05 * randn();
    gy = max(-4.0, min(-0.5, gy));
    gvy = vy_base + 0.05 * (rand() - 0.5);
    gvx = -0.02 + 0.04 * (rand() - 0.5);
    world = world.setAgentState(gi, gx, gy, gvx, gvy, max(0.30, cfg.sigma_agent));
    world.agents(gi).length = 0.8; world.agents(gi).width = 0.5;
end

custom_updater = [];
end
