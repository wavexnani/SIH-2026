function [summary, metrics_cell] = run_scenario_02_herd_clears(n_trials)
if nargin < 1, n_trials = 50; end
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common');

metrics_cell = cell(n_trials, 1);
fprintf('[Scenario 02: Herd Clears / Recovery] Running %d Monte Carlo trials...\n', n_trials);

for trial_id = 1:n_trials
    scenario_builder_fn = @(cfg, trial_seed) build_scenario_02(cfg, trial_seed);
    metrics_cell{trial_id} = DynamicScenarioRunner.runTrial(2, trial_id, scenario_builder_fn);
end

summary = MetricsEvaluator.summarizeScenario('Herd Clears Recovery', metrics_cell);
end

function [world, custom_updater] = build_scenario_02(cfg, trial_seed)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);

n_goats = 40;
world.n_agents = n_goats;

rng(trial_seed);
herd_x_center = 50.0 + (rand() - 0.5) * 4.0;
vy_fast = 0.50 + 0.10 * (rand() - 0.5); % Fast crossing speed to clear by t = 10s

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
