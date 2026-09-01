function [summary, metrics_cell] = run_scenario_06_crossing_agent(n_trials)
if nargin < 1, n_trials = 50; end
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common');

metrics_cell = cell(n_trials, 1);
fprintf('[Scenario 06: Crossing Dynamic Agent] Running %d Monte Carlo trials...\n', n_trials);

for trial_id = 1:n_trials
    scenario_builder_fn = @(cfg, trial_seed) build_scenario_06(cfg, trial_seed);
    metrics_cell{trial_id} = DynamicScenarioRunner.runTrial(6, trial_id, scenario_builder_fn);
end

summary = MetricsEvaluator.summarizeScenario('Crossing Dynamic Agent', metrics_cell);
end

function [world, custom_updater] = build_scenario_06(cfg, trial_seed)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);
world.n_agents = 1;

rng(trial_seed);
x_cross = 45.0 + (rand() - 0.5) * 5.0;
vy_cross = 1.2 + 0.3 * (rand() - 0.5);

world = world.setAgentState(1, x_cross, -2.0, 0.0, vy_cross, cfg.sigma_agent);
world.agents(1).length = 1.0; world.agents(1).width = 0.8;

custom_updater = [];
end
