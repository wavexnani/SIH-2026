function [summary, metrics_cell] = run_scenario_04_opposite_gap(n_trials)
if nargin < 1, n_trials = 50; end
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common');

metrics_cell = cell(n_trials, 1);
fprintf('[Scenario 04: Opposite Gap Opens] Running %d Monte Carlo trials...\n', n_trials);

for trial_id = 1:n_trials
    scenario_builder_fn = @(cfg, trial_seed) build_scenario_04(cfg, trial_seed);
    metrics_cell{trial_id} = DynamicScenarioRunner.runTrial(4, trial_id, scenario_builder_fn);
end

summary = MetricsEvaluator.summarizeScenario('Opposite Gap Opens', metrics_cell);
end

function [world, custom_updater] = build_scenario_04(cfg, trial_seed)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);
world.n_agents = 2;

rng(trial_seed);
x_obs = 60.0 + (rand() - 0.5) * 4.0;

world = world.setAgentState(1, x_obs, 4.0, 0.0, 0.0, cfg.sigma_agent);
world.agents(1).length = 4.0; world.agents(1).width = 2.0;

world = world.setAgentState(2, x_obs, 1.0, 0.0, 0.0, cfg.sigma_agent);
world.agents(2).length = 4.0; world.agents(2).width = 2.0;

custom_updater = @(w, t, k, dt) update_opposite_gap(w, t);
end

function w = update_opposite_gap(w, t)
if t > 6.0
    w.agents(2).y = max(-2.0, 1.0 - 0.4 * (t - 6.0));
end
end
