function [summary, metrics_cell] = run_scenario_03_partial_gap(n_trials)
if nargin < 1, n_trials = 50; end
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common');

metrics_cell = cell(n_trials, 1);
fprintf('[Scenario 03: Partial Gap Transition] Running %d Monte Carlo trials...\n', n_trials);

for trial_id = 1:n_trials
    scenario_builder_fn = @(cfg, trial_seed) build_scenario_03(cfg, trial_seed);
    metrics_cell{trial_id} = DynamicScenarioRunner.runTrial(3, trial_id, scenario_builder_fn);
end

summary = MetricsEvaluator.summarizeScenario('Partial Gap Transition', metrics_cell);
end

function [world, custom_updater] = build_scenario_03(cfg, trial_seed)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);
world.n_agents = 2;

rng(trial_seed);
x_obs = 55.0 + (rand() - 0.5) * 4.0;
y_mid = 2.5 + (rand() - 0.5) * 0.4;

% Initially gap = 1.0m (tight, < W_req = 1.60m)
world = world.setAgentState(1, x_obs, y_mid + 1.2, 0.0, 0.0, cfg.sigma_agent);
world.agents(1).length = 4.0; world.agents(1).width = 1.4;

world = world.setAgentState(2, x_obs, y_mid - 1.2, 0.0, 0.0, cfg.sigma_agent);
world.agents(2).length = 4.0; world.agents(2).width = 1.4;

custom_updater = @(w, t, k, dt) update_partial_gap(w, t, y_mid);
end

function w = update_partial_gap(w, t, y_mid)
if t > 8.0
    gap = min(2.2, 1.0 + 0.15 * (t - 8.0));
    w.agents(1).y = y_mid + (gap / 2.0) + 0.7;
    w.agents(2).y = y_mid - (gap / 2.0) - 0.7;
end
end
