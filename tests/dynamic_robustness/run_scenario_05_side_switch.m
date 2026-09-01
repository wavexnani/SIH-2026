function [summary, metrics_cell] = run_scenario_05_side_switch(n_trials)
if nargin < 1, n_trials = 50; end
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common');

metrics_cell = cell(n_trials, 1);
fprintf('[Scenario 05: Side Switch Topology Stress] Running %d Monte Carlo trials...\n', n_trials);

for trial_id = 1:n_trials
    scenario_builder_fn = @(cfg, trial_seed) build_scenario_05(cfg, trial_seed);
    metrics_cell{trial_id} = DynamicScenarioRunner.runTrial(5, trial_id, scenario_builder_fn);
end

summary = MetricsEvaluator.summarizeScenario('Side Switch Topology Stress', metrics_cell);
end

function [world, custom_updater] = build_scenario_05(cfg, trial_seed)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);
world.n_agents = 2;

rng(trial_seed);
x_obs = 65.0 + (rand() - 0.5) * 4.0;

world = world.setAgentState(1, x_obs, 1.0, 0.0, 0.0, cfg.sigma_agent);
world.agents(1).length = 4.0; world.agents(1).width = 2.0;

world = world.setAgentState(2, x_obs, -3.0, 0.0, 0.0, cfg.sigma_agent);
world.agents(2).length = 4.0; world.agents(2).width = 2.0;

custom_updater = @(w, t, k, dt) update_side_switch(w, t);
end

function w = update_side_switch(w, t)
if t > 5.0
    w.agents(1).y = min(4.5, 1.0 + 0.5 * (t - 5.0));
    w.agents(2).y = max(-3.0, -0.5 - 0.5 * (t - 5.0));
end
end
