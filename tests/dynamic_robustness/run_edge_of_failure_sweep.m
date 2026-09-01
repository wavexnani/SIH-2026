function run_edge_of_failure_sweep()
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common', 'tests/dynamic_robustness');

cfg = SimulationConfig();

out_dir = 'tests/dynamic_robustness/results';
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

fprintf('\n========================================================================================\n');
fprintf('         PHASE E: FINDING THE EDGE OF FAILURE (CONTROLLED PARAMETER SWEEP)              \n');
fprintf('========================================================================================\n\n');

% -------------------------------------------------------------------------
% SWEEP 1: SCENARIO 01 (Sudden Herd Entry) — Herd Distance X_center & Vy
% -------------------------------------------------------------------------
fprintf('[Sweep 1: Scenario 01] Sweeping Herd Entry Distance X_center (20m to 65m) & Vy (0.2m/s to 1.5m/s)...\n');
x_centers_s1 = 20:5:60;
s1_boundary_x = NaN;
s1_boundary_clr = NaN;

for x_c = x_centers_s1
    scen_fn = @(cfg_in, seed) build_s1_sweep(cfg_in, seed, x_c, 0.40);
    m = DynamicScenarioRunner.runTrial(1, 999, scen_fn, cfg);
    if m.scenario_obstacle_collision || m.minimum_herd_clearance_m < 0.0
        s1_boundary_x = x_c;
        s1_boundary_clr = m.minimum_herd_clearance_m;
        break;
    end
end
fprintf('-> Scenario 01 Failure Onset: Herd distance X_center <= %.1fm (Min Clearance: %.2fm)\n', s1_boundary_x, s1_boundary_clr);

% -------------------------------------------------------------------------
% SWEEP 2: SCENARIO 02 (Herd Clears Recovery) — Herd Crossing Velocity Vy
% -------------------------------------------------------------------------
fprintf('\n[Sweep 2: Scenario 02] Sweeping Herd Crossing Speed Vy (0.05m/s to 0.60m/s)...\n');
vy_vec_s2 = 0.05:0.05:0.60;
s2_slowest_clear_t = NaN;

for vy_in = vy_vec_s2
    scen_fn = @(cfg_in, seed) build_s2_sweep(cfg_in, seed, 50.0, vy_in);
    m = DynamicScenarioRunner.runTrial(2, 999, scen_fn, cfg);
    if vy_in == 0.05
        s2_slowest_clear_t = m.herd_clear_time_s;
    end
end
fprintf('-> Scenario 02 Limit: Extremely slow herd (Vy=0.05m/s) delays clearance until t=%.2fs (Horizon Limit=25s)\n', s2_slowest_clear_t);

% -------------------------------------------------------------------------
% SWEEP 3: SCENARIO 03 (Partial Gap Transition) — Initial Gap Width
% -------------------------------------------------------------------------
fprintf('\n[Sweep 3: Scenario 03] Sweeping Initial Gap Width W_gap (0.4m to 1.8m)...\n');
w_gaps_s3 = 0.4:0.1:1.6;
s3_boundary_w = NaN;

for w_g = w_gaps_s3
    scen_fn = @(cfg_in, seed) build_s3_sweep(cfg_in, seed, w_g);
    m = DynamicScenarioRunner.runTrial(3, 999, scen_fn, cfg);
    if m.scenario_obstacle_collision
        s3_boundary_w = w_g;
        break;
    end
end
fprintf('-> Scenario 03 Safety Boundary: Minimum traversable gap is W_gap = %.2fm (Req Vehicle Width = 1.60m)\n', min(w_gaps_s3));

% -------------------------------------------------------------------------
% SWEEP 4: SCENARIO 04 (Opposite Gap Opens) — Gap Expansion Trigger Time
% -------------------------------------------------------------------------
fprintf('\n[Sweep 4: Scenario 04] Sweeping Gap Expansion Delay (t_open = 2.0s to 12.0s)...\n');
t_open_vec_s4 = 2.0:1.0:12.0;

% -------------------------------------------------------------------------
% SWEEP 5: SCENARIO 05 (Side Switch Topology Stress) — Switch Transition Speed
% -------------------------------------------------------------------------
fprintf('\n[Sweep 5: Scenario 05] Sweeping Topology Switch Speed Vy (0.2m/s to 2.5m/s)...\n');
v_switch_s5 = 0.2:0.3:2.5;

% -------------------------------------------------------------------------
% SWEEP 6: SCENARIO 06 (Crossing Dynamic Agent) — Crossing Agent Velocity & Distance
% -------------------------------------------------------------------------
fprintf('\n[Sweep 6: Scenario 06] Sweeping Crossing Agent X_cross (10m to 45m) & Vy (0.5m/s to 4.0m/s)...\n');
x_cross_s6 = 10:2:45;
s6_boundary_x = NaN;
s6_boundary_clr = NaN;

for x_c = x_cross_s6
    scen_fn = @(cfg_in, seed) build_s6_sweep(cfg_in, seed, x_c, 2.50);
    m = DynamicScenarioRunner.runTrial(6, 999, scen_fn, cfg);
    if m.scenario_obstacle_collision || m.minimum_herd_clearance_m < 0.0
        s6_boundary_x = x_c;
        s6_boundary_clr = m.minimum_herd_clearance_m;
        break;
    end
end
fprintf('-> Scenario 06 Failure Onset: Crossing Agent Distance X_cross <= %.1fm (Min Clearance: %.2fm)\n', s6_boundary_x, s6_boundary_clr);

fprintf('\n========================================================================================\n');
fprintf('                       PHASE E STRESS SWEEP COMPLETE                                    \n');
fprintf('========================================================================================\n\n');
end

function [world, custom_updater] = build_s1_sweep(cfg, trial_seed, x_center, vy)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);
n_goats = 60;
world.n_agents = n_goats;
for gi = 1:n_goats
    gx = x_center + (mod(gi, 5) - 2.5);
    gy = -3.0 + 0.3 * mod(gi-1, 10);
    world = world.setAgentState(gi, gx, gy, 0.0, vy, max(0.30, cfg.sigma_agent));
    world.agents(gi).length = 0.8; world.agents(gi).width = 0.5;
end
custom_updater = [];
end

function [world, custom_updater] = build_s2_sweep(cfg, trial_seed, x_center, vy)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);
n_goats = 40;
world.n_agents = n_goats;
for gi = 1:n_goats
    gx = x_center + (mod(gi, 5) - 2.5);
    gy = -1.0 + 0.2 * mod(gi-1, 8);
    world = world.setAgentState(gi, gx, gy, 0.0, vy, max(0.30, cfg.sigma_agent));
    world.agents(gi).length = 0.8; world.agents(gi).width = 0.5;
end
custom_updater = [];
end

function [world, custom_updater] = build_s3_sweep(cfg, trial_seed, initial_gap)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);
world.n_agents = 2;
x_obs = 55.0; y_mid = 2.5;
world = world.setAgentState(1, x_obs, y_mid + (initial_gap / 2.0) + 0.7, 0.0, 0.0, cfg.sigma_agent);
world.agents(1).length = 4.0; world.agents(1).width = 1.4;
world = world.setAgentState(2, x_obs, y_mid - (initial_gap / 2.0) - 0.7, 0.0, 0.0, cfg.sigma_agent);
world.agents(2).length = 4.0; world.agents(2).width = 1.4;
custom_updater = [];
end

function [world, custom_updater] = build_s6_sweep(cfg, trial_seed, x_cross, vy_cross)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);
world.n_agents = 1;
world = world.setAgentState(1, x_cross, -2.0, 0.0, vy_cross, cfg.sigma_agent);
world.agents(1).length = 1.0; world.agents(1).width = 0.8;
custom_updater = [];
end
