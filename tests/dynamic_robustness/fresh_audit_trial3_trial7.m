function fresh_audit_trial3_trial7()
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common', 'tests/dynamic_robustness');

fprintf('\n========================================================================================\n');
fprintf('         STEP 1: FRESH DETERMINISTIC FOOTPRINT AUDIT OF TRIALS 3 & 7                   \n');
fprintf('========================================================================================\n');

audit_single_trial(3, 2045);
audit_single_trial(7, 2049);

end

function audit_single_trial(trial_id, trial_seed)
cfg = SimulationConfig();
rng(trial_seed);

dt = cfg.dt;
N_steps = 250;

map_obj = FreeSpaceMap('unstructured');
map_obj.y_min_base = 0.0;
map_obj.y_max_base = 6.0;

bp = FreeSpaceBoundProvider(map_obj, cfg.vehicle_width / 2.0);
planner = CACRCPlanner(cfg);
planner.bound_provider = bp;
sf = SafetyFilter(cfg);
vehicle = BicycleModel(cfg);
obs_model = ObservationModel('ideal', trial_seed);

world = ScenarioDefinitions('indian_realistic_demo_v5', cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);

n_goats = 40;
goat_first = 4;
goat_last = goat_first + n_goats - 1;
while length(world.agents) < goat_last
    world.agents(length(world.agents) + 1) = Agent(length(world.agents) + 1, -100, -100, 0, 0);
end
world.n_agents = goat_last;

herd_x_center = 50.0 + (rand() - 0.5) * 4.0;
vy_fast = 0.50 + 0.10 * (rand() - 0.5);

for gi = 1:n_goats
    ai = goat_first + gi - 1;
    gx = herd_x_center + (rand() - 0.5) * 4.0;
    gy = -1.0 + 0.2 * mod(gi-1, 8);
    gvy = vy_fast;
    gvx = 0.0;
    world = world.setAgentState(ai, gx, gy, gvx, gvy, max(0.30, cfg.sigma_agent));
    world.agents(ai).length = 0.8; world.agents(ai).width = 0.5;
end

ref_path = zeros(900, 5);
ref_path(:,1) = linspace(0, 200, 900)';
ref_path(:,2) = 2.5;
ref_path(:,5) = 5.0;

min_clr_goats = inf; t_min_goats = 0; step_min_goats = 0;
min_clr_cattle = inf; t_min_cattle = 0; step_min_cattle = 0;
min_clr_all = inf; t_min_all = 0; step_min_all = 0;

ego_min_goats_state = [];
ego_min_cattle_state = [];
agent_cattle_state = [];
goat_closest_state = [];

for k = 1:N_steps
    t = (k-1)*dt;
    
    [obs_world, ~] = obs_model.observe(world, dt);
    [u_mpc, pred_states, status, info] = planner.plan(obs_world, ref_path, 5.0);
    [u_cmd, filter_active, filter_reason] = sf.filter(u_mpc, status, world, pred_states, bp);
    
    % Compute clearances independently
    [c_goats, closest_goat] = compute_agent_group_clearance(world.ego, world.agents(goat_first:goat_last), cfg);
    [c_cattle, closest_cattle] = compute_agent_group_clearance(world.ego, world.agents(3), cfg);
    [c_all, closest_all] = compute_agent_group_clearance(world.ego, world.agents(1:world.n_agents), cfg);
    
    if c_goats < min_clr_goats
        min_clr_goats = c_goats;
        t_min_goats = t;
        step_min_goats = k;
        ego_min_goats_state = world.ego;
        goat_closest_state = closest_goat;
    end
    
    if c_cattle < min_clr_cattle
        min_clr_cattle = c_cattle;
        t_min_cattle = t;
        step_min_cattle = k;
        ego_min_cattle_state = world.ego;
        agent_cattle_state = closest_cattle;
    end
    
    if c_all < min_clr_all
        min_clr_all = c_all;
        t_min_all = t;
        step_min_all = k;
    end
    
    world.ego = vehicle.stepKinematic(world.ego, u_cmd(2), u_cmd(1), dt);
    for ai = goat_first:goat_last
        world.agents(ai).x = world.agents(ai).x + world.agents(ai).vx * dt;
        world.agents(ai).y = world.agents(ai).y + world.agents(ai).vy * dt;
    end
end

fprintf('\n--- AUDIT RESULTS FOR TRIAL %d (Seed %d) ---\n', trial_id, trial_seed);
fprintf('1. GOAT HERD ONLY CLEARANCE:\n');
fprintf('   Min Clearance: %6.2f m | Occurred at Step %d (t = %.2f s)\n', min_clr_goats, step_min_goats, t_min_goats);
fprintf('   Ego State: X = %.2fm, Y = %.2fm, V = %.2fm/s\n', ego_min_goats_state.x, ego_min_goats_state.y, ego_min_goats_state.v);
fprintf('   Closest Goat State: ID=%d, X=%.2fm, Y=%.2fm\n', goat_closest_state.id, goat_closest_state.x, goat_closest_state.y);

fprintf('2. CATTLE B ONLY CLEARANCE:\n');
fprintf('   Min Clearance: %6.2f m | Occurred at Step %d (t = %.2f s)\n', min_clr_cattle, step_min_cattle, t_min_cattle);
fprintf('   Ego State: X = %.2fm, Y = %.2fm, V = %.2fm/s\n', ego_min_cattle_state.x, ego_min_cattle_state.y, ego_min_cattle_state.v);
fprintf('   Cattle B State: ID=%d, X=%.2fm, Y=%.2fm\n', agent_cattle_state.id, agent_cattle_state.x, agent_cattle_state.y);

fprintf('3. ALL AGENTS (METRIC) CLEARANCE:\n');
fprintf('   Min Clearance: %6.2f m | Occurred at Step %d (t = %.2f s)\n', min_clr_all, step_min_all, t_min_all);

% Exact Footprint Separation at worst Cattle B timestep
ego_L = cfg.vehicle_length; ego_W = cfg.vehicle_width;
cat_L = 1.0; cat_W = 1.0;
if isprop(agent_cattle_state, 'length') && agent_cattle_state.length > 0, cat_L = agent_cattle_state.length; end
if isprop(agent_cattle_state, 'width') && agent_cattle_state.width > 0, cat_W = agent_cattle_state.width; end

dx_sep = abs(ego_min_cattle_state.x - agent_cattle_state.x) - (ego_L + cat_L)/2.0;
dy_sep = abs(ego_min_cattle_state.y - agent_cattle_state.y) - (ego_W + cat_W)/2.0;

fprintf('4. DETAILED SEPARATION BREAKDOWN AT WORST CLEARANCE TIMESTEP (t = %.2fs):\n', t_min_cattle);
fprintf('   Ego Footprint: X ∈ [%.2f, %.2f], Y ∈ [%.2f, %.2f]\n', ...
    ego_min_cattle_state.x - ego_L/2, ego_min_cattle_state.x + ego_L/2, ...
    ego_min_cattle_state.y - ego_W/2, ego_min_cattle_state.y + ego_W/2);
fprintf('   Cattle B Footprint: X ∈ [%.2f, %.2f], Y ∈ [%.2f, %.2f]\n', ...
    agent_cattle_state.x - cat_L/2, agent_cattle_state.x + cat_L/2, ...
    agent_cattle_state.y - cat_W/2, agent_cattle_state.y + cat_W/2);
fprintf('   Longitudinal Separation dx_sep: %.2f m\n', dx_sep);
fprintf('   Lateral Separation dy_sep: %.2f m\n', dy_sep);
fprintf('   Footprint Clearance: %.2f m\n', min_clr_cattle);

end

function [c_min, closest_ag] = compute_agent_group_clearance(ego, agents, cfg)
c_min = inf;
closest_ag = struct('id', 0, 'x', 0, 'y', 0);
ego_L = cfg.vehicle_length; ego_W = cfg.vehicle_width;

for ai = 1:length(agents)
    ag = agents(ai);
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
    
    if dist < c_min
        c_min = dist;
        closest_ag = ag;
    end
end
end
