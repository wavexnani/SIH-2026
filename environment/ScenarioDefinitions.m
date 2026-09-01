function world = ScenarioDefinitions(scenario_name, cfg)
    % SCENARIODEFINITIONS Create a predefined scenario
    %
    % Stage 0: Define agents and static obstacles
    %
    % Usage:
    %   world = ScenarioDefinitions('simple_oncoming', cfg);
    %
    % Scenarios:
    %   'simple_oncoming':    One agent approaching head-on
    %   'overtaking':         Agent overtaking from left
    %   'moderate':           2 agents + 3 obstacles (default)
    %   'complex':            Multiple agents and obstacles
    
    % Initialize world
    world = WorldState(cfg);
    
    % Switch on scenario name
    switch scenario_name
        case {'static', 'static_obs', 'stage2'}
            world = scenario_static(world, cfg);
            
        case {'clear', 'stage1'}
            world = scenario_clear(world, cfg);
            
        case 'simple_oncoming'
            world = scenario_simple_oncoming(world, cfg);
            
        case 'overtaking'
            world = scenario_overtaking(world, cfg);
            
        case 'moderate'
            world = scenario_moderate(world, cfg);
            
        case 'passable_moderate'
            world = scenario_passable_moderate(world, cfg);
            
        case 'passable_marginal'
            world = scenario_passable_marginal(world, cfg);
            
        case 'impassable_center'
            world = scenario_impassable_center(world, cfg);
            
        case 'multi_obstacle_sequence'
            world = scenario_multi_obstacle_sequence(world, cfg);
            
        case 'multi_vehicle_following'
            world = scenario_multi_vehicle_following(world, cfg);
            
        case 'multi_vehicle_yield_overtake'
            world = scenario_multi_vehicle_yield_overtake(world, cfg);
            
        case 'multi_vehicle_oncoming_conflict'
            world = scenario_multi_vehicle_oncoming_conflict(world, cfg);
            
        case 'unstructured_road'
            world = scenario_unstructured_road(world, cfg);
            
        case {'indian_unstructured', 'unstructured_indian'}
            world = scenario_indian_unstructured(world, cfg);
            
        case {'indian_unstructured_bottleneck', 'unstructured_bottleneck'}
            world = scenario_indian_unstructured_bottleneck(world, cfg);
            
        case {'indian_unstructured_no_obs', 'unstructured_no_obs'}
            world = scenario_indian_unstructured_no_obs(world, cfg);
            
        case {'indian_realistic_demo', 'realistic_demo', 'scenario_indian_realistic_demo'}
            world = scenario_indian_realistic_demo(world, cfg);
            
        case {'indian_realistic_demo_v2', 'realistic_demo_v2', 'scenario_indian_realistic_demo_v2'}
            world = scenario_indian_realistic_demo_v2(world, cfg);
            
        case {'indian_realistic_demo_v3', 'realistic_demo_v3', 'scenario_indian_realistic_demo_v3'}
            world = scenario_indian_realistic_demo_v3(world, cfg);
            
        case {'indian_realistic_demo_v4', 'realistic_demo_v4', 'scenario_indian_realistic_demo_v4'}
            world = scenario_indian_realistic_demo_v4(world, cfg);
            
        case {'indian_realistic_demo_v5', 'realistic_demo_v5', 'scenario_indian_realistic_demo_v5'}
            world = scenario_indian_realistic_demo_v5(world, cfg);
            
        case 'cattle_crossing'
            world = scenario_cattle_crossing(world, cfg);
            
        case 'complex'
            world = scenario_complex(world, cfg);
            
        otherwise
            warning('Unknown scenario: %s. Using default (moderate)', scenario_name);
            world = scenario_moderate(world, cfg);
    end
end

% =========================================================================
% SCENARIO 1: Simple Oncoming
% =========================================================================
function world = scenario_simple_oncoming(world, cfg)
    % One agent approaching head-on
    %
    % Setup:
    %   Ego:      (10 m, 3 m) moving forward at 10 m/s
    %   Agent 1:  (80 m, 3 m) moving backward at -10 m/s
    %   Obstacle: (50 m, 1 m) static
    
    % Ego
    world = world.setEgoState(10, 3, 0, 0);
    
    % Agent 1: Head-on approaching
    world = world.setAgentState(1, 80, 3, -10, 0, cfg.sigma_agent);
    
    % Dummy agents (not used)
    world = world.setAgentState(2, -100, -100, 0, 0, cfg.sigma_agent);
    world = world.setAgentState(3, -100, -100, 0, 0, cfg.sigma_agent);
    
    % Static obstacle at midpoint
    world = world.setStaticObstacle(1, 50, 1, 3, 2);
    
    % Dummy obstacles
    world = world.setStaticObstacle(2, -100, -100, 2, 2);
    world = world.setStaticObstacle(3, -100, -100, 2, 2);
    world = world.setStaticObstacle(4, -100, -100, 2, 2);
end

% =========================================================================
% SCENARIO 2: Overtaking
% =========================================================================
function world = scenario_overtaking(world, cfg)
    % Agent overtaking from left side
    %
    % Setup:
    %   Ego:      (10 m, 3 m) at 8 m/s, need to avoid overtaking agent
    %   Agent 1:  (40 m, 4.5 m) moving right at +8 m/s (in left lane)
    %   Obstacle: (70 m, 2.5 m) static
    
    % Ego
    world = world.setEgoState(10, 3, 0, 0);
    
    % Agent 1: Overtaking from left
    world = world.setAgentState(1, 40, 4.5, 8, 0.5, cfg.sigma_agent);
    
    % Dummy agents
    world = world.setAgentState(2, -100, -100, 0, 0, cfg.sigma_agent);
    world = world.setAgentState(3, -100, -100, 0, 0, cfg.sigma_agent);
    
    % Static obstacle
    world = world.setStaticObstacle(1, 70, 1.5, 3, 2);
    
    % Dummy obstacles
    world = world.setStaticObstacle(2, -100, -100, 2, 2);
    world = world.setStaticObstacle(3, -100, -100, 2, 2);
    world = world.setStaticObstacle(4, -100, -100, 2, 2);
end

% =========================================================================
% SCENARIO 3: Moderate (Default)
% =========================================================================
function world = scenario_moderate(world, cfg)
    % Moderate complexity: 2-3 agents, 3 static obstacles
    %
    % Setup:
    %   Ego:       (10 m, 3 m) starting from rest
    %   Agent 1:   (50 m, 2 m) moving forward slowly
    %   Agent 2:   (60 m, 4 m) moving forward at medium speed
    %   Obstacles: At (30, 2), (70, 1.5), (85, 4)
    
    % Ego vehicle
    world = world.setEgoState(10, 3, 0, 0);
    
    % Agent 1: Slow vehicle ahead
    world = world.setAgentState(1, 50, 2, 5, 0, cfg.sigma_agent);
    
    % Agent 2: Faster vehicle in adjacent lane
    world = world.setAgentState(2, 60, 4.2, 8, 0, cfg.sigma_agent);
    
    % Agent 3: Unused
    world = world.setAgentState(3, -100, -100, 0, 0, cfg.sigma_agent);
    
    % Static obstacle 1
    world = world.setStaticObstacle(1, 30, 1.5, 3, 2);
    
    % Static obstacle 2
    world = world.setStaticObstacle(2, 70, 4, 3, 2);
    
    % Static obstacle 3
    world = world.setStaticObstacle(3, 85, 2.5, 2, 2);
    
    % Dummy obstacle
    world = world.setStaticObstacle(4, -100, -100, 2, 2);
end

% =========================================================================
% SCENARIO 4: Complex
% =========================================================================
function world = scenario_complex(world, cfg)
    % Complex: Multiple agents with various velocities
    %
    % Setup:
    %   Ego:       (10 m, 3 m)
    %   Agent 1:   (45 m, 2.5 m) slower moving vehicle
    %   Agent 2:   (65 m, 3.5 m) medium speed, changing position
    %   Agent 3:   (75 m, 1.5 m) faster vehicle
    %   Obstacles: Strategic placement at (35, 1.5), (60, 4), (80, 2)
    
    % Ego vehicle
    world = world.setEgoState(10, 3, 0, 0);
    
    % Agent 1
    world = world.setAgentState(1, 45, 2.5, 6, 0, cfg.sigma_agent);
    
    % Agent 2
    world = world.setAgentState(2, 65, 3.5, 10, 0, cfg.sigma_agent);
    
    % Agent 3
    world = world.setAgentState(3, 75, 1.5, 12, 0, cfg.sigma_agent);
    
    % Static obstacle 1
    world = world.setStaticObstacle(1, 35, 1.5, 3, 2);
    
    % Static obstacle 2
    world = world.setStaticObstacle(2, 60, 4.5, 2.5, 2);
    
    % Static obstacle 3
    world = world.setStaticObstacle(3, 80, 2, 3, 2);
    
    % Static obstacle 4
    world = world.setStaticObstacle(4, 50, 0.7, 2, 1.5);
end

% =========================================================================
% SCENARIO: Static Obstacles (Stage 2 Validation)
% =========================================================================
function world = scenario_static(world, cfg)
    % Static obstacle avoidance validation: 3 static obstacles
    world = world.setEgoState(10, 3, 0, 0);
    
    % Dummy agents (far off-road)
    for i = 1:cfg.n_agents
        world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent);
    end
    
    % Static obstacle 1: (30, 1.5)
    world = world.setStaticObstacle(1, 30, 1.5, 3, 2);
    
    % Static obstacle 2: (70, 4.0)
    world = world.setStaticObstacle(2, 70, 4.0, 3, 2);
    
    % Static obstacle 3: (85, 2.5)
    world = world.setStaticObstacle(3, 85, 2.5, 2, 2);
    
    % Dummy obstacle
    world = world.setStaticObstacle(4, -100, -100, 2, 2);
end

% =========================================================================
% SCENARIO 0: Clear (Controller-Only Validation)
% =========================================================================
function world = scenario_clear(world, cfg)
    % Clear track for controller validation without obstacles
    world = world.setEgoState(10, 3, 0, 0);
    
    % Dummy agents (far off-road)
    for i = 1:cfg.n_agents
        world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent);
    end
    
    % Dummy obstacles (far off-road)
    for j = 1:cfg.n_static_obs
        world = world.setStaticObstacle(j, -100, -100, 2, 2);
    end
end

% =========================================================================
% SCENARIO 4: Passable Moderate (Geometrically Feasible Corridor)
% =========================================================================
function world = scenario_passable_moderate(world, cfg)
    % Ego vehicle
    world = world.setEgoState(10, 3, 0, 0);
    
    % Unused agents
    world = world.setAgentState(1, -100, -100, 0, 0, cfg.sigma_agent);
    world = world.setAgentState(2, -100, -100, 0, 0, cfg.sigma_agent);
    world = world.setAgentState(3, -100, -100, 0, 0, cfg.sigma_agent);
    
    % Static obstacle 1: x = 30.0, y = 1.80m, radius = 0.50m (width=1.0m, height=1.0m)
    % d_safe = 0.90 + 0.50 + 2(0.09) + 0.10 = 1.68m
    % Left corridor: y >= 1.80 + 1.68 = 3.48m <= 4.90m (Passable!)
    world = world.setStaticObstacle(1, 30, 1.8, 1.0, 1.0);
    world = world.setStaticObstacle(2, -100, -100, 1.0, 1.0);
    world = world.setStaticObstacle(3, -100, -100, 1.0, 1.0);
    world = world.setStaticObstacle(4, -100, -100, 1.0, 1.0);
end

% =========================================================================
% SCENARIO: Marginally Passable Corridor
% =========================================================================
function world = scenario_passable_marginal(world, cfg)
    world = world.setEgoState(10, 3, 0, 0);
    for i = 1:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
    
    % Static obstacle 1: x = 30.0, y = 2.10m, width = 1.0m, height = 1.0m (radius = 0.50m)
    % Requires left pass corridor y >= 2.10 + 1.68 = 3.78m <= 4.65m (Tight/marginal!)
    world = world.setStaticObstacle(1, 30, 2.10, 1.0, 1.0);
    for j = 2:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
end

% =========================================================================
% SCENARIO: Geometrically Impossible Center Obstacle
% =========================================================================
function world = scenario_impassable_center(world, cfg)
    world = world.setEgoState(10, 3, 0, 0);
    for i = 1:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
    
    % Static obstacle 1: x = 30.0, y = 3.00m, width = 1.80m, height = 1.80m (radius = 0.90m)
    % Left pass requires y >= 3.00 + 2.08 = 5.08m > 4.65m (Violation!)
    % Right pass requires y <= 3.00 - 2.08 = 0.92m < 1.35m (Violation!)
    world = world.setStaticObstacle(1, 30, 3.00, 1.80, 1.80);
    for j = 2:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
end

% =========================================================================
% SCENARIO: Multi-Obstacle Sequence (Left Overtake -> Right Overtake)
% =========================================================================
function world = scenario_multi_obstacle_sequence(world, cfg)
    world = world.setEgoState(10, 3, 0, 0);
    for i = 1:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
    
    % Obstacle 1: x = 30.0m, y = 1.80m (Left overtake required)
    world = world.setStaticObstacle(1, 30, 1.80, 1.0, 1.0);
    % Obstacle 2: x = 65.0m, y = 4.20m (Right overtake required)
    world = world.setStaticObstacle(2, 65, 4.20, 1.0, 1.0);
    
    for j = 3:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
end

% =========================================================================
% STAGE 5 MULTI-VEHICLE SCENARIOS
% =========================================================================

function world = scenario_multi_vehicle_following(world, cfg)
    % Slower lead vehicle ahead moving initially at 2.5 m/s in same lane (y = 1.80m)
    % Phase 12B: Dynamic closing-rate scenario with dynamic agent IDM car-following
    world = world.setEgoState(10, 1.80, 0, 0);
    dyn_agent = DynamicAgent(1, 35.0, 1.80, 2.5, 0.0, 5.0, 1.80);
    world.dynamic_agents = dyn_agent;
    world = world.setAgentState(1, dyn_agent.x, dyn_agent.y, dyn_agent.v * cos(dyn_agent.theta), dyn_agent.v * sin(dyn_agent.theta), cfg.sigma_agent);
    
    for i = 2:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
    for j = 1:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
end

function world = scenario_multi_vehicle_yield_overtake(world, cfg)
    % Agent 1: Slower lead vehicle at x = 35m, y = 1.80m moving at 3.5 m/s
    % Agent 2: Oncoming vehicle in left lane (y = 4.20m) moving backward at -7.0 m/s
    % Ego yields behind Agent 1 until Agent 2 passes, then overtakes
    world = world.setEgoState(10, 1.80, 0, 0);
    world = world.setAgentState(1, 35, 1.80, 3.5, 0, cfg.sigma_agent);
    world = world.setAgentState(2, 70, 4.20, -7.0, 0, cfg.sigma_agent);
    for i = 3:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
    for j = 1:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
end

function world = scenario_multi_vehicle_oncoming_conflict(world, cfg)
    % Static obstacle at x = 35.0m, y = 1.80m (Requires left overtake corridor)
    % Agent 1: Oncoming vehicle in left overtake corridor (x = 60m, y = 4.20m)
    %   moving at -8.0 m/s. Ego yields (YIELD) until Agent 1 clears, then
    %   overtakes the static obstacle via the left corridor.
    world = world.setEgoState(10, 1.80, 0, 0);
    world = world.setAgentState(1, 60, 4.20, -8.0, 0, cfg.sigma_agent);
    for i = 2:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
    
    world = world.setStaticObstacle(1, 35, 1.80, 1.0, 1.0);
    for j = 2:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
end

function world = scenario_unstructured_road(world, cfg)
    % Unstructured road with irregular boundaries and static obstacle
    world = world.setEgoState(10, 3.0, 0, 5.0);
    world = world.setStaticObstacle(1, 45.0, 1.50, 1.80, 1.80);
    for j = 2:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
    for i = 1:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
end

function world = scenario_cattle_crossing(world, cfg)
    % Cattle crossing scenario: Static obstacle + slow crossing dynamic agent
    world = world.setEgoState(10, 3.0, 0, 5.0);
    world = world.setStaticObstacle(1, 60.0, 1.80, 1.80, 1.80);
    world = world.setAgentState(1, 50.0, 1.0, 0.4, 0.8, cfg.sigma_agent);
    for i = 2:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
end

function world = scenario_indian_unstructured(world, cfg)
    % Representative Indian Unstructured Road Scenario (Hypothesis Validation):
    % - No formal lane markings
    % - Irregular road boundaries
    % - Obstacle 1 (Static right lane parked vehicle): x = 38.0m, y = 1.00m
    % - Obstacle 2 (Static left shoulder encroaching stall): x = 42.0m, y = 4.50m
    % - Dynamic Cattle Agent (Agent 1): x = 65.0m, y = 3.20m moving across road
    % - Ego vehicle starting at x = 10.0m, y = 2.50m
    
    world = world.setEgoState(10, 2.50, 0, 5.0);
    
    % Static Obstacle 1: Parked vehicle / autorickshaw in right lane
    world = world.setStaticObstacle(1, 38.0, 1.00, 2.20, 1.50);
    
    % Static Obstacle 2: Encroaching roadside stall / debris in left shoulder
    world = world.setStaticObstacle(2, 42.0, 4.50, 4.50, 1.50);
    
    for j = 3:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
    
    % Dynamic Cattle / Irregular Agent (Agent 1)
    % Cattle moving slowly across road with non-lane heading & expanding uncertainty
    world = world.setAgentState(1, 65.0, 3.20, 0.40, -0.10, max(0.30, cfg.sigma_agent));
    for i = 2:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
end

function world = scenario_indian_unstructured_bottleneck(world, cfg)
    % Isolated Bottleneck-Only Experiment (Experiment A Confound Isolation):
    % - Identical static obstacle & road geometry as indian_unstructured
    % - NO distant cattle agent (isolates free-space hypothesis test from cattle coordination)
    
    world = world.setEgoState(10, 2.50, 0, 5.0);
    
    % Static Obstacle 1: Parked vehicle / autorickshaw in right lane (x = 38.0m, y = 1.00m)
    world = world.setStaticObstacle(1, 38.0, 1.00, 2.20, 1.50);
    
    % Static Obstacle 2: Encroaching roadside stall / debris in left shoulder (x = 42.0m, y = 4.50m)
    world = world.setStaticObstacle(2, 42.0, 4.50, 4.50, 1.50);
    
    for j = 3:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
    
    % No dynamic agents in bottleneck-only experiment
    for i = 1:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
end

function world = scenario_indian_unstructured_no_obs(world, cfg)
    % Counterfactual 1 Scenario: Identical Indian Unstructured Road, NO STATIC OBSTACLES
    % Used to verify that Baseline (Fixed-Lane) cruises smoothly past x = 38m when obstacles are absent.
    
    world = world.setEgoState(10, 2.50, 0, 5.0);
    
    % No static obstacles
    for j = 1:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
    
    % No dynamic agents
    for i = 1:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
end

function world = scenario_indian_realistic_demo(world, cfg)
    % PHASE 33: Presentation-Only Dynamic Realistic Indian Road Scenario
    % Demonstrates multi-agent coordination (cattle crossing, oncoming motorcycle,
    % roadside bovine, static autorickshaw, roadside stall) in a realistic 25s timeline.
    
    world = world.setEgoState(10.0, 2.50, 0.0, 5.0);
    
    % Static Obstacle 1: Parked Auto-Rickshaw in right shoulder (x = 38.0m, y = 0.50m)
    world = world.setStaticObstacle(1, 38.0, 0.50, 2.20, 1.30);
    
    % Static Obstacle 2: Roadside Stall in left shoulder (x = 48.0m, y = 5.00m)
    world = world.setStaticObstacle(2, 48.0, 5.00, 4.00, 1.30);
    
    for j = 3:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
    
    % Dynamic Agent 1: Primary Crossing Cattle A (starts x=65m, y=5.2m, slow wander after bottleneck)
    world = world.setAgentState(1, 65.0, 5.20, 0.15, -0.05, max(0.30, cfg.sigma_agent));
    world.agents(1).length = 2.0; world.agents(1).width = 0.8;
    
    % Dynamic Agent 2: Secondary Roadside Cattle B (bovine at x=120m, y=5.2m, grazing)
    world = world.setAgentState(2, 120.0, 5.20, 0.05, -0.02, max(0.30, cfg.sigma_agent));
    world.agents(2).length = 2.0; world.agents(2).width = 0.8;
    
    % Dynamic Agent 3: Oncoming Motorcycle in left lane (x=140m, y=4.8m, v=-4.0m/s)
    world = world.setAgentState(3, 140.0, 4.80, -4.00, 0.00, max(0.30, cfg.sigma_agent));
    world.agents(3).length = 2.0; world.agents(3).width = 0.8;
    
    for i = 4:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
end

function world = scenario_indian_realistic_demo_v2(world, cfg)
    % PHASE 34: Presentation-Only Dynamic Realistic Village Road Scenario v2
    % Features static bottleneck obstacles, 2 dynamic cattle, oncoming motorcycle,
    % herder guide avatar, and a dynamic goat herd road crossing event.
    
    world = world.setEgoState(10.0, 2.50, 0.0, 5.0);
    
    % Static Obstacle 1: Parked Auto-Rickshaw on right shoulder (x = 38.0m, y = 0.00m)
    world = world.setStaticObstacle(1, 38.0, 0.00, 2.20, 1.20);
    
    % Static Obstacle 2: Roadside Stall on left shoulder (x = 48.0m, y = 5.80m)
    world = world.setStaticObstacle(2, 48.0, 5.80, 4.00, 1.20);
    
    for j = 3:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
    
    % Expand agent capacity if needed
    if world.n_agents < 9
        world.n_agents = 9;
        for i = length(world.agents)+1:9
            world.agents(i) = Agent(i, -100, -100, 0, 0);
        end
    end
    
    % Dynamic Agent 1: Primary Crossing Cattle A (starts x=65m, y=5.2m)
    world = world.setAgentState(1, 65.0, 5.20, 0.15, -0.05, max(0.30, cfg.sigma_agent));
    world.agents(1).length = 1.8; world.agents(1).width = 0.8;
    
    % Dynamic Agent 2: Secondary Roadside Cattle B (bovine at x=120m, y=5.2m, grazing)
    world = world.setAgentState(2, 120.0, 5.20, 0.05, -0.02, max(0.30, cfg.sigma_agent));
    world.agents(2).length = 1.8; world.agents(2).width = 0.8;
    
    % Dynamic Agent 3: Oncoming Motorcycle in left lane (x=100m, y=4.8m, v=-4.0m/s)
    world = world.setAgentState(3, 100.0, 4.80, -4.00, 0.00, max(0.30, cfg.sigma_agent));
    world.agents(3).length = 2.0; world.agents(3).width = 0.8;
    
    % Dynamic Goat Herd Crossing Event (Agents 4..8: Goats)
    % Goats start on left shoulder (y=5.0-5.3m, x=82-90m), crossing diagonally to right shoulder
    world = world.setAgentState(4, 82.0, 5.00, 0.20, -0.30, max(0.30, cfg.sigma_agent)); % Goat 1
    world.agents(4).length = 1.0; world.agents(4).width = 0.45;
    
    world = world.setAgentState(5, 84.0, 5.20, 0.18, -0.28, max(0.30, cfg.sigma_agent)); % Goat 2
    world.agents(5).length = 0.9; world.agents(5).width = 0.40;
    
    world = world.setAgentState(6, 86.0, 5.10, 0.22, -0.32, max(0.30, cfg.sigma_agent)); % Goat 3
    world.agents(6).length = 1.1; world.agents(6).width = 0.45;
    
    world = world.setAgentState(7, 88.0, 5.30, 0.15, -0.25, max(0.30, cfg.sigma_agent)); % Goat 4
    world.agents(7).length = 1.0; world.agents(7).width = 0.45;
    
    world = world.setAgentState(8, 90.0, 5.20, 0.25, -0.35, max(0.30, cfg.sigma_agent)); % Goat 5
    world.agents(8).length = 0.95; world.agents(8).width = 0.42;
    
    % Dynamic Agent 9: Herder Guide walking behind herd
    world = world.setAgentState(9, 87.0, 5.40, 0.20, -0.25, max(0.30, cfg.sigma_agent));
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    
    for i = 10:world.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
end

function world = scenario_indian_realistic_demo_v3(world, cfg)
    % PHASE 36: Presentation Dynamic Realistic Village Road Scenario v3
    % Features smooth spatial geometry:
    % - Parked Rickshaw (x=25m, y=-0.50m, right shoulder offset)
    % - Roadside Stall (x=45m, y=6.50m, left shoulder offset)
    % - Cattle A (x=75m, y=6.50m, left shoulder grazing)
    % - Goat Herd Crossing Event (x=105-112m, y=5.5m -> 0.5m)
    % - Herder Avatar (x=112m, y=5.6m)
    % - Cattle B (x=150m, y=6.50m)
    % - Oncoming Motorcycle (x=175m, y=5.20m, v=-4.0m/s)
    
    world = world.setEgoState(10.0, 2.50, 0.0, 5.0);
    
    % Static Obstacle 1: Parked Auto-Rickshaw on right shoulder (x = 25.0m, y = -1.50m)
    world = world.setStaticObstacle(1, 25.0, -1.50, 2.00, 0.80);
    
    % Static Obstacle 2: Roadside Stall on right shoulder (x = 45.0m, y = -1.50m)
    world = world.setStaticObstacle(2, 45.0, -1.50, 3.00, 0.80);
    
    for j = 3:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
    
    % Expand agent capacity if needed
    if world.n_agents < 9
        world.n_agents = 9;
        for i = length(world.agents)+1:9
            world.agents(i) = Agent(i, -100, -100, 0, 0);
        end
    end
    
    % Dynamic Agent 1: Primary Crossing Cattle A (starts x=75m, y=7.5m, grazing on left shoulder)
    world = world.setAgentState(1, 75.0, 7.50, 0.00, 0.00, max(0.30, cfg.sigma_agent));
    world.agents(1).length = 1.8; world.agents(1).width = 0.8;
    
    % Dynamic Agent 2: Secondary Roadside Cattle B (bovine at x=150m, y=7.5m, grazing)
    world = world.setAgentState(2, 150.0, 7.50, 0.00, 0.00, max(0.30, cfg.sigma_agent));
    world.agents(2).length = 1.8; world.agents(2).width = 0.8;
    
    % Dynamic Agent 3: Oncoming Motorcycle in left lane (x=190m, y=5.5m, v=-3.0m/s)
    world = world.setAgentState(3, 190.0, 5.50, -3.00, 0.00, max(0.30, cfg.sigma_agent));
    world.agents(3).length = 2.0; world.agents(3).width = 0.8;
    
    % Dynamic Goat Herd Crossing Event (Agents 4..8: Goats)
    % Goats start on left shoulder (y=5.5m, x=105-112m), crossing dynamically across road (vy=-0.45m/s)
    world = world.setAgentState(4, 105.0, 5.50, 0.20, -0.45, max(0.30, cfg.sigma_agent)); % Goat 1
    world.agents(4).length = 1.0; world.agents(4).width = 0.45;
    
    world = world.setAgentState(5, 107.0, 5.50, 0.18, -0.42, max(0.30, cfg.sigma_agent)); % Goat 2
    world.agents(5).length = 0.9; world.agents(5).width = 0.40;
    
    world = world.setAgentState(6, 109.0, 5.50, 0.22, -0.48, max(0.30, cfg.sigma_agent)); % Goat 3
    world.agents(6).length = 1.1; world.agents(6).width = 0.45;
    
    world = world.setAgentState(7, 111.0, 5.50, 0.15, -0.40, max(0.30, cfg.sigma_agent)); % Goat 4
    world.agents(7).length = 1.0; world.agents(7).width = 0.45;
    
    world = world.setAgentState(8, 112.0, 5.50, 0.25, -0.50, max(0.30, cfg.sigma_agent)); % Goat 5
    world.agents(8).length = 0.95; world.agents(8).width = 0.42;
    
    % Dynamic Agent 9: Herder Guide walking behind herd
    world = world.setAgentState(9, 112.0, 5.60, 0.18, -0.42, max(0.30, cfg.sigma_agent));
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    
    n_total = length(world.agents);
    for i = 10:n_total, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
    world.n_agents = 9;
end

function world = scenario_indian_realistic_demo_v4(world, cfg)
    % PHASE 38: Realistic Dynamic Village Road Scenario v4
    % Features realistic spatial geometry & precise dynamic synchronization:
    % - Parked Rickshaw (x=30m, y=0.50m): encroaches into right lane, forcing left shift
    % - Roadside Stall (x=55m, y=5.20m): encroaches into left lane, forcing right shift
    % - Cattle A (x=80m, y=5.80m): grazing near left shoulder
    % - Goat Herd Crossing Event (x=85-92m, y=5.50m -> 1.00m):
    %   Goats start crossing at vy = -0.18 m/s so they occupy y in [1.8m, 3.2m] at t = 14s..18s
    %   when Ego arrives at x = 85m..92m! Forces cautious deceleration/yielding!
    % - Herder Avatar (x=89m, y=5.75m -> 1.25m, vy = -0.17 m/s)
    % - Oncoming Motorcycle (x=140m, y=4.80m, v=-3.5m/s)
    % - Cattle B (x=150m, y=5.80m)
    
    world = world.setEgoState(10.0, 2.50, 0.0, 5.0);
    
    % Static Obstacle 1: Parked Auto-Rickshaw partially intruding right shoulder edge (x = 28.0m, y = -0.50m)
    world = world.setStaticObstacle(1, 28.0, -0.50, 2.00, 1.00);
    
    % Static Obstacle 2: Roadside Stall on left shoulder edge (x = 55.0m, y = 6.00m)
    world = world.setStaticObstacle(2, 55.0, 6.00, 3.00, 1.00);
    
    for j = 3:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
    
    % Expand agent capacity if needed
    if world.n_agents < 9
        world.n_agents = 9;
        for i = length(world.agents)+1:9
            world.agents(i) = Agent(i, -100, -100, 0, 0);
        end
    end
    
    % Dynamic Agent 1: Cattle A (x=80m, y=5.8m, grazing on left shoulder)
    world = world.setAgentState(1, 80.0, 5.80, 0.00, 0.00, max(0.30, cfg.sigma_agent));
    world.agents(1).length = 1.8; world.agents(1).width = 0.8;
    
    % Dynamic Agent 2: Cattle B (x=150m, y=5.8m, grazing on left shoulder)
    world = world.setAgentState(2, 150.0, 5.80, 0.00, 0.00, max(0.30, cfg.sigma_agent));
    world.agents(2).length = 1.8; world.agents(2).width = 0.8;
    
    % Dynamic Agent 3: Oncoming Motorcycle in left lane (x=140m, y=4.8m, v=-3.5m/s)
    world = world.setAgentState(3, 140.0, 4.80, -3.50, 0.00, max(0.30, cfg.sigma_agent));
    world.agents(3).length = 2.0; world.agents(3).width = 0.8;
    
    % Dynamic Goat Herd Crossing Event (Agents 4..8: Goats)
    % Goats start on left shoulder (y=5.5m, x=85-92m), crossing dynamically at vy = -0.32m/s
    world = world.setAgentState(4, 85.0, 5.50, 0.08, -0.32, max(0.30, cfg.sigma_agent)); % Goat 1
    world.agents(4).length = 1.0; world.agents(4).width = 0.45;
    
    world = world.setAgentState(5, 87.0, 5.60, 0.07, -0.30, max(0.30, cfg.sigma_agent)); % Goat 2
    world.agents(5).length = 0.9; world.agents(5).width = 0.40;
    
    world = world.setAgentState(6, 88.5, 5.45, 0.09, -0.34, max(0.30, cfg.sigma_agent)); % Goat 3
    world.agents(6).length = 1.1; world.agents(6).width = 0.45;
    
    world = world.setAgentState(7, 90.0, 5.65, 0.06, -0.28, max(0.30, cfg.sigma_agent)); % Goat 4
    world.agents(7).length = 1.0; world.agents(7).width = 0.45;
    
    world = world.setAgentState(8, 92.0, 5.55, 0.10, -0.35, max(0.30, cfg.sigma_agent)); % Goat 5
    world.agents(8).length = 0.95; world.agents(8).width = 0.42;
    
    % Dynamic Agent 9: Herder Guide walking behind herd
    world = world.setAgentState(9, 89.0, 5.75, 0.07, -0.30, max(0.30, cfg.sigma_agent));
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    
    n_total = length(world.agents);
    for i = 10:n_total, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
    world.n_agents = 9;
end

function world = scenario_indian_realistic_demo_v5(world, cfg)
    % PHASE 41: Final Behavioral + Perception Realism Upgrade Scenario v5
    % Refined village-road geometry & synchronized obstacle timing:
    % - Ego Initial State: (x = 10.0m, y = 2.50m, theta = 0.0, v = 5.0m/s)
    % - Parked Auto-Rickshaw (x = 28.0m, y = -0.30m): gentle right-shoulder intrusion
    % - Roadside Stall (x = 55.0m, y = 5.80m): gentle left-shoulder intrusion
    % - Cattle A (x = 75.0m, y = 5.60m): grazing on left dirt shoulder
    % - Goat Herd Crossing Event (x = 82-89m, y = 5.40m -> 0.80m, vy = -0.28..-0.34 m/s):
    %   Synchronized to reach road corridor at t = 13.5s..17.5s as Ego arrives at x = 80m!
    % - Herder Guide (x = 86.5m, y = 5.70m -> 1.10m, vy = -0.28 m/s)
    % - Oncoming Motorcycle (x = 140.0m, y = 4.60m, vx = -3.5m/s)
    % - Cattle B (x = 150.0m, y = 5.60m)
    
    world = world.setEgoState(10.0, 2.50, 0.0, 5.0);
    
    % Static Obstacle 1: Parked Auto-Rickshaw partially encroaching right shoulder margin (x = 28.0m, y = -0.20m)
    world = world.setStaticObstacle(1, 28.0, -0.20, 2.00, 1.00);
    
    % Static Obstacle 2: Roadside Stall on left shoulder margin
    world = world.setStaticObstacle(2, 55.0, 6.20, 3.00, 1.00);
    
    for j = 3:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
    
    if world.n_agents < 9
        world.n_agents = 9;
        for i = length(world.agents)+1:9
            world.agents(i) = Agent(i, -100, -100, 0, 0);
        end
    end
    
    % Dynamic Agent 1: Cattle A (x=75m, y=6.0m, grazing on left shoulder)
    world = world.setAgentState(1, 75.0, 6.00, 0.00, 0.00, max(0.30, cfg.sigma_agent));
    world.agents(1).length = 1.8; world.agents(1).width = 0.8;
    
    % Dynamic Agent 2: Cattle B (x=150m, y=6.0m, grazing on left shoulder)
    world = world.setAgentState(2, 150.0, 6.00, 0.00, 0.00, max(0.30, cfg.sigma_agent));
    world.agents(2).length = 1.8; world.agents(2).width = 0.8;
    
    % Dynamic Agent 3: Oncoming Motorcycle in left lane (x=140m, y=4.6m, v=-3.5m/s)
    world = world.setAgentState(3, 140.0, 4.60, -3.50, 0.00, max(0.30, cfg.sigma_agent));
    world.agents(3).length = 2.0; world.agents(3).width = 0.8;
    
    % Dynamic Goat Herd Crossing Event (Agents 4..8: Compact Goat Herd crossing right-to-left at vy = +0.315m/s)
    world = world.setAgentState(4, 80.0, 0.50, 0.05, 0.315, max(0.30, cfg.sigma_agent)); % Goat 1
    world.agents(4).length = 1.0; world.agents(4).width = 0.45;
    
    world = world.setAgentState(5, 81.0, 0.30, 0.04, 0.320, max(0.30, cfg.sigma_agent)); % Goat 2
    world.agents(5).length = 0.9; world.agents(5).width = 0.40;
    
    world = world.setAgentState(6, 82.0, 0.10, 0.06, 0.310, max(0.30, cfg.sigma_agent)); % Goat 3
    world.agents(6).length = 1.1; world.agents(6).width = 0.45;
    
    world = world.setAgentState(7, 83.0, 0.40, 0.03, 0.315, max(0.30, cfg.sigma_agent)); % Goat 4
    world.agents(7).length = 1.0; world.agents(7).width = 0.45;
    
    world = world.setAgentState(8, 84.0, 0.20, 0.07, 0.325, max(0.30, cfg.sigma_agent)); % Goat 5
    world.agents(8).length = 0.95; world.agents(8).width = 0.42;
    
    % Dynamic Agent 9: Herder Guide walking behind herd
    world = world.setAgentState(9, 82.5, 0.00, 0.04, 0.310, max(0.30, cfg.sigma_agent));
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    world.agents(9).length = 0.60; world.agents(9).width = 0.50;
    
    n_total = length(world.agents);
    for i = 10:n_total, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
    world.n_agents = 9;
end






