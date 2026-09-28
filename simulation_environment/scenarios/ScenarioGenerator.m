classdef ScenarioGenerator
    % SCENARIOGENERATOR Parameterized Multi-Level Scenario Factory
    %
    % Purpose:
    %   Constructs realistic, reproducible scenarios across 4 difficulty tiers:
    %     Level 1 — Nominal (Large free space, high feasibility)
    %     Level 2 — Moderate (Gradual/asymmetric narrowing, S-curves, obstacle sequences)
    %     Level 3 — Challenging (Tight corridors, oncoming conflicts, constrained overtaking)
    %     Level 4 — Infeasible (Physically impossible passages to test emergency response)
    
    methods (Static)
        function [world, map_obj, level_num, name_str] = createScenario(scenario_id, cfg)
            if nargin < 2 || isempty(cfg)
                cfg = SimulationConfig();
            end
            
            switch lower(scenario_id)
                % === LEVEL 1: NOMINAL ===
                case {'level1_nominal', 'nominal_straight'}
                    level_num = 1; name_str = 'Level 1: Nominal Straight Road';
                    world = WorldState(cfg);
                    world = world.setEgoState(10, 3.0, 0, 5.0);
                    for i = 1:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
                    for j = 1:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
                    map_obj = FreeSpaceMap.createCorridorMap(0.0, 6.0);
                    
                % === LEVEL 2: MODERATE ===
                case {'level2_narrowing', 'gradual_narrowing'}
                    level_num = 2; name_str = 'Level 2: Gradual Asymmetric Road Narrowing';
                    world = WorldState(cfg);
                    world = world.setEgoState(10, 3.0, 0, 5.0);
                    world = world.setStaticObstacle(1, 45.0, 1.50, 1.80, 1.80);
                    for j = 2:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
                    for i = 1:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
                    map_obj = FreeSpaceMap.createUnstructuredMap();
                    
                case {'level2_scurve', 'scurve_road'}
                    level_num = 2; name_str = 'Level 2: Curved S-Shape Road Corridor';
                    world = WorldState(cfg);
                    world = world.setEgoState(10, 3.0, 0, 5.0);
                    for j = 1:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
                    for i = 1:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
                    
                    % Construct S-Curve FreeSpaceMap
                    N_pts = 100;
                    x_pts = linspace(0, 150, N_pts)';
                    y_center = 3.0 + 1.5 * sin(x_pts / 15.0);
                    y_min = y_center - 2.5;
                    y_max = y_center + 2.5;
                    map_obj = FreeSpaceMap([x_pts, y_min, y_max]);
                    
                % === LEVEL 3: CHALLENGING ===
                case {'level3_tight_corridor', 'tight_corridor'}
                    level_num = 3; name_str = 'Level 3: Constrained Narrow Free-Space Corridor';
                    world = WorldState(cfg);
                    world = world.setEgoState(10, 3.0, 0, 5.0);
                    world = world.setStaticObstacle(1, 40.0, 1.80, 1.80, 1.80);
                    world = world.setStaticObstacle(2, 65.0, 3.20, 1.80, 1.80);
                    for j = 3:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
                    for i = 1:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
                    
                    % Construct Tight Road Constriction
                    N_pts = 100;
                    x_pts = linspace(0, 150, N_pts)';
                    y_min = zeros(N_pts, 1); y_max = 6.0 * ones(N_pts, 1);
                    for k = 1:N_pts
                        px = x_pts(k);
                        if px >= 30 && px <= 70
                            % Pinch road width to 3.2m
                            s = (px - 30) / 40.0;
                            w_pinch = 6.0 - 2.8 * sin(pi * s);
                            y_min(k) = (6.0 - w_pinch) / 2.0;
                            y_max(k) = y_min(k) + w_pinch;
                        end
                    end
                    map_obj = FreeSpaceMap([x_pts, y_min, y_max]);
                    
                % === LEVEL 4: INFEASIBLE ===
                case {'level4_infeasible', 'infeasible_blocked'}
                    level_num = 4; name_str = 'Level 4: Deliberately Infeasible Total Blockage';
                    world = WorldState(cfg);
                    world = world.setEgoState(10, 3.0, 0, 5.0);
                    % Total blockage across entire width at x=40m
                    world = world.setStaticObstacle(1, 40.0, 1.00, 2.50, 2.50);
                    world = world.setStaticObstacle(2, 40.0, 3.50, 2.50, 2.50);
                    for j = 3:cfg.n_static_obs, world = world.setStaticObstacle(j, -100, -100, 1.0, 1.0); end
                    for i = 1:cfg.n_agents, world = world.setAgentState(i, -100, -100, 0, 0, cfg.sigma_agent); end
                    map_obj = FreeSpaceMap.createCorridorMap(0.0, 6.0);
                    
                otherwise
                    error('Unknown scenario ID: %s', scenario_id);
            end
        end
    end
end
