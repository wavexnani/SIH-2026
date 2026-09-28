classdef ScenarioLadder
    % SCENARIOLADDER Standardized 10-Level Evaluation Scenario Ladder
    %
    % Levels:
    %   Level 1: Nominal straight open road (Baseline driving)
    %   Level 2: Static obstacle avoidance
    %   Level 3: Road geometry (Narrowing and curved boundaries)
    %   Level 4: Dynamic following (Slow lead vehicle deceleration)
    %   Level 5: Dynamic yield interaction
    %   Level 6: Overtaking (Slow lead vehicle requiring lateral pass)
    %   Level 7: Oncoming conflict (Opposing dynamic agent)
    %   Level 8: Combined environment (Narrowing + static obs + dynamic vehicle)
    %   Level 9: Disturbed environment (Level 8 under perception & actuator noise)
    %   Level 10: Deliberately infeasible (Free-space collapse total blockage)

    properties (Constant)
        LEVEL_NAMES = {
            'Level 1: Nominal Straight',
            'Level 2: Static Obstacle',
            'Level 3: Road Geometry',
            'Level 4: Dynamic Following',
            'Level 5: Yield Interaction',
            'Level 6: Overtaking',
            'Level 7: Oncoming Conflict',
            'Level 8: Combined Environment',
            'Level 9: Disturbed Environment',
            'Level 10: Deliberately Infeasible'
        };
        
        SCENARIO_KEYS = {
            'clear',
            'static',
            'multi_obstacle_sequence',
            'multi_vehicle_following',
            'multi_vehicle_yield_overtake',
            'overtaking',
            'multi_vehicle_oncoming_conflict',
            'complex',
            'complex',
            'impassable_center'
        };
    end
    
    methods (Static)
        function [scen_key, name] = getLevel(level_idx)
            if level_idx < 1 || level_idx > 10
                error('Scenario level_idx must be between 1 and 10');
            end
            scen_key = ScenarioLadder.SCENARIO_KEYS{level_idx};
            name = ScenarioLadder.LEVEL_NAMES{level_idx};
        end
    end
end
