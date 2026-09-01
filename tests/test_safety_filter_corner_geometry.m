function test_safety_filter_corner_geometry()
    % TEST_SAFETY_FILTER_CORNER_GEOMETRY Unit Test for Vehicle Corner Clearance Geometry
    %
    % Purpose:
    %   Verifies that SafetyFilter correctly transforms rear-axle state predictions
    %   to geometric body center footprint coordinates, accurately evaluating
    %   front bumper and rear bumper corner obstacle intersections.
    
    addpath('planning', 'config', 'core', 'environment', 'stages');
    
    fprintf('\n========================================================================================\n');
    fprintf('        UNIT TEST: SAFETY FILTER FRONT/REAR CORNER GEOMETRY AUDIT                      \n');
    fprintf('========================================================================================\n\n');
    
    cfg = SimulationConfig();
    sf = SafetyFilter(cfg);
    
    % Ego vehicle at rear axle (x=10.0, y=3.0, theta=0.0)
    world = WorldState(cfg);
    world.ego = EgoState(10.0, 3.0, 0.0, 5.0, 0.0);
    
    % Predicted trajectory (10 steps along straight line)
    Np = 10;
    pred_states = zeros(Np, 4);
    for k = 1:Np
        pred_states(k, :) = [10.0 + (k-1)*0.5, 3.0, 0.0, 5.0];
    end
    
    % Case 1: Obstacle near Front Bumper Corner
    % Vehicle length = 4.7m, wheelbase = 2.7m -> body center = 10.0 + 1.35 = 11.35m
    % Bounding box front edge = 11.35 + 2.35 = 13.7m
    % Place obstacle at x = 14.0m, y = 3.0m (obs length 1.0m, dx_overlap = (4.7+1.0)/2 = 2.85m)
    % |11.35 - 14.0| = 2.65m <= 2.85m -> MUST DETECT
    world.n_static_obs = 1;
    world.static_obs = [14.0, 3.0, 1.0, 1.0];
    
    [u_safe, filter_active, reason] = sf.filter([0.0; 0.0], 1, world, pred_states);
    assert(filter_active, 'SafetyFilter MUST trigger for obstacle near front bumper corner');
    fprintf('[PASS] Case 1: Front bumper corner collision correctly detected.\n');
    
    % Case 2: Obstacle Cleared Past Rear Bumper (x = 6.0m, behind rear axle x=10.0m)
    world.static_obs = [6.0, 3.0, 1.0, 1.0];
    [u_safe, filter_active, reason] = sf.filter([0.0; 0.0], 1, world, pred_states);
    assert(~filter_active, 'SafetyFilter MUST NOT trigger for obstacle behind rear bumper');
    fprintf('[PASS] Case 2: Cleared obstacle behind rear bumper correctly ignored.\n');
    
    fprintf('\n========================================================================================\n');
    fprintf('        ALL CORNER GEOMETRY TESTS PASSED SUCCESSFULLY!                                   \n');
    fprintf('========================================================================================\n\n');
end
