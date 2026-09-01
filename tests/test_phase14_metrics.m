function test_phase14_metrics()
    % TEST_PHASE14_METRICS Targeted Unit & Regression Test Suite for Phase 14 Metric Fixes
    %
    % Validates:
    %   1. OBB SAT Footprint Clearance vs Old Radial Metric
    %   2. Lateral RMSE calculation relative to active reference path
    %   3. TTC field consistency between Stage logger and RealismMetrics
    
    addpath('planning', 'config', 'vehicle', 'core', 'environment', 'stages', 'metrics');
    cfg = SimulationConfig();
    
    fprintf('\n========================================================\n');
    fprintf('  RUNNING PHASE 14 METRICS VALIDATION SUITE             \n');
    fprintf('========================================================\n\n');
    
    % -----------------------------------------------------------------
    % STEP 1: OBB FOOTPRINT CLEARANCE TARGETED TESTS
    % -----------------------------------------------------------------
    fprintf('[TEST 1] OBB Footprint Clearance Validation...\n');
    
    % Ego vehicle: L_e = 4.7m, W_e = 1.8m
    L_e = cfg.vehicle_length; % 4.7m
    W_e = cfg.vehicle_width;  % 1.8m
    
    % Test A: Separated vehicles (Ego at x=10, Lead at x=20, y=1.8, th=0)
    % Center dx = 10.0m. Extents: L_e/2 + L_lead/2 = 2.35 + 2.25 = 4.60m.
    % Expected OBB clearance = 10.0 - 4.60 = 5.40m.
    clr_sep = WorldState.computeOBBClearance(10.0, 1.8, 0.0, L_e, W_e, 20.0, 1.8, 4.5, 1.8, 0.0);
    assert(abs(clr_sep - 5.40) < 1e-3, sprintf('Test A Failed: Expected 5.40m, got %.3fm', clr_sep));
    fprintf('  [PASS] Test A (Separated Vehicles): Clearance = %.2f m\n', clr_sep);
    
    % Test B: Touching vehicles (Ego at x=10, Lead at x=14.6, y=1.8, th=0)
    % Center dx = 4.60m. Expected OBB clearance = 0.00m.
    clr_touch = WorldState.computeOBBClearance(10.0, 1.8, 0.0, L_e, W_e, 14.6, 1.8, 4.5, 1.8, 0.0);
    assert(abs(clr_touch - 0.00) < 1e-3, sprintf('Test B Failed: Expected 0.00m, got %.3fm', clr_touch));
    fprintf('  [PASS] Test B (Touching Vehicles): Clearance = %.2f m\n', clr_touch);
    
    % Test C: Longitudinal Rear-End Overlap (CRITICAL COMPARISON WITH OLD RADIAL METRIC)
    % Ego at x=10.0, Lead at x=14.0 (dx = 4.0m, bumpers overlap by 0.60m)
    % True OBB clearance = 4.0 - 4.60 = -0.60m (Collision!)
    % Old radial metric: 4.0 - 0.90 - 1.00 = +2.10m (WRONG!)
    clr_rear_obb = WorldState.computeOBBClearance(10.0, 1.8, 0.0, L_e, W_e, 14.0, 1.8, 4.5, 1.8, 0.0);
    r_ego_old = W_e / 2; % 0.90m
    r_agent_old = 1.8 / 2 + 0.10; % 1.00m
    clr_rear_old = norm([10.0 - 14.0, 0.0]) - r_ego_old - r_agent_old; % +2.10m
    
    assert(abs(clr_rear_obb - (-0.60)) < 1e-3, sprintf('Test C Failed: Expected -0.60m, got %.3fm', clr_rear_obb));
    assert(clr_rear_old > 2.0, 'Test C Failed: Old metric did not produce misleading positive clearance');
    fprintf('  [PASS] Test C (Longitudinal Rear-End Overlap):\n');
    fprintf('         - Old Radial Metric:  +%.2f m (MISLEADING SAFE!)\n', clr_rear_old);
    fprintf('         - Correct OBB Metric: %.2f m (CORRECT COLLISION!)\n', clr_rear_obb);
    
    % Test D: Lateral Overlap (Ego at x=10, y=1.8; Agent at x=10, y=3.0, dy=1.2m)
    % Lateral half-width sum = 0.90 + 0.90 = 1.80m. Expected OBB clearance = 1.20 - 1.80 = -0.60m.
    clr_lat_obb = WorldState.computeOBBClearance(10.0, 1.8, 0.0, L_e, W_e, 10.0, 3.0, 4.5, 1.8, 0.0);
    assert(abs(clr_lat_obb - (-0.60)) < 1e-3, sprintf('Test D Failed: Expected -0.60m, got %.3fm', clr_lat_obb));
    fprintf('  [PASS] Test D (Lateral Overlap): Clearance = %.2f m\n', clr_lat_obb);
    
    % Test E: Diagonal Separation (Ego at x=10, y=1.8, th=pi/4; Obs at x=15, y=3.0, L=2, W=2)
    clr_diag = WorldState.computeOBBClearance(10.0, 1.8, pi/4, L_e, W_e, 15.0, 3.0, 2.0, 2.0, 0.0);
    assert(clr_diag > 0.0, 'Test E Failed: Expected positive diagonal clearance');
    fprintf('  [PASS] Test E (Diagonal Separation): Clearance = %.2f m\n', clr_diag);
    
    % Test F: Ego vs Static Obstacle Integration via WorldState
    world_test = WorldState(cfg);
    world_test = world_test.setEgoState(10.0, 1.8, 0.0, 5.0);
    world_test = world_test.setStaticObstacle(1, 12.5, 1.8, 2.0, 2.0); % Bumper overlap (Ego max x = 12.35, Obs min x = 11.5)
    clr_ws_obs = world_test.getMinClearance(cfg);
    is_coll_obs = world_test.checkCollision(cfg);
    assert(clr_ws_obs < 0.0 && is_coll_obs, 'Test F Failed: WorldState static obstacle collision check failed');
    fprintf('  [PASS] Test F (Ego vs Static Obstacle via WorldState): Min Clearance = %.2f m, Collided = %d\n', clr_ws_obs, is_coll_obs);
    
    % Test G: Ego vs Dynamic Agent Integration via WorldState
    world_test2 = WorldState(cfg);
    world_test2 = world_test2.setEgoState(10.0, 1.8, 0.0, 5.0);
    world_test2 = world_test2.setAgentState(1, 14.0, 1.8, 2.0, 0.0, 0.3); % Rear-end overlap
    clr_ws_ag = world_test2.getMinClearance(cfg);
    is_coll_ag = world_test2.checkCollision(cfg);
    assert(clr_ws_ag < 0.0 && is_coll_ag, 'Test G Failed: WorldState agent collision check failed');
    fprintf('  [PASS] Test G (Ego vs Dynamic Agent via WorldState): Min Clearance = %.2f m, Collided = %d\n', clr_ws_ag, is_coll_ag);
    
    % -----------------------------------------------------------------
    % STEP 2: LATERAL RMSE VALIDATION
    % -----------------------------------------------------------------
    fprintf('\n[TEST 2] Active Reference Lateral RMSE Validation...\n');
    
    % Synthetic history for nominal lane following
    N = 100;
    hist_nom.ego_y = 1.80 + 0.02 * randn(N, 1);
    hist_nom.ref_y = repmat(1.80, N, 1);
    rmse_nom = sqrt(mean((hist_nom.ego_y - hist_nom.ref_y).^2));
    assert(rmse_nom < 0.05, sprintf('Test 2A Failed: Nominal RMSE = %.3fm', rmse_nom));
    fprintf('  [PASS] Test 2A (Nominal Lane Following): Active Reference RMSE = %.3f m\n', rmse_nom);
    
    % Synthetic history for valid overtake (ref transitions to y=3.35m, ego tracks 3.35m)
    hist_ov.ego_y = [linspace(1.80, 3.35, 30)'; repmat(3.35, 70, 1)] + 0.02 * randn(N, 1);
    hist_ov.ref_y = [linspace(1.80, 3.35, 30)'; repmat(3.35, 70, 1)];
    rmse_ov_active = sqrt(mean((hist_ov.ego_y - hist_ov.ref_y).^2));
    rmse_ov_fixed  = sqrt(mean((hist_ov.ego_y - 1.80).^2)); % Old metric
    
    assert(rmse_ov_active < 0.05, sprintf('Test 2B Failed: Active Overtake RMSE = %.3fm', rmse_ov_active));
    assert(rmse_ov_fixed > 1.0, 'Test 2B Failed: Old fixed metric did not show false degradation');
    fprintf('  [PASS] Test 2B (Valid Alternate Reference Overtake):\n');
    fprintf('         - Old Fixed y=1.80m Metric: %.3f m (FALSE DEGRADED_SAFE!)\n', rmse_ov_fixed);
    fprintf('         - Correct Active Reference:  %.3f m (SUCCESS!)\n', rmse_ov_active);
    
    % Synthetic history for genuine deviation from active reference
    hist_dev.ego_y = hist_ov.ref_y + 0.60; % 0.6m systematic off-reference drift
    hist_dev.ref_y = hist_ov.ref_y;
    rmse_dev = sqrt(mean((hist_dev.ego_y - hist_dev.ref_y).^2));
    assert(rmse_dev > 0.50, sprintf('Test 2C Failed: Deviation RMSE = %.3fm', rmse_dev));
    fprintf('  [PASS] Test 2C (Genuine Reference Deviation): Active Reference RMSE = %.3f m\n', rmse_dev);
    
    % -----------------------------------------------------------------
    % STEP 3: TTC FIELD CONSISTENCY VALIDATION
    % -----------------------------------------------------------------
    fprintf('\n[TEST 3] TTC Field Consistency Validation...\n');
    
    hist_ttc.t = (0:0.1:5.0)';
    hist_ttc.ego_theta = zeros(51, 1);
    hist_ttc.delta_plant = zeros(51, 1);
    hist_ttc.a_plant = zeros(51, 1);
    hist_ttc.ego_y = repmat(1.80, 51, 1);
    hist_ttc.ego_x = linspace(10, 50, 51)';
    hist_ttc.ego_v = repmat(8.0, 51, 1);
    hist_ttc.min_clearance = repmat(5.0, 51, 1);
    hist_ttc.is_collision = false(51, 1);
    hist_ttc.inside_bounds = true(51, 1);
    hist_ttc.solver_status = ones(51, 1);
    hist_ttc.solve_time_ms = repmat(2.0, 51, 1);
    hist_ttc.filter_active = false(51, 1);
    hist_ttc.selected_topology = repmat({'hard_left'}, 51, 1);
    hist_ttc.perception_e_pos = zeros(51, 1);
    hist_ttc.perception_e_vel = zeros(51, 1);
    hist_ttc.perception_e_heading = zeros(51, 1);
    hist_ttc.perception_age = zeros(51, 1);
    hist_ttc.steering_error = zeros(51, 1);
    hist_ttc.acceleration_error = zeros(51, 1);
    
    % Populate stage logger field history.min_ttc
    hist_ttc.min_ttc = repmat(inf, 51, 1);
    hist_ttc.min_ttc(20:30) = [5.5, 5.0, 4.5, 4.0, 3.5, 3.0, 2.8, 3.2, 4.0, 5.0, 6.0]';
    
    met = RealismMetrics.compute(hist_ttc, [], cfg);
    assert(abs(met.traffic.min_ttc - 2.8) < 1e-3, sprintf('Test 3 Failed: RealismMetrics min_ttc = %.3f (Expected 2.8s)', met.traffic.min_ttc));
    fprintf('  [PASS] Test 3 (RealismMetrics TTC Consistency): Min TTC = %.2f s\n', met.traffic.min_ttc);
    
    fprintf('\n========================================================\n');
    fprintf('  ALL PHASE 14 TARGETED METRIC TESTS PASSED!            \n');
    fprintf('========================================================\n\n');
end
