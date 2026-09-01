function test_stage5_perception()
    % TEST_STAGE5_PERCEPTION Independent Standalone Unit Verification for Perception & Risk Estimation
    %
    % Tests MultiVehicleDetector state extraction, RiskPredictor TTC estimation,
    % and RiskPredictor dynamic trajectory prediction against analytical ground truth.
    
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir), projectRoot = pwd; else projectRoot = fileparts(current_dir); end
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'core'));
    
    fprintf('\n========================================================\n');
    fprintf('  STAGE 5.1 PERCEPTION & RISK PREDICTION UNIT TEST SUITE\n');
    fprintf('========================================================\n\n');
    
    cfg = SimulationConfig();
    pass_count = 0;
    
    % --- Mock World Setup ---
    world = WorldState(cfg);
    world = world.setEgoState(10.0, 2.0, 0.0, 8.0); % Ego @ x=10m, y=2.0m, theta=0.0, v=8.0m/s
    world = world.setAgentState(1, 30.0, 2.0, 4.0, 0.0, cfg.sigma_agent); % Lead Agent @ x=30m, y=2.0m, vx=4m/s
    world = world.setAgentState(2, 50.0, 3.75, -7.0, 0.0, cfg.sigma_agent); % Oncoming Agent @ x=50m, y=3.75m, vx=-7m/s
    
    % --- Test 1: Relative State Detection ---
    detections = MultiVehicleDetector.detect(world, world.ego);
    
    pass1_det1 = (length(detections) == 2) && ...
                 abs(detections(1).dx - 20.0) < 1e-3 && ...
                 abs(detections(1).dy - 0.0) < 1e-3 && ...
                 abs(detections(1).dvx - (-4.0)) < 1e-3 && ...
                 detections(1).is_ahead && ~detections(1).is_behind && detections(1).is_same_lane;
                 
    pass1_det2 = abs(detections(2).dx - 40.0) < 1e-3 && ...
                 abs(detections(2).dy - 1.75) < 1e-3 && ...
                 detections(2).is_oncoming;
                 
    pass1 = pass1_det1 && pass1_det2;
    print_unit_test('Test 1: MultiVehicleDetector Relative Extraction (dx, dy, dvx, flags)', pass1);
    if pass1, pass_count = pass_count + 1; end
    
    % --- Test 2: Analytical TTC Estimation ---
    [ttc_vec, min_ttc] = RiskPredictor.predictTTC(detections, 4.5);
    % Lead clearance = 20 - 4.5 = 15.5m. Closing rate = 8 - 4 = 4.0 m/s. Expected TTC1 = 15.5 / 4.0 = 3.875s
    % Oncoming clearance = 40.0m. Closing rate = 8 + 7 = 15.0 m/s. Expected TTC2 = 40.0 / 15.0 = 2.6667s
    expected_ttc1 = 15.5 / 4.0;
    expected_ttc2 = (40.0 - 4.5) / 15.0;
    err_ttc1 = abs(ttc_vec(1) - expected_ttc1);
    err_ttc2 = abs(ttc_vec(2) - expected_ttc2);
    pass2 = (err_ttc1 < 1e-2) && (err_ttc2 < 1e-2) && (abs(min_ttc - min(expected_ttc1, expected_ttc2)) < 1e-2);
    print_unit_test('Test 2: RiskPredictor TTC Calculation (Lead TTC=3.875s, Oncoming TTC=2.667s)', pass2);
    if pass2, pass_count = pass_count + 1; end
    
    % --- Test 3: Kinematic Trajectory Prediction ---
    Np = 10; dt = 0.10;
    preds = RiskPredictor.predictTrajectories(detections, Np, dt);
    
    % Check predicted end position for Agent 1 at step 10 (t = 1.0s) => x_end = 30 + 4.0 * 1.0 = 34.0m
    % Check predicted end position for Agent 2 at step 10 (t = 1.0s) => x_end = 50 - 7.0 * 1.0 = 43.0m
    pass3_p1 = (length(preds) == 2) && abs(preds(1).x_traj(10) - 34.0) < 1e-3;
    pass3_p2 = abs(preds(2).x_traj(10) - 43.0) < 1e-3;
    pass3 = pass3_p1 && pass3_p2;
    print_unit_test('Test 3: RiskPredictor State-Based Trajectory Prediction (Np=10, dt=0.1s)', pass3);
    if pass3, pass_count = pass_count + 1; end
    
    fprintf('\n========================================================\n');
    fprintf('  PERCEPTION & RISK UNIT TEST SUMMARY: %d/3 PASSED\n', pass_count);
    fprintf('========================================================\n\n');
    
    assert(pass_count == 3, 'Stage 5.1 Perception Unit Verification Failed!');
end

function print_unit_test(title_str, is_pass)
    if is_pass
        fprintf('[PASS] %s\n', title_str);
    else
        fprintf('[FAIL] %s\n', title_str);
    end
end
