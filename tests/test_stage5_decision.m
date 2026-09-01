function test_stage5_decision()
    % TEST_STAGE5_DECISION Independent Standalone Unit Verification for Interaction & Macro-Intent Selection
    %
    % Tests InteractionClassifier and CoordinationDecisionLayer state logic across
    % deterministic mock interaction state matrices (MAINTAIN, YIELD, OVERTAKE, LATCHING).
    
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir), projectRoot = pwd; else projectRoot = fileparts(current_dir); end
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'core'));
    
    fprintf('\n========================================================\n');
    fprintf('  STAGE 5.1 INTERACTION & DECISION LAYER UNIT TEST SUITE\n');
    fprintf('========================================================\n\n');
    
    cfg = SimulationConfig();
    pass_count = 0;
    decision_layer = CoordinationDecisionLayer(8.0, 1.5, 5.0);
    
    % --- Test 1: Nominal Clear Road (MAINTAIN) ---
    world1 = WorldState(cfg);
    world1 = world1.setEgoState(10.0, 2.0, 0.0, 8.0);
    det1 = MultiVehicleDetector.detect(world1, world1.ego);
    [ttc1, ~] = RiskPredictor.predictTTC(det1);
    inter1 = InteractionClassifier.classify(det1, world1.ego, ttc1);
    dec1 = decision_layer.evaluate(det1, inter1, world1.ego);
    
    pass1 = strcmp(dec1.macro_intent, 'MAINTAIN') && (dec1.target_v == 8.0) && dec1.allow_overtake;
    print_unit_test('Test 1: Clear Road Intent Selection (MAINTAIN)', pass1);
    if pass1, pass_count = pass_count + 1; end
    
    % --- Test 2: Slower Lead Vehicle + Clear Oncoming Lane (OVERTAKE) ---
    world2 = WorldState(cfg);
    world2 = world2.setEgoState(10.0, 2.0, 0.0, 8.0);
    world2 = world2.setAgentState(1, 30.0, 2.0, 4.0, 0.0, cfg.sigma_agent);
    det2 = MultiVehicleDetector.detect(world2, world2.ego);
    [ttc2, ~] = RiskPredictor.predictTTC(det2);
    inter2 = InteractionClassifier.classify(det2, world2.ego, ttc2);
    dec2 = decision_layer.evaluate(det2, inter2, world2.ego);
    
    pass2 = strcmp(dec2.macro_intent, 'OVERTAKE') && (decision_layer.active_overtake_id == 1);
    print_unit_test('Test 2: Clear Corridor Overtake Selection (OVERTAKE on Agent 1)', pass2);
    if pass2, pass_count = pass_count + 1; end
    
    % --- Test 3: Slower Lead Vehicle + Oncoming Threat (YIELD) ---
    decision_layer.reset();
    world3 = WorldState(cfg);
    world3 = world3.setEgoState(10.0, 2.0, 0.0, 8.0);
    world3 = world3.setAgentState(1, 30.0, 2.0, 3.5, 0.0, cfg.sigma_agent);
    world3 = world3.setAgentState(2, 50.0, 3.75, -7.0, 0.0, cfg.sigma_agent); % Oncoming in left lane
    det3 = MultiVehicleDetector.detect(world3, world3.ego);
    [ttc3, ~] = RiskPredictor.predictTTC(det3);
    inter3 = InteractionClassifier.classify(det3, world3.ego, ttc3);
    dec3 = decision_layer.evaluate(det3, inter3, world3.ego);
    
    pass3 = strcmp(dec3.macro_intent, 'YIELD') && (dec3.target_v <= 5.0) && ~dec3.allow_overtake;
    print_unit_test('Test 3: Oncoming Conflict Yielding Intent (YIELD behind Agent 1)', pass3);
    if pass3, pass_count = pass_count + 1; end
    
    % --- Test 4: Oncoming Threat without Lead Vehicle (YIELD) ---
    decision_layer.reset();
    world4 = WorldState(cfg);
    world4 = world4.setEgoState(10.0, 2.0, 0.0, 8.0);
    world4 = world4.setAgentState(1, 35.0, 3.75, -7.0, 0.0, cfg.sigma_agent);
    det4 = MultiVehicleDetector.detect(world4, world4.ego);
    [ttc4, ~] = RiskPredictor.predictTTC(det4);
    inter4 = InteractionClassifier.classify(det4, world4.ego, ttc4);
    dec4 = decision_layer.evaluate(det4, inter4, world4.ego);
    
    pass4 = strcmp(dec4.macro_intent, 'YIELD') && ~dec4.allow_overtake && dec4.target_v < 8.0;
    print_unit_test('Test 4: Oncoming-Only Conflict Selection (YIELD)', pass4);
    if pass4, pass_count = pass_count + 1; end

    % --- Test 5: Overtake Intent Latching & De-latching ---
    % Simulate ego overtaking Agent 1, starting in OVERTAKE state
    decision_layer.active_overtake_id = 1;
    world5 = WorldState(cfg);
    world5 = world5.setEgoState(40.0, 3.75, 0.0, 8.0); % Ego @ x=40m, past Agent 1 @ x=30m (dx = -10m)
    world5 = world5.setAgentState(1, 30.0, 2.0, 3.5, 0.0, cfg.sigma_agent);
    det5 = MultiVehicleDetector.detect(world5, world5.ego);
    [ttc5, ~] = RiskPredictor.predictTTC(det5);
    inter5 = InteractionClassifier.classify(det5, world5.ego, ttc5);
    dec5 = decision_layer.evaluate(det5, inter5, world5.ego);
    
    pass5 = (decision_layer.active_overtake_id <= 0) && strcmp(dec5.macro_intent, 'MAINTAIN');
    print_unit_test('Test 5: Overtake Intent De-latching (Clearance dx < -5.0m -> MAINTAIN)', pass5);
    if pass5, pass_count = pass_count + 1; end
    
    fprintf('\n========================================================\n');
    fprintf('  INTERACTION & DECISION UNIT TEST SUMMARY: %d/5 PASSED\n', pass_count);
    fprintf('========================================================\n\n');
    
    assert(pass_count == 5, 'Stage 5.1 Decision Unit Verification Failed!');
end

function print_unit_test(title_str, is_pass)
    if is_pass
        fprintf('[PASS] %s\n', title_str);
    else
        fprintf('[FAIL] %s\n', title_str);
    end
end
