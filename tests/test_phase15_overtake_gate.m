function test_phase15_overtake_gate()
    % TEST_PHASE15_OVERTAKE_GATE Comprehensive Unit Verification for Spatial/TTC Overtake Gate
    %
    % Tests Spatial/TTC safety gate logic across cases A-G:
    %   A. Wide corridor + no oncoming -> OVERTAKE allowed
    %   B. Wide corridor + distant oncoming -> OVERTAKE allowed
    %   C. Narrow corridor + oncoming -> OVERTAKE rejected
    %   D. 6.0m road with two 1.8m vehicles & explicit safety margin -> Math correct
    %   E. TTC below threat threshold -> OVERTAKE rejected
    %   F. TTC above threshold & spatially feasible -> Gate allowed
    %   G. Determinism -> Identical inputs produce identical decisions
    
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir), projectRoot = pwd; else projectRoot = fileparts(current_dir); end
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'core'));
    
    fprintf('\n========================================================\n');
    fprintf('  PHASE 15A SPATIAL / TTC OVERTAKE SAFETY GATE UNIT TESTS\n');
    fprintf('========================================================\n\n');
    
    cfg = SimulationConfig();
    pass_count = 0;
    ctrl_gate = CoordinationDecisionLayer(8.0, 1.5, 5.0);
    
    % --- Test A: Wide Corridor + No Oncoming -> OVERTAKE Allowed ---
    worldA = WorldState(cfg);
    worldA = worldA.setEgoState(10.0, 1.80, 0.0, 8.0);
    worldA = worldA.setAgentState(1, 25.0, 1.80, 4.0, 0.0, cfg.sigma_agent);
    detA = MultiVehicleDetector.detect(worldA, worldA.ego);
    [ttcA, ~] = RiskPredictor.predictTTC(detA);
    interA = InteractionClassifier.classify(detA, worldA.ego, ttcA);
    decA = ctrl_gate.evaluate(detA, interA, worldA.ego);
    
    passA = strcmp(decA.macro_intent, 'OVERTAKE') && decA.overtake_allowed && ...
            strcmp(decA.overtake_block_reason, 'SPATIAL_PASS_FEASIBLE');
    print_unit_test('Test A: Wide corridor + no oncoming -> OVERTAKE allowed', passA);
    if passA, pass_count = pass_count + 1; end
    
    % --- Test B: Wide Corridor + Distant Oncoming (TTC > 6.0s) -> OVERTAKE Allowed ---
    ctrl_gate.reset();
    worldB = WorldState(cfg);
    worldB = worldB.setEgoState(10.0, 1.80, 0.0, 8.0);
    worldB = worldB.setAgentState(1, 25.0, 1.80, 4.0, 0.0, cfg.sigma_agent);
    worldB = worldB.setAgentState(2, 120.0, 4.20, -5.0, 0.0, cfg.sigma_agent); % Distant oncoming (dx=110m, closing=13m/s, TTC=8.46s)
    detB = MultiVehicleDetector.detect(worldB, worldB.ego);
    [ttcB, ~] = RiskPredictor.predictTTC(detB);
    interB = InteractionClassifier.classify(detB, worldB.ego, ttcB);
    decB = ctrl_gate.evaluate(detB, interB, worldB.ego);
    
    passB = strcmp(decB.macro_intent, 'OVERTAKE') && decB.overtake_allowed && decB.oncoming_ttc_s > 6.0;
    print_unit_test('Test B: Wide corridor + distant oncoming (TTC=8.5s) -> OVERTAKE allowed', passB);
    if passB, pass_count = pass_count + 1; end
    
    % --- Test C: Narrow Corridor + Oncoming -> OVERTAKE Rejected ---
    ctrl_gate.reset();
    worldC = WorldState(cfg);
    worldC = worldC.setEgoState(10.0, 1.80, 0.0, 8.0);
    worldC = worldC.setAgentState(1, 25.0, 1.80, 4.0, 0.0, cfg.sigma_agent);
    worldC = worldC.setAgentState(2, 50.0, 4.20, -7.0, 0.0, cfg.sigma_agent); % Oncoming at dx=40m, closing=15m/s, TTC=2.67s
    detC = MultiVehicleDetector.detect(worldC, worldC.ego);
    [ttcC, ~] = RiskPredictor.predictTTC(detC);
    interC = InteractionClassifier.classify(detC, worldC.ego, ttcC);
    decC = ctrl_gate.evaluate(detC, interC, worldC.ego);
    
    passC = strcmp(decC.macro_intent, 'YIELD') && ~decC.overtake_allowed && ...
            (strcmp(decC.overtake_block_reason, 'INSUFFICIENT_SPATIAL_GAP') || strcmp(decC.overtake_block_reason, 'TTC_TOO_LOW'));
    print_unit_test('Test C: Narrow corridor + oncoming -> OVERTAKE rejected (YIELD selected)', passC);
    if passC, pass_count = pass_count + 1; end
    
    % --- Test D: 6.0m Road with Two 1.8m Vehicles Math Verification ---
    % Lead vehicle right lane (y=1.8m, right edge=2.7m), Oncoming vehicle left lane (y=4.2m, left edge=3.3m)
    % Net clear gap = 3.3m - 2.7m = 0.6m. Required gap = 1.8m + 1.8m + 0.5m = 4.10m
    [is_feasD, availD, reqD, reasonD, ~] = ctrl_gate.is_spatial_overtake_feasible(detC(1), detC, interC, worldC.ego);
    
    passD = ~is_feasD && (reqD == 4.10) && (availD < reqD);
    print_unit_test(sprintf('Test D: Feasibility Math Verification (Req=%.2fm, Avail=%.2fm < 4.10m -> Blocked)', reqD, availD), passD);
    if passD, pass_count = pass_count + 1; end
    
    % --- Test E: TTC Below Threat Threshold (TTC = 3.0s <= 6.0s) -> Rejected ---
    ctrl_gate.reset();
    worldE = WorldState(cfg);
    worldE = worldE.setEgoState(10.0, 1.80, 0.0, 8.0);
    worldE = worldE.setAgentState(1, 25.0, 1.80, 4.0, 0.0, cfg.sigma_agent);
    worldE = worldE.setAgentState(2, 55.0, 5.50, -7.0, 0.0, cfg.sigma_agent); % Wide lateral position but TTC=3.0s
    detE = MultiVehicleDetector.detect(worldE, worldE.ego);
    [ttcE, ~] = RiskPredictor.predictTTC(detE);
    interE = InteractionClassifier.classify(detE, worldE.ego, ttcE);
    decE = ctrl_gate.evaluate(detE, interE, worldE.ego);
    
    passE = ~decE.overtake_allowed && strcmp(decE.overtake_block_reason, 'TTC_TOO_LOW');
    print_unit_test('Test E: TTC below threat threshold (TTC=3.0s <= 6.0s) -> OVERTAKE rejected', passE);
    if passE, pass_count = pass_count + 1; end
    
    % --- Test F: TTC Above Threshold AND Spatially Feasible -> Allowed ---
    ctrl_gate.reset();
    worldF = WorldState(cfg);
    worldF = worldF.setEgoState(10.0, 1.80, 0.0, 8.0);
    worldF = worldF.setAgentState(1, 25.0, 1.80, 4.0, 0.0, cfg.sigma_agent);
    worldF = worldF.setAgentState(2, 140.0, 4.20, -5.0, 0.0, cfg.sigma_agent); % Distant TTC = 10.0s > 6.0s, clear gap
    detF = MultiVehicleDetector.detect(worldF, worldF.ego);
    [ttcF, ~] = RiskPredictor.predictTTC(detF);
    interF = InteractionClassifier.classify(detF, worldF.ego, ttcF);
    decF = ctrl_gate.evaluate(detF, interF, worldF.ego);
    
    passF = decF.overtake_allowed && strcmp(decF.overtake_block_reason, 'SPATIAL_PASS_FEASIBLE');
    print_unit_test('Test F: TTC above threshold (10s > 6s) & spatially clear -> Gate Allowed', passF);
    if passF, pass_count = pass_count + 1; end
    
    % --- Test G: Determinism Verification ---
    ctrl_gate.reset();
    decG1 = ctrl_gate.evaluate(detC, interC, worldC.ego);
    ctrl_gate.reset();
    decG2 = ctrl_gate.evaluate(detC, interC, worldC.ego);
    
    passG = strcmp(decG1.macro_intent, decG2.macro_intent) && ...
            (decG1.overtake_allowed == decG2.overtake_allowed) && ...
            strcmp(decG1.overtake_block_reason, decG2.overtake_block_reason);
    print_unit_test('Test G: Determinism (identical inputs produce identical gate decisions)', passG);
    if passG, pass_count = pass_count + 1; end
    
    fprintf('\n========================================================\n');
    fprintf('  PHASE 15A OVERTAKE GATE UNIT TEST SUMMARY: %d/7 PASSED\n', pass_count);
    fprintf('========================================================\n\n');
    
    assert(pass_count == 7, 'Phase 15A Overtake Safety Gate Unit Tests Failed!');
end

function print_unit_test(title_str, is_pass)
    if is_pass
        fprintf('[PASS] %s\n', title_str);
    else
        fprintf('[FAIL] %s\n', title_str);
    end
end
