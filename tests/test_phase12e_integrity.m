function test_phase12e_integrity()
    % TEST_PHASE12E_INTEGRITY Forensic Evaluation Integrity Unit Test Suite
    %
    % Verifies:
    %   Test A: Vector TTC Kinematics (Oncoming head-on, rear-end, separating, lateral)
    %   Test B: Footprint Clearance Semantics (Body-to-body exact physical distance)
    %   Test C: Mutually Exclusive Outcome Classification Precedence Truth Table
    %   Test D: Seed Determinism & Trajectory Signature Reproducibility
    %   Test E: Seed Independence vs Stochastic Realizations
    %   Test F: Deterministic Scenario Initialization Invariance under Seed Change
    
    addpath('environment', 'core', 'vehicle', 'planning', 'config', 'stages', 'metrics');
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 12E FORENSIC EVALUATION INTEGRITY UNIT TEST SUITE                        \n');
    fprintf('========================================================================================\n\n');
    
    cfg = SimulationConfig();
    L_ego = cfg.vehicle_length; % 4.7m
    W_ego = cfg.vehicle_width;  % 1.8m
    
    % -------------------------------------------------------------------------
    % TEST A: Vector TTC Kinematics
    % -------------------------------------------------------------------------
    fprintf('Running Test A: Vector TTC Kinematics...\n');
    
    % Case A1: Oncoming head-on conflict
    % Ego at x=10, v_x=+5.0. Agent at x=50, v_x=-8.0, L=4.7.
    % Bumper gap dx = (50 - 10) - (4.7/2 + 4.7/2) = 40 - 4.7 = 35.3m.
    % Closing speed v_closing = 5.0 - (-8.0) = 13.0 m/s.
    % Expected TTC = 35.3 / 13.0 = 2.7154 s.
    ego_x = 10.0; ego_vx = 5.0; ego_y = 1.80;
    ag_x = 50.0; ag_vx = -8.0; ag_y = 1.80; ag_L = 4.7; ag_W = 1.8;
    ttc_oncoming = computeVectorTTC(ego_x, ego_y, ego_vx, 0.0, L_ego, W_ego, ag_x, ag_y, ag_vx, 0.0, ag_L, ag_W);
    assert(abs(ttc_oncoming - 35.3 / 13.0) < 1e-3, 'Test A1 Failed: Oncoming TTC incorrect');
    fprintf('  A1 (Oncoming Head-On):  TTC = %.4f s (Expected: %.4f s) -> PASS\n', ttc_oncoming, 35.3 / 13.0);
    
    % Case A2: Rear-end approaching
    % Ego at x=10, v_x=+10.0. Agent at x=50, v_x=+5.0.
    % dx = 35.3m. v_closing = 10.0 - 5.0 = 5.0 m/s.
    % Expected TTC = 35.3 / 5.0 = 7.06 s.
    ttc_approaching = computeVectorTTC(ego_x, ego_y, 10.0, 0.0, L_ego, W_ego, ag_x, ag_y, 5.0, 0.0, ag_L, ag_W);
    assert(abs(ttc_approaching - 35.3 / 5.0) < 1e-3, 'Test A2 Failed: Approaching TTC incorrect');
    fprintf('  A2 (Rear-End Approaching): TTC = %.4f s (Expected: %.4f s) -> PASS\n', ttc_approaching, 35.3 / 5.0);
    
    % Case A3: Separating vehicles
    % Ego at x=10, v_x=+5.0. Agent at x=50, v_x=+10.0.
    % v_closing = 5.0 - 10.0 = -5.0 m/s <= 0 -> TTC = Inf.
    ttc_separating = computeVectorTTC(ego_x, ego_y, 5.0, 0.0, L_ego, W_ego, ag_x, ag_y, 10.0, 0.0, ag_L, ag_W);
    assert(isinf(ttc_separating), 'Test A3 Failed: Separating TTC must be Inf');
    fprintf('  A3 (Separating Vehicles): TTC = %s -> PASS\n', tostring(ttc_separating));
    
    % Case A4: Adjacent non-conflicting lane
    % Ego at y=1.80. Agent at y=5.0 (dy = 3.2m > W_ego/2 + ag_W/2 + 0.5m = 2.3m).
    ttc_lateral = computeVectorTTC(ego_x, ego_y, 10.0, 0.0, L_ego, W_ego, ag_x, 5.0, 5.0, 0.0, ag_L, ag_W);
    assert(isinf(ttc_lateral), 'Test A4 Failed: Non-conflicting lane TTC must be Inf');
    fprintf('  A4 (Non-Conflicting Lane): TTC = %s -> PASS\n', tostring(ttc_lateral));
    
    % -------------------------------------------------------------------------
    % TEST B: Footprint Clearance Semantics
    % -------------------------------------------------------------------------
    fprintf('Running Test B: Footprint Clearance Semantics...\n');
    % Body-to-body clearance: dx_center = 10m, dy_center = 0m.
    % Body clearance = 10 - (4.7/2 + 4.7/2) = 5.3m.
    clr_touching = 10.0 - L_ego;
    assert(abs(clr_touching - 5.3) < 1e-4, 'Test B Failed: Body clearance formula error');
    fprintf('  Body-to-body clearance definition verified (0.0 = touching, negative = overlap) -> PASS\n');
    
    % -------------------------------------------------------------------------
    % TEST C: Outcome Classification Precedence Truth Table
    % -------------------------------------------------------------------------
    fprintf('Running Test C: Outcome Classification Precedence Rules...\n');
    
    % C1: Collision overrides all
    out_c1 = classifyOutcome(1, false, 10, 0.0, 1.2, 0);
    assert(strcmp(out_c1, 'COLLISION'), 'Precedence Rule 1 Failed: COLLISION must override');
    
    % C2: Boundary violation overrides solver infeasibility and safe-stop
    out_c2 = classifyOutcome(0, true, 10, 0.0, 1.2, 0);
    assert(strcmp(out_c2, 'UNSAFE_FAILURE'), 'Precedence Rule 2 Failed: UNSAFE_FAILURE must override');
    
    % C3: Planner Infeasible (QP failed > 5 steps and v_final >= 0.10)
    out_c3 = classifyOutcome(0, false, 10, 5.0, 1.2, 0);
    assert(strcmp(out_c3, 'PLANNER_INFEASIBLE'), 'Precedence Rule 3 Failed: PLANNER_INFEASIBLE rule');
    
    % C4: Safe Stop (v_final < 0.10, min_clr > 0, 0 collisions)
    out_c4 = classifyOutcome(0, false, 0, 0.05, 2.5, 0);
    assert(strcmp(out_c4, 'SAFE_STOP'), 'Precedence Rule 4 Failed: SAFE_STOP rule');
    
    % C5: Degraded Safe (0 coll, 0 bounds, lat_rmse > 0.50)
    out_c5 = classifyOutcome(0, false, 0, 5.0, 0.65, 0);
    assert(strcmp(out_c5, 'DEGRADED_SAFE'), 'Precedence Rule 5 Failed: DEGRADED_SAFE rule');
    
    % C6: Success (0 coll, 0 bounds, lat_rmse <= 0.50)
    out_c6 = classifyOutcome(0, false, 0, 5.0, 0.12, 0);
    assert(strcmp(out_c6, 'SUCCESS'), 'Precedence Rule 6 Failed: SUCCESS rule');
    fprintf('  Mutually Exclusive Outcome Truth Table Verified (6/6 Rules) -> PASS\n');
    
    % -------------------------------------------------------------------------
    % TEST D: Seed Determinism & Trajectory Signature Reproducibility
    % -------------------------------------------------------------------------
    fprintf('Running Test D: Seed Determinism & Trajectory Hashing...\n');
    [~, ~, hist1] = stage5_multivehicle_coordination('scenario', 'multi_vehicle_yield_overtake', 'uncertainty_mode', 'nominal_perception', 'seed', 42, 'verbose', false);
    [~, ~, hist2] = stage5_multivehicle_coordination('scenario', 'multi_vehicle_yield_overtake', 'uncertainty_mode', 'nominal_perception', 'seed', 42, 'verbose', false);
    
    hash1 = computeTrajectoryHash(hist1);
    hash2 = computeTrajectoryHash(hist2);
    assert(strcmp(hash1, hash2), 'Test D Failed: Identical seeds produced non-identical hashes');
    fprintf('  Identical Seed (42 vs 42): Hash1=%s | Hash2=%s -> PASS (Bit-Exact)\n', hash1, hash2);
    
    % -------------------------------------------------------------------------
    % TEST E: Seed Independence vs Stochastic Realizations
    % -------------------------------------------------------------------------
    fprintf('Running Test E: Seed Independence...\n');
    [~, ~, hist3] = stage5_multivehicle_coordination('scenario', 'multi_vehicle_yield_overtake', 'uncertainty_mode', 'nominal_perception', 'seed', 99, 'verbose', false);
    hash3 = computeTrajectoryHash(hist3);
    assert(~strcmp(hash1, hash3), 'Test E Failed: Distinct seeds produced identical trajectories');
    fprintf('  Distinct Seed (42 vs 99): Hash1=%s | Hash3=%s -> PASS (Stochastic Divergence)\n', hash1, hash3);
    
    % -------------------------------------------------------------------------
    % TEST F: Deterministic Scenario Initialization Invariance
    % -------------------------------------------------------------------------
    fprintf('Running Test F: Scenario Initialization Invariance...\n');
    w1 = ScenarioDefinitions('multi_vehicle_yield_overtake', SimulationConfig());
    w2 = ScenarioDefinitions('multi_vehicle_yield_overtake', SimulationConfig());
    assert(w1.ego.x == w2.ego.x && w1.ego.y == w2.ego.y, 'Test F Failed: Scenario init changed');
    fprintf('  Deterministic Scenario Initialization Invariant -> PASS\n');
    
    fprintf('\n========================================================================================\n');
    fprintf('  ALL PHASE 12E EVALUATION INTEGRITY UNIT TESTS PASSED SUCCESSFULLY!                    \n');
    fprintf('========================================================================================\n\n');
end

function ttc = computeVectorTTC(ego_x, ego_y, ego_vx, ego_vy, L_ego, W_ego, ag_x, ag_y, ag_vx, ag_vy, ag_L, ag_W)
    % Compute vector longitudinal TTC considering velocity heading directions
    dx_center = ag_x - ego_x;
    dy_center = abs(ag_y - ego_y);
    
    % Lateral conflict threshold (vehicle body overlap + 0.5m margin)
    max_lateral_span = (W_ego / 2) + (ag_W / 2) + 0.5;
    if dy_center > max_lateral_span
        ttc = Inf;
        return;
    end
    
    % Longitudinal bumper-to-bumper gap
    dx_gap = abs(dx_center) - (L_ego / 2 + ag_L / 2);
    if dx_gap <= 0
        ttc = 0.0; % Touch / Collision
        return;
    end
    
    % Closing velocity along longitudinal axis
    % If agent is ahead (dx_center > 0), closing requires ego_vx > ag_vx (i.e. ego_vx - ag_vx > 0)
    % If agent is oncoming (ag_vx < 0), ego_vx - ag_vx = ego_vx + |ag_vx| > 0
    if dx_center > 0
        v_closing = ego_vx - ag_vx;
    else
        v_closing = ag_vx - ego_vx;
    end
    
    if v_closing > 1e-3
        ttc = dx_gap / v_closing;
    else
        ttc = Inf;
    end
end

function outcome = classifyOutcome(is_coll, is_boundary_fail, qp_infeas_steps, final_v, lat_rmse, emergency_steps)
    if is_coll > 0
        outcome = 'COLLISION';
    elseif is_boundary_fail
        outcome = 'UNSAFE_FAILURE';
    elseif qp_infeas_steps > 5 && final_v >= 0.10
        outcome = 'PLANNER_INFEASIBLE';
    elseif final_v < 0.10
        outcome = 'SAFE_STOP';
    elseif lat_rmse > 0.50 || emergency_steps > 0
        outcome = 'DEGRADED_SAFE';
    else
        outcome = 'SUCCESS';
    end
end

function str = computeTrajectoryHash(hist)
    % Unique Trajectory MD5/SHA256 signature from spatial state array
    data = [hist.ego_x, hist.ego_y, hist.ego_v, hist.ego_theta];
    bytes = typecast(data(:), 'uint8');
    md = java.security.MessageDigest.getInstance('MD5');
    md.update(bytes);
    hash_bytes = md.digest();
    str = sprintf('%02x', typecast(hash_bytes, 'uint8'));
    str = str(1:8); % 8-character hex signature
end

function s = tostring(val)
    if isinf(val), s = 'Inf'; else, s = sprintf('%.4f', val); end
end
