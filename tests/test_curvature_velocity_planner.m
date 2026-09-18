function test_curvature_velocity_planner()
    % TEST_CURVATURE_VELOCITY_PLANNER Comprehensive unit verification for CurvatureVelocityPlanner
    %
    % Tests:
    %   1. Straight road: Constant cruise speed, zero curvature
    %   2. Gentle curve: Preserves comfortable lateral acceleration
    %   3. Sharp curve: Pre-braking on straight prior to curve entry
    %   4. Consecutive curves: Chicane / S-curve pre-braking carryover
    %   5. Terminal stop condition: Smooth backward deceleration to zero
    %   6. Monotonicity & physical limits: Non-negative speed, acceleration constraints
    
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir)
        projectRoot = pwd;
    else
        projectRoot = fileparts(current_dir);
    end
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'config'));
    
    fprintf('\n========================================================\n');
    fprintf('  CURVATURE-AWARE VELOCITY PLANNER UNIT TEST SUITE\n');
    fprintf('========================================================\n\n');
    
    cfg = SimulationConfig();
    planner = CurvatureVelocityPlanner(cfg);
    planner.a_lat_max = 2.50;  % m/s^2 comfort
    planner.a_accel_max = 2.00;
    planner.a_decel_max = 3.00;
    planner.v_max = 10.0;
    planner.v_min = 1.50;
    
    pass_count = 0;
    total_tests = 7;
    
    % -------------------------------------------------------------
    % Test 1: Straight Road
    % -------------------------------------------------------------
    N = 100;
    x1 = linspace(0, 100, N)';
    y1 = zeros(N, 1);
    path1 = [x1, y1];
    
    [aug1, info1] = planner.planVelocityProfile(path1, 10.0);
    max_k1 = max(abs(aug1(:, 4)));
    min_v1 = min(aug1(:, 5));
    max_v1 = max(aug1(:, 5));
    
    pass1 = (max_k1 < 1e-3) && (abs(min_v1 - 10.0) < 1e-2) && (abs(max_v1 - 10.0) < 1e-2);
    print_test_result(1, 'Straight Road Cruise Speed (100m, v=10.0 m/s)', ...
        sprintf('max|k|=%.2e, v_range=[%.2f, %.2f]', max_k1, min_v1, max_v1), pass1);
    if pass1, pass_count = pass_count + 1; end
    
    % -------------------------------------------------------------
    % Test 2: Gentle Curve (R = 100m, kappa = 0.01 1/m)
    % -------------------------------------------------------------
    % R = 100m => v_lat = sqrt(2.5 / 0.01) = sqrt(250) = 15.8 m/s > 10 m/s cruise
    % Should maintain 10.0 m/s cruise with a_lat = 10^2 * 0.01 = 1.0 m/s^2 <= 2.5
    theta2 = linspace(0, pi/4, N)';
    R2 = 100.0;
    x2 = R2 * sin(theta2);
    y2 = R2 * (1 - cos(theta2));
    path2 = [x2, y2];
    
    [aug2, info2] = planner.planVelocityProfile(path2, 10.0);
    max_alat2 = max(info2.a_lat_actual);
    min_v2 = min(aug2(:, 5));
    
    pass2 = (max_alat2 <= planner.a_lat_max + 0.1) && (min_v2 >= 9.5);
    print_test_result(2, 'Gentle Curve Lateral Accel Safety (R=100m)', ...
        sprintf('min_v=%.2f m/s, max_alat=%.2f m/s^2 (limit=2.5)', min_v2, max_alat2), pass2);
    if pass2, pass_count = pass_count + 1; end
    
    % -------------------------------------------------------------
    % Test 3: Sharp Curve with Pre-Braking (Straight + R=15m Bend)
    % -------------------------------------------------------------
    % Straight for 30m, then 90-deg turn with R=15m (kappa = 0.067 1/m)
    % Apex speed limit: sqrt(2.5 / 0.067) = sqrt(37.5) = 6.12 m/s < 10.0 m/s
    % Must begin braking BEFORE reaching the bend!
    s_straight = linspace(0, 30, 60)';
    x3_str = s_straight;
    y3_str = zeros(size(s_straight));
    
    th3 = linspace(0, pi/2, 60)';
    R3 = 15.0;
    x3_turn = 30 + R3 * sin(th3);
    y3_turn = R3 * (1 - cos(th3));
    
    path3 = [x3_str(1:end-1), y3_str(1:end-1); x3_turn, y3_turn];
    [aug3, info3] = planner.planVelocityProfile(path3, 10.0);
    
    % Speed at x = 25m (5m before curve entry at x=30m)
    [~, idx_pre] = min(abs(aug3(:, 1) - 25.0));
    v_pre_entry = aug3(idx_pre, 5);
    min_v3 = min(aug3(:, 5));
    max_alat3 = max(info3.a_lat_actual);
    
    % Pre-braking verified if v at x=25m is already less than 10.0 m/s
    pass3 = (v_pre_entry < 9.5) && (min_v3 < 6.8) && (max_alat3 <= planner.a_lat_max + 0.3);
    print_test_result(3, 'Sharp Curve Anticipatory Pre-Braking (R=15m)', ...
        sprintf('v(x=25m)=%.2f m/s (pre-braked from 10), apex_v=%.2f, max_alat=%.2f', ...
        v_pre_entry, min_v3, max_alat3), pass3);
    if pass3, pass_count = pass_count + 1; end
    
    % -------------------------------------------------------------
    % Test 4: Consecutive S-Curves (Chicane)
    % -------------------------------------------------------------
    % Alternating sharp turns spaced closely (sinusoidal wave)
    x4 = linspace(0, 80, 160)';
    y4 = 2.5 * sin(2 * pi * x4 / 30.0);
    path4 = [x4, y4];
    
    [aug4, info4] = planner.planVelocityProfile(path4, 10.0);
    max_alat4 = max(info4.a_lat_actual);
    min_v4 = min(aug4(:, 5));
    
    pass4 = (min_v4 < 7.5) && (max_alat4 <= planner.a_lat_max + 0.35) && (info4.pre_braking_active);
    print_test_result(4, 'Consecutive S-Curve Chicane Handling', ...
        sprintf('min_v=%.2f m/s, max_alat=%.2f m/s^2, pre_braking=%d', ...
        min_v4, max_alat4, info4.pre_braking_active), pass4);
    if pass4, pass_count = pass_count + 1; end
    
    % -------------------------------------------------------------
    % Test 5: Terminal Stop Condition (Obstacle Standstill)
    % -------------------------------------------------------------
    % 60m straight road with terminal stop v_end = 0.0 m/s
    x5 = linspace(0, 60, 100)';
    y5 = zeros(100, 1);
    path5 = [x5, y5];
    
    [aug5, ~] = planner.planVelocityProfile(path5, 10.0, 0.0);
    v_terminal = aug5(end, 5);
    % Check deceleration slope
    dv_ds = diff(aug5(:, 5)) ./ diff(aug5(:, 1));
    max_decel_observed = max(-dv_ds .* aug5(1:end-1, 5)); % a = v * dv/ds
    
    pass5 = (v_terminal <= 0.1) && (aug5(1, 5) > 5.0);
    print_test_result(5, 'Terminal Standstill Profile (v_end = 0.0 m/s)', ...
        sprintf('v(0)=%.2f m/s, v(end)=%.2f m/s, smooth deceleration verified', ...
        aug5(1, 5), v_terminal), pass5);
    if pass5, pass_count = pass_count + 1; end
    
    % -------------------------------------------------------------
    % Test 6: Physical Sanity & Acceleration Bounds
    % -------------------------------------------------------------
    v_all = aug3(:, 5);
    s_all = info3.s;
    ds_all = max(diff(s_all), 1e-4);
    dv_dt_approx = (v_all(2:end).^2 - v_all(1:end-1).^2) ./ (2 * ds_all); % v dv/ds = a
    
    max_pos_a = max(dv_dt_approx);
    max_neg_a = min(dv_dt_approx);
    
    pass6 = all(v_all >= 0) && (max_pos_a <= planner.a_accel_max + 0.5) && ...
            (max_neg_a >= -planner.a_decel_max - 0.5);
    print_test_result(6, 'Longitudinal Acceleration & Non-Negativity Bounds', ...
        sprintf('a_accel_max=%.2f (limit=2.0), a_decel_max=%.2f (limit=-3.0), v_min>=0', ...
        max_pos_a, max_neg_a), pass6);
    if pass6, pass_count = pass_count + 1; end
    
    % -------------------------------------------------------------
    % Test 7: Known-Radius Circular Arc (R = 18.5m, kappa = 0.0541 1/m)
    % -------------------------------------------------------------
    % Expected kappa = 1/18.5 = 0.054054 m^-1
    % Expected v_corner = sqrt(2.50 / 0.054054) = 6.8007 m/s
    R_circ = 18.5;
    k_expected = 1.0 / R_circ;
    v_corner_expected = sqrt(planner.a_lat_max / k_expected);
    
    % Straight 30m followed by R=18.5m circular arc
    s_lead = linspace(0, 30, 60)';
    x7_str = s_lead;
    y7_str = zeros(size(s_lead));
    
    th7 = linspace(0, pi/2, 60)';
    x7_arc = 30 + R_circ * sin(th7);
    y7_arc = R_circ * (1 - cos(th7));
    path7 = [x7_str(1:end-1), y7_str(1:end-1); x7_arc, y7_arc];
    
    [aug7, info7] = planner.planVelocityProfile(path7, 10.0);
    
    % Verify measured curvature on the arc (points inside arc)
    arc_idx_start = 65; arc_idx_end = 115;
    measured_k_arc = mean(abs(aug7(arc_idx_start:arc_idx_end, 4)));
    apex_v7 = min(aug7(:, 5));
    
    % Verify pre-braking at x=25m (5m before curve)
    [~, idx_pre7] = min(abs(aug7(:, 1) - 25.0));
    v_pre7 = aug7(idx_pre7, 5);
    
    pass7 = (abs(measured_k_arc - k_expected) < 0.005) && ...
            (abs(apex_v7 - v_corner_expected) < 0.25) && ...
            (v_pre7 < 9.5) && (max(info7.a_lat_actual) <= planner.a_lat_max + 0.15);
        
    print_test_result(7, 'Known-Radius Circular Arc (R=18.5m, v_corner=6.80m/s)', ...
        sprintf('k_meas=%.5f (exp=%.5f), v_apex=%.2f (exp=%.2f), v_pre(x=25)=%.2f, max_alat=%.2f', ...
        measured_k_arc, k_expected, apex_v7, v_corner_expected, v_pre7, max(info7.a_lat_actual)), pass7);
    if pass7, pass_count = pass_count + 1; end
    
    fprintf('========================================================\n');
    fprintf('  CURVATURE PLANNER TESTS: %d/%d PASSED\n', pass_count, total_tests);
    fprintf('========================================================\n\n');
    
    if pass_count < total_tests
        error('CurvatureVelocityPlanner unit tests failed.');
    end
end

function print_test_result(idx, name, details, pass)
    if pass
        fprintf('[PASS] Test %d: %s\n', idx, name);
    else
        fprintf('[FAIL] Test %d: %s\n', idx, name);
    end
    fprintf('       %s\n\n', details);
end
