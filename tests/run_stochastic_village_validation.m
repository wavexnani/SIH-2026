function results = run_stochastic_village_validation()
    % RUN_STOCHASTIC_VILLAGE_VALIDATION Automated Validation Suite for PS26037
    %
    % Validates the unified continuous dense stochastic unstructured village road environment:
    %   Test 1: Deterministic baseline regression (multi_vehicle_yield_overtake)
    %   Test 2: Unified continuous dense stochastic village environment (150 steps, all 5 classes)
    %   Test 3: Traffic density scalability (LOW vs MEDIUM vs HIGH)
    %   Test 4: Controlled stochasticity & multi-seed reproducibility verification (Seed 42 vs 43)
    %   Test 5: Local perception sensing window & observer integrity
    %   Test 6: Simulation telemetry export for web viewer
    
    project_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(project_root, 'tests'));
    addpath(fullfile(project_root, 'stages'));
    addpath(fullfile(project_root, 'planning'));
    addpath(fullfile(project_root, 'config'));
    addpath(fullfile(project_root, 'vehicle'));
    addpath(fullfile(project_root, 'core'));
    addpath(fullfile(project_root, 'environment'));
    addpath(fullfile(project_root, 'scripts'));
    
    fprintf('\n');
    fprintf('════════════════════════════════════════════════════════════════════════════════════════\n');
    fprintf('        SIH26037 CONTINUOUS STOCHASTIC VILLAGE TRAFFIC VALIDATION SUITE                 \n');
    fprintf('════════════════════════════════════════════════════════════════════════════════════════\n\n');
    
    results = struct();
    all_tests_passed = true;
    
    % -------------------------------------------------------------------------
    % TEST 1: Deterministic Regression Baseline (multi_vehicle_yield_overtake)
    % -------------------------------------------------------------------------
    fprintf('>>> Running Test 1: Deterministic Baseline Regression (multi_vehicle_yield_overtake)...\n');
    try
        [det_passed, det_metrics, ~, ~] = stage5_multivehicle_coordination( ...
            'scenario', 'multi_vehicle_yield_overtake', 'verbose', false, 'max_steps', 150);
        
        test1_ok = det_passed && (det_metrics.collision_steps == 0) && (det_metrics.bounds_steps == 150);
        results.Test1.passed = test1_ok;
        results.Test1.metrics = det_metrics;
        
        if test1_ok
            fprintf('>>> RESULT Test 1: PASSED (0 collisions, 150/150 bounds compliance, clr=+%.2fm)\n\n', det_metrics.min_clr);
        else
            fprintf('>>> RESULT Test 1: FAILED (Collisions=%d, Bounds=%d)\n\n', det_metrics.collision_steps, det_metrics.bounds_steps);
            all_tests_passed = false;
        end
    catch ME
        fprintf('>>> RESULT Test 1: ERROR (%s)\n\n', ME.message);
        results.Test1.passed = false;
        results.Test1.error = ME.message;
        all_tests_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % TEST 2: Unified Continuous Dense Stochastic Village Environment (150 steps)
    % -------------------------------------------------------------------------
    fprintf('>>> Running Test 2: Unified Continuous Dense Stochastic Village Environment (150 steps)...\n');
    try
        [stoch_passed, stoch_metrics, stoch_hist, stoch_simLog] = stage5_multivehicle_coordination( ...
            'scenario', 'stochastic_village_environment', 'seed', 42, 'density', 'MEDIUM', ...
            'max_steps', 150, 'verbose', false);
        
        % Invariants verification:
        % 1. Zero collisions
        % 2. 100% road bounds compliance
        % 3. All 5 heterogeneous classes present
        % 4. Peak simultaneous observed >= 5
        % 5. Zero unhandled QP crashes
        types = stoch_metrics.agent_type_counts;
        has_all_classes = (types.car > 0) && (types.bike > 0) && (types.auto > 0) && ...
                          (types.pedestrian > 0) && (types.cattle > 0);
        has_dense_simul = (stoch_metrics.max_simultaneous_observed >= 5);
        no_collision = (stoch_metrics.collision_steps == 0);
        in_bounds = (stoch_metrics.bounds_steps == 150);
        
        test2_ok = no_collision && in_bounds && has_all_classes && has_dense_simul;
        results.Test2.passed = test2_ok;
        results.Test2.metrics = stoch_metrics;
        
        % Find a representative multi-object frame
        rep_frame_idx = 25;
        rep_obs = stoch_simLog{rep_frame_idx}.observation.agents;
        
        fprintf('  Collision Steps                     : %d / 150\n', stoch_metrics.collision_steps);
        fprintf('  Road Bounds Compliance              : %d / 150\n', stoch_metrics.bounds_steps);
        fprintf('  Min Clearance                       : +%.3f m\n', stoch_metrics.min_clr);
        fprintf('  Peak Simultaneous Active (World)    : %d agents\n', stoch_metrics.max_simultaneous_active);
        fprintf('  Peak Simultaneous Observed (Ego)    : %d agents\n', stoch_metrics.max_simultaneous_observed);
        fprintf('  Total Spawned Traffic Participants  : %d\n', stoch_metrics.traffic_count);
        fprintf('     - Cars                           : %d\n', types.car);
        fprintf('     - Bikes                          : %d\n', types.bike);
        fprintf('     - Autos                          : %d\n', types.auto);
        fprintf('     - Pedestrians                    : %d\n', types.pedestrian);
        fprintf('     - Cattle                         : %d\n', types.cattle);
        fprintf('  Pedestrian Crossings                : %d\n', stoch_metrics.pedestrian_crossings);
        fprintf('  Cattle Crossings                    : %d\n', stoch_metrics.cattle_crossings);
        fprintf('  Vehicles Passed Ego                 : %d\n', stoch_metrics.vehicles_passed_ego);
        fprintf('  Simultaneous Agents in Frame %d      : %d observable agents\n', rep_frame_idx, length(rep_obs));
        
        if test2_ok
            fprintf('>>> RESULT Test 2: PASSED (Dense Heterogeneous Environment Operational)\n\n');
        else
            fprintf('>>> RESULT Test 2: FAILED (Criteria not fully met)\n\n');
            all_tests_passed = false;
        end
    catch ME
        fprintf('>>> RESULT Test 2: ERROR (%s)\n\n', ME.message);
        results.Test2.passed = false;
        results.Test2.error = ME.message;
        all_tests_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % TEST 3: Traffic Density Scalability (LOW vs HIGH)
    % -------------------------------------------------------------------------
    fprintf('>>> Running Test 3: Traffic Density Scalability (LOW vs HIGH)...\n');
    try
        [~, m_low] = stage5_multivehicle_coordination('scenario', 'stochastic_village_environment', ...
            'seed', 42, 'density', 'LOW', 'max_steps', 50, 'verbose', false);
        [~, m_high] = stage5_multivehicle_coordination('scenario', 'stochastic_village_environment', ...
            'seed', 42, 'density', 'HIGH', 'max_steps', 50, 'verbose', false);
        
        test3_ok = (m_low.max_simultaneous_active < m_high.max_simultaneous_active) && ...
                   (m_low.collision_steps == 0) && (m_high.collision_steps == 0);
        results.Test3.passed = test3_ok;
        results.Test3.low_peak = m_low.max_simultaneous_active;
        results.Test3.high_peak = m_high.max_simultaneous_active;
        
        fprintf('  LOW Density Peak Active             : %d agents (Total Spawned: %d)\n', m_low.max_simultaneous_active, m_low.traffic_count);
        fprintf('  HIGH Density Peak Active            : %d agents (Total Spawned: %d)\n', m_high.max_simultaneous_active, m_high.traffic_count);
        
        if test3_ok
            fprintf('>>> RESULT Test 3: PASSED (Density scales dynamically as configured)\n\n');
        else
            fprintf('>>> RESULT Test 3: FAILED\n\n');
            all_tests_passed = false;
        end
    catch ME
        fprintf('>>> RESULT Test 3: ERROR (%s)\n\n', ME.message);
        results.Test3.passed = false;
        results.Test3.error = ME.message;
        all_tests_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % TEST 4: Controlled Stochasticity & Reproducibility (Seed 42 vs 43)
    % -------------------------------------------------------------------------
    fprintf('>>> Running Test 4: Controlled Seed Reproducibility & Variation...\n');
    try
        [~, ~, h42_a] = stage5_multivehicle_coordination('scenario', 'stochastic_village_environment', ...
            'seed', 42, 'max_steps', 50, 'verbose', false);
        [~, ~, h42_b] = stage5_multivehicle_coordination('scenario', 'stochastic_village_environment', ...
            'seed', 42, 'max_steps', 50, 'verbose', false);
        [~, ~, h43] = stage5_multivehicle_coordination('scenario', 'stochastic_village_environment', ...
            'seed', 43, 'max_steps', 50, 'verbose', false);
        
        diff_replay = max(abs(h42_a.ego_x - h42_b.ego_x));
        diff_seed = max(abs(h42_a.ego_x - h43.ego_x));
        
        test4_ok = (diff_replay == 0.0) && (diff_seed > 0.01);
        results.Test4.passed = test4_ok;
        results.Test4.replay_diff = diff_replay;
        results.Test4.seed_variation = diff_seed;
        
        fprintf('  Seed 42 Replay Difference           : %.6f m (Bit-Exact Identity)\n', diff_replay);
        fprintf('  Seed 42 vs Seed 43 Divergence       : %.4f m (Statistically Distinct Realization)\n', diff_seed);
        
        if test4_ok
            fprintf('>>> RESULT Test 4: PASSED (Dedicated RNG Stream Reproducibility Verified)\n\n');
        else
            fprintf('>>> RESULT Test 4: FAILED\n\n');
            all_tests_passed = false;
        end
    catch ME
        fprintf('>>> RESULT Test 4: ERROR (%s)\n\n', ME.message);
        results.Test4.passed = false;
        results.Test4.error = ME.message;
        all_tests_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % TEST 5: Local Perception Sensing Window & Observer Integrity
    % -------------------------------------------------------------------------
    fprintf('>>> Running Test 5: Local Perception Window & Observer Integrity...\n');
    try
        [~, ~, ~, simLog] = stage5_multivehicle_coordination('scenario', 'stochastic_village_environment', ...
            'seed', 42, 'max_steps', 50, 'verbose', false);
        
        % Verify no observed object has dx > 50.0m or dx < -15.0m
        range_violations = 0;
        total_observed_checks = 0;
        for k = 1:length(simLog)
            ego_pt = simLog{k}.groundTruth.ego;
            obs_agents = simLog{k}.observation.agents;
            for a = 1:length(obs_agents)
                total_observed_checks = total_observed_checks + 1;
                dx = obs_agents(a).x - ego_pt.x;
                d_rel = hypot(dx, obs_agents(a).y - ego_pt.y);
                if dx > 50.5 || dx < -15.5 || d_rel > 50.5
                    range_violations = range_violations + 1;
                end
            end
        end
        
        test5_ok = (range_violations == 0) && (total_observed_checks > 100);
        results.Test5.passed = test5_ok;
        results.Test5.total_checks = total_observed_checks;
        results.Test5.range_violations = range_violations;
        
        fprintf('  Total Observation Checks            : %d\n', total_observed_checks);
        fprintf('  Out-Of-Range Perception Violations  : %d\n', range_violations);
        
        if test5_ok
            fprintf('>>> RESULT Test 5: PASSED (Local perception bounds strictly enforced; zero cheating)\n\n');
        else
            fprintf('>>> RESULT Test 5: FAILED\n\n');
            all_tests_passed = false;
        end
    catch ME
        fprintf('>>> RESULT Test 5: ERROR (%s)\n\n', ME.message);
        results.Test5.passed = false;
        results.Test5.error = ME.message;
        all_tests_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % TEST 6: Web Viewer Telemetry Export
    % -------------------------------------------------------------------------
    fprintf('>>> Running Test 6: Web Viewer Telemetry Export...\n');
    try
        json_path = fullfile(project_root, 'web_viewer', 'data', 'stochastic_scenario.json');
        run_stochastic_hero_export();
        
        test6_ok = exist(json_path, 'file') == 2;
        if test6_ok
            finfo = dir(json_path);
            fprintf('  Exported File                       : %s (%.1f KB)\n', finfo.name, finfo.bytes / 1024);
            fprintf('>>> RESULT Test 6: PASSED (JSON artifact generated for HTML viewer)\n\n');
        else
            fprintf('>>> RESULT Test 6: FAILED (Output file not found)\n\n');
            all_tests_passed = false;
        end
        results.Test6.passed = test6_ok;
    catch ME
        fprintf('>>> RESULT Test 6: ERROR (%s)\n\n', ME.message);
        results.Test6.passed = false;
        results.Test6.error = ME.message;
        all_tests_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % FINAL SUMMARY
    % -------------------------------------------------------------------------
    fprintf('════════════════════════════════════════════════════════════════════════════════════════\n');
    fprintf('                   VALIDATION SUITE FINAL OUTCOME: ');
    if all_tests_passed
        fprintf('[ ALL 6 TESTS PASSED ]\n');
    else
        fprintf('[ FAILURES DETECTED ]\n');
    end
    fprintf('════════════════════════════════════════════════════════════════════════════════════════\n\n');
end
