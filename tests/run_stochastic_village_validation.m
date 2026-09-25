function results = run_stochastic_village_validation()
    % RUN_STOCHASTIC_VILLAGE_VALIDATION Automated Validation Suite for PS26037
    %
    % Executes Tests A through J:
    %   Test A: Existing deterministic regression (Stage 5.1 multi-vehicle suite)
    %   Test B: Heterogeneous traffic run (car, bike, auto, pedestrian, cattle)
    %   Test C: Curved road geometry
    %   Test D: Uphill / downhill longitudinal road grade
    %   Test E: Potholes and physical road defects
    %   Test F: Bidirectional traffic (forward and oncoming)
    %   Test G: Pedestrian crossing and shoulder walking
    %   Test H: Cattle crossing and grazing
    %   Test I: Mixed unstructured village traffic environment
    %   Test J: Controlled stochasticity & multi-seed reproducibility verification
    
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
    fprintf('========================================================================================\n');
    fprintf('        SIH26037 STOCHASTIC UNSTRUCTURED VILLAGE TRAFFIC VALIDATION SUITE               \n');
    fprintf('========================================================================================\n\n');
    
    results = struct();
    all_tests_passed = true;
    
    % -------------------------------------------------------------------------
    % TEST A: Existing Deterministic Regression Baseline
    % -------------------------------------------------------------------------
    fprintf('>>> Running Test A: Existing Deterministic Regression Suite...\n');
    try
        [det_passed, ~] = stage5_multivehicle_validation();
        if det_passed
            fprintf('>>> RESULT Test A: PASSED (100%% Baseline Preservation)\n\n');
            results.TestA.status = 'PASS';
        else
            fprintf('>>> RESULT Test A: FAILED (Deterministic baseline regressions detected)\n\n');
            results.TestA.status = 'FAIL';
            all_tests_passed = false;
        end
    catch ME
        fprintf('>>> RESULT Test A: ERROR (%s)\n\n', ME.message);
        results.TestA.status = 'ERROR';
        all_tests_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test Cases Matrix for Tests B to I
    % -------------------------------------------------------------------------
    cases = {
        'TestB', 'stochastic_heterogeneous_traffic', 'Heterogeneous Traffic (Car, Bike, Auto, Ped, Cattle)';
        'TestC', 'stochastic_curved_road',           'Curved Village Road (Parametric Centerline & Bounds)';
        'TestD', 'stochastic_grade_road',            'Longitudinal Road Grade (Uphill/Downhill Dynamics)';
        'TestE', 'stochastic_pothole_road',          'Road Defects & Pothole Hazards';
        'TestF', 'stochastic_bidirectional_traffic', 'Dense Bidirectional Traffic Flow';
        'TestG', 'stochastic_pedestrian_crossing',   'Pedestrian Crossing & Shoulder Walking Behavior';
        'TestH', 'stochastic_cattle_crossing',       'Livestock Crossing & Roadside Grazing Behavior';
        'TestI', 'stochastic_village_mixed',         'Full Mixed Unstructured Village Road Environment'
    };
    
    for c = 1:size(cases, 1)
        test_id = cases{c, 1};
        scen_name = cases{c, 2};
        desc_str = cases{c, 3};
        
        fprintf('----------------------------------------------------------------------------------------\n');
        fprintf('>>> Running %s: %s [%s]\n', test_id, desc_str, scen_name);
        fprintf('----------------------------------------------------------------------------------------\n');
        
        try
            [passed, metrics, history, ~] = stage5_multivehicle_coordination( ...
                'scenario', scen_name, 'verbose', false, 'max_steps', 100, ...
                'seed', 42, 'uncertainty_mode', 'ideal');
            
            % Compute Intent Transition Counts
            intent_transitions = 0;
            for k = 2:length(history.macro_intent)
                if ~strcmp(history.macro_intent{k}, history.macro_intent{k-1})
                    intent_transitions = intent_transitions + 1;
                end
            end
            
            % Check safety invariants
            test_pass = (metrics.collision_steps == 0) && (metrics.bounds_steps == 100);
            
            results.(test_id).scenario = scen_name;
            results.(test_id).description = desc_str;
            results.(test_id).passed = test_pass;
            results.(test_id).outcome = metrics.outcome;
            results.(test_id).metrics = metrics;
            results.(test_id).intent_transitions = intent_transitions;
            
            fprintf('  Collision Steps         : %d / 100\n', metrics.collision_steps);
            fprintf('  Road-Bound Compliance   : %d / 100 (%.1f%%)\n', metrics.bounds_steps, (metrics.bounds_steps/100)*100);
            fprintf('  Minimum Clearance       : %+.2f m\n', metrics.min_clr);
            fprintf('  Minimum TTC             : %.2f s\n', metrics.min_ttc);
            fprintf('  Distance Traveled       : %.2f m\n', metrics.dist_traveled);
            fprintf('  Final Speed             : %.2f m/s\n', metrics.final_v);
            fprintf('  Hard QP Feasible Steps  : %d / 100\n', metrics.hard_qp_count);
            fprintf('  Safety Filter Overrides : %d steps\n', metrics.safety_rejected_count);
            fprintf('  Emergency Braking Steps : %d steps\n', metrics.emergency_braking_count);
            fprintf('  Mean Planner Latency    : %.2f ms/step\n', metrics.mean_solve_time);
            fprintf('  Intent Transitions      : %d\n', intent_transitions);
            fprintf('  Traffic Count Spawned   : %d\n', metrics.traffic_count);
            fprintf('  Active Agents End       : %d\n', metrics.active_agent_count);
            if isfield(metrics, 'agent_type_counts')
                c_counts = metrics.agent_type_counts;
                fprintf('  Class Breakdown         : Cars=%d, Bikes=%d, Autos=%d, Peds=%d, Cattle=%d\n', ...
                    c_counts.car, c_counts.bike, c_counts.auto, c_counts.pedestrian, c_counts.cattle);
            end
            fprintf('  Pedestrian Crossings    : %d\n', metrics.pedestrian_crossings);
            fprintf('  Cattle Crossings        : %d\n', metrics.cattle_crossings);
            fprintf('  Outcome Status          : %s\n', metrics.outcome);
            
            if test_pass
                fprintf('>>> RESULT %s: [ PASS ]\n\n', test_id);
            else
                fprintf('>>> RESULT %s: [ FAIL ] (Safety constraints violated)\n\n', test_id);
                all_tests_passed = false;
            end
        catch ME
            fprintf('>>> RESULT %s: [ ERROR ] (%s)\n', test_id, ME.message);
            for stk = 1:length(ME.stack)
                fprintf('    in %s (line %d)\n', ME.stack(stk).file, ME.stack(stk).line);
            end
            fprintf('\n');
            results.(test_id).status = 'ERROR';
            all_tests_passed = false;
        end
    end
    
    % -------------------------------------------------------------------------
    % TEST J: Multi-Seed Reproducibility & Stochastic Realization Variation
    % -------------------------------------------------------------------------
    fprintf('----------------------------------------------------------------------------------------\n');
    fprintf('>>> Running Test J: Stochastic Reproducibility (Seed 42) vs Variation (Seed 43)\n');
    fprintf('----------------------------------------------------------------------------------------\n');
    try
        [~, ~, h42_a] = stage5_multivehicle_coordination('scenario', 'stochastic_heterogeneous_traffic', ...
            'seed', 42, 'max_steps', 50, 'verbose', false);
        [~, ~, h42_b] = stage5_multivehicle_coordination('scenario', 'stochastic_heterogeneous_traffic', ...
            'seed', 42, 'max_steps', 50, 'verbose', false);
        [~, ~, h43]   = stage5_multivehicle_coordination('scenario', 'stochastic_heterogeneous_traffic', ...
            'seed', 43, 'max_steps', 50, 'verbose', false);
        
        diff_reproducible = max(abs(h42_a.ego_x - h42_b.ego_x)) + max(abs(h42_a.ego_y - h42_b.ego_y));
        diff_variation    = max(abs(h42_a.ego_x - h43.ego_x)) + max(abs(h42_a.ego_y - h43.ego_y));
        
        fprintf('  Seed 42 Exact Reproducibility Difference : %.6f m (Expected: 0.000000 m)\n', diff_reproducible);
        fprintf('  Seed 42 vs Seed 43 Realization Shift     : %.4f m (Expected: > 1.0000 m)\n', diff_variation);
        
        pass_j = (diff_reproducible < 1e-9) && (diff_variation > 1.0);
        results.TestJ.diff_reproducible = diff_reproducible;
        results.TestJ.diff_variation = diff_variation;
        results.TestJ.passed = pass_j;
        
        if pass_j
            fprintf('>>> RESULT Test J: [ PASS ] (Exact Seed Reproducibility + Distinct Stochastic Variations)\n\n');
        else
            fprintf('>>> RESULT Test J: [ FAIL ] (RNG stream independence violated)\n\n');
            all_tests_passed = false;
        end
    catch ME
        fprintf('>>> RESULT Test J: [ ERROR ] (%s)\n\n', ME.message);
        results.TestJ.status = 'ERROR';
        all_tests_passed = false;
    end
    
    fprintf('========================================================================================\n');
    if all_tests_passed
        fprintf('      ALL TESTS A THROUGH J PASSED! FULL STOCHASTIC ENVIRONMENT ACCEPTED!       \n');
    else
        fprintf('      ONE OR MORE TESTS FAILED! CHECK INDIVIDUAL BREAKDOWN ABOVE.               \n');
    end
    fprintf('========================================================================================\n\n');
end
