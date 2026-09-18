function results = run_regression_suite()
    % RUN_REGRESSION_SUITE Runs all core regression tests to ensure baseline preservation.
    
    addpath('tests', 'planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch');
    
    suite = {
        'test_curvature_velocity_planner', @test_curvature_velocity_planner;
        'test_steering_filter',            @test_steering_filter;
        'test_freespace_regression',       @test_freespace_regression;
        'test_unstructured_road',          @test_unstructured_road;
        'test_phase15_overtake_gate',      @test_phase15_overtake_gate
    };
    
    fprintf('\n========================================================\n');
    fprintf('         RUNNING SIH PS26037 REGRESSION SUITE           \n');
    fprintf('========================================================\n\n');
    
    all_passed = true;
    results = struct();
    
    for i = 1:size(suite, 1)
        name = suite{i, 1};
        fn = suite{i, 2};
        fprintf('\n>>> Executing Test: %s ...\n', name);
        t_start = tic;
        try
            fn();
            elapsed = toc(t_start);
            fprintf('>>> RESULT: %s PASSED (%.2f s)\n', name, elapsed);
            results.(name) = 'PASS';
        catch ME
            elapsed = toc(t_start);
            fprintf('>>> RESULT: %s FAILED (%.2f s): %s\n', name, elapsed, ME.message);
            fprintf('Stack trace:\n');
            for k = 1:length(ME.stack)
                fprintf('  in %s (line %d)\n', ME.stack(k).file, ME.stack(k).line);
            end
            results.(name) = 'FAIL';
            all_passed = false;
        end
    end
    
    fprintf('\n========================================================\n');
    if all_passed
        fprintf('  ALL REGRESSION TESTS PASSED! BASELINE IS PRESERVED.  \n');
    else
        fprintf('  REGRESSION TESTS FAILED! CHECK OUTPUT ABOVE.         \n');
    end
    fprintf('========================================================\n\n');
end
