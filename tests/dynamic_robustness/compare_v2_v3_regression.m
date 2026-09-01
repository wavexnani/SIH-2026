function compare_v2_v3_regression()
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common', 'tests/dynamic_robustness');

v2_data = load('tests/dynamic_robustness/results/dynamic_robustness_benchmark_results.mat');
v3_data = load('tests/dynamic_robustness/results/dynamic_robustness_benchmark_v3_results.mat');

v2_sum = v2_data.summaries;
v3_sum = v3_data.summaries;

fprintf('\n=======================================================================================================================================\n');
fprintf('                                   STEP 10: REGRESSION PROTECTION ANALYSIS (V2 vs V3)                                                 \n');
fprintf('=======================================================================================================================================\n');
fprintf('%-28s | %-12s | %-12s | %-12s | %-12s | %-12s | %-12s | %-15s\n', ...
    'Scenario', 'V2 Scen-Free', 'V3 Scen-Free', 'V2 Min Clr', 'V3 Min Clr', 'V2 PlanPrev', 'V3 PlanPrev', 'Difference');
fprintf('---------------------------------------------------------------------------------------------------------------------------------------\n');

for i = 1:6
    s2 = v2_sum{i};
    s3 = v3_sum{i};
    
    diff_str = 'PERFECT MATCH';
    if abs(s2.scenario_collision_free_rate - s3.scenario_collision_free_rate) > 1e-4 || ...
       abs(s2.min_scenario_clearance - s3.min_scenario_clearance) > 1e-4 || ...
       abs(s2.planner_prevention_pct - s3.planner_prevention_pct) > 1e-4
        diff_str = 'VARIANCE DETECTED';
    end
    
    fprintf('%-28s | %-12.1f%% | %-12.1f%% | %-12.2f m | %-12.2f m | %-12.1f%% | %-12.1f%% | %-15s\n', ...
        s3.scenario_name, s2.scenario_collision_free_rate, s3.scenario_collision_free_rate, ...
        s2.min_scenario_clearance, s3.min_scenario_clearance, ...
        s2.planner_prevention_pct, s3.planner_prevention_pct, diff_str);
end
fprintf('=======================================================================================================================================\n\n');
end
