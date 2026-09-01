function run_stage51_validation()
    % RUN_STAGE51_VALIDATION Execute the complete Stage 5.1 acceptance chain.
    %
    % This runner intentionally fails fast. A Stage 5.1 PASS requires every
    % unit test, all three 150-step multi-vehicle scenarios, and the retained
    % Stage 4 regression scenarios to complete without assertion failures.

    project_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(project_root, 'tests'));
    addpath(fullfile(project_root, 'stages'));
    addpath(fullfile(project_root, 'planning'));
    addpath(fullfile(project_root, 'config'));
    addpath(fullfile(project_root, 'vehicle'));
    addpath(fullfile(project_root, 'core'));
    addpath(fullfile(project_root, 'environment'));

    fprintf('\n=== STAGE 5.1 COMPLETE VALIDATION ===\n');
    test_stage5_perception();
    test_stage5_decision();

    [all_passed, results] = stage5_multivehicle_validation();
    assert(all_passed, 'Stage 5.1 multi-vehicle acceptance criteria failed.');

    regression_scenarios = {'passable_moderate', 'passable_marginal', ...
                            'impassable_center', 'multi_obstacle_sequence'};
    for i = 1:numel(regression_scenarios)
        [passed, metrics] = stage4_cacrc_safety( ...
            'scenario', regression_scenarios{i}, 'verbose', false);
        assert(passed, 'Stage 4 regression failed: %s', regression_scenarios{i});
        fprintf('[PASS] Stage 4 regression: %s\n', regression_scenarios{i});
    end

    fprintf('\n[PASS] STAGE 5.1 COMPLETE VALIDATION\n');
end
