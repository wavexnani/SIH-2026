function diag_robustness_cases()
    % DIAG_ROBUSTNESS_CASES Investigate the 2 robustness emergency braking cases
    addpath('stages', 'planning', 'config', 'vehicle', 'core', 'environment');
    
    fprintf('\n========================================================================================\n');
    fprintf('           INVESTIGATING ROBUSTNESS EDGE CASES (RUNS 13 & 14)                          \n');
    fprintf('========================================================================================\n\n');

    % Run 13: Ego Speed +10%
    fprintf('----------------------------------------------------------------------------------------\n');
    fprintf('  RUN 13 DIAGNOSTIC: Scenario 3 (Oncoming Conflict) | Ego Initial Speed +10%% (v0 = 5.5 m/s)\n');
    fprintf('----------------------------------------------------------------------------------------\n');
    [~, ~, h13] = stage5_multivehicle_coordination(...
        'scenario', 'multi_vehicle_oncoming_conflict', ...
        'verbose', false, ...
        'ego_v', 5.5, ...
        'gap_offset', 0.0);

    em_steps_13 = find(h13.solver_status == 0);
    for idx = 1:length(em_steps_13)
        k = em_steps_13(idx);
        fprintf('  Step %3d (t=%.2fs): x=%.2fm, y=%.2fm, v=%.2fm/s, Intent=%s, FilterActive=%d, Reason=%s\n', ...
            k, h13.t(k), h13.ego_x(k), h13.ego_y(k), h13.ego_v(k), h13.macro_intent{k}, h13.filter_active(k), h13.filter_reason{k});
    end
    fprintf('  Summary Run 13: Collisions=%d | Bounds=%d/150 | MinClearance=%+.2fm\n\n', ...
        sum(h13.is_collision), sum(h13.inside_bounds), min(h13.min_clearance));

    % Run 14: Initial Gap -2m
    fprintf('----------------------------------------------------------------------------------------\n');
    fprintf('  RUN 14 DIAGNOSTIC: Scenario 3 (Oncoming Conflict) | Initial Obstacle Gap -2.0m (dx0 = 23m)\n');
    fprintf('----------------------------------------------------------------------------------------\n');
    [~, ~, h14] = stage5_multivehicle_coordination(...
        'scenario', 'multi_vehicle_oncoming_conflict', ...
        'verbose', false, ...
        'ego_v', 5.0, ...
        'gap_offset', -2.0);

    em_steps_14 = find(h14.solver_status == 0);
    for idx = 1:length(em_steps_14)
        k = em_steps_14(idx);
        fprintf('  Step %3d (t=%.2fs): x=%.2fm, y=%.2fm, v=%.2fm/s, Intent=%s, FilterActive=%d, Reason=%s\n', ...
            k, h14.t(k), h14.ego_x(k), h14.ego_y(k), h14.ego_v(k), h14.macro_intent{k}, h14.filter_active(k), h14.filter_reason{k});
    end
    fprintf('  Summary Run 14: Collisions=%d | Bounds=%d/150 | MinClearance=%+.2fm\n\n', ...
        sum(h14.is_collision), sum(h14.inside_bounds), min(h14.min_clearance));
end
