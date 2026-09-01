function [all_passed, results] = stage5_multivehicle_validation()
    % STAGE5_MULTIVEHICLE_VALIDATION Stage 5.1 Validation Checkpoint Audit Harness
    %
    % Executes closed-loop safety verification across all 3 multi-vehicle scenarios
    % with strict Stage 4 invariants: 100% Road-Bound Compliance (150/150 steps),
    % 0 Collisions, Positive Minimum Clearance, and Zero Emergency Braking.
    
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir), projectRoot = pwd; else projectRoot = fileparts(current_dir); end
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'vehicle'));
    addpath(fullfile(projectRoot, 'core'));
    addpath(fullfile(projectRoot, 'environment'));
    addpath(fullfile(projectRoot, 'stages'));
    
    scenarios = {
        'multi_vehicle_following', ...
        'multi_vehicle_yield_overtake', ...
        'multi_vehicle_oncoming_conflict'
    };
    
    fprintf('\n');
    fprintf('========================================================================================\n');
    fprintf('           STAGE 5.1 MULTI-VEHICLE COORDINATION VALIDATION CHECKPOINT                  \n');
    fprintf('========================================================================================\n\n');
    
    all_passed = true;
    results = struct();
    
    for s = 1:length(scenarios)
        scen_name = scenarios{s};
        fprintf('----------------------------------------------------------------------------------------\n');
        fprintf('  AUDITING SCENARIO [%d/%d]: %s\n', s, length(scenarios), scen_name);
        fprintf('----------------------------------------------------------------------------------------\n');
        
        [passed, metrics, history] = stage5_multivehicle_coordination( ...
            'scenario', scen_name, 'verbose', false, 'max_steps', 150);
        
        % Independently retain the complete Stage 5.1 acceptance gate so the
        % audit remains valid even if the scenario runner is changed later.
        strict_pass = (metrics.collision_steps == 0) && ...
                      (metrics.bounds_steps == 150) && ...
                      (metrics.min_clr > 0.0) && ...
                      (metrics.emergency_braking_count == 0);
                      
        results.(scen_name).passed = strict_pass;
        results.(scen_name).metrics = metrics;
        
        fprintf('  Collision Steps:        %d / 150\n', metrics.collision_steps);
        fprintf('  Road-Bound Compliance:  %d / 150 (%.1f%%)\n', metrics.bounds_steps, (metrics.bounds_steps/150)*100);
        fprintf('  Minimum Clearance:      %+.2f m\n', metrics.min_clr);
        fprintf('  Distance Traveled:      %.2f m\n', metrics.dist_traveled);
        fprintf('  Final Speed:            %.2f m/s\n', metrics.final_v);
        fprintf('  Emergency Braking:      %d steps\n', metrics.emergency_braking_count);
        fprintf('  Avg Solver Time:        %.2f ms/step\n', metrics.mean_solve_time);
        
        if strict_pass
            fprintf('  SCENARIO AUDIT STATUS:  [ PASS ] (100%% Compliance & Zero Collisions)\n\n');
        else
            fprintf('  SCENARIO AUDIT STATUS:  [ FAIL ]\n\n');
            all_passed = false;
        end
    end
    
    % Update stage5_multivehicle_coordination criterion for future direct calls
    % (handled in stage5_multivehicle_coordination.m)
    
    fprintf('========================================================================================\n');
    if all_passed
        fprintf('  STAGE 5.1 VALIDATION CHECKPOINT: [ OVERALL PASS ] - ALL INVARIANTS SATISFIED!\n');
    else
        fprintf('  STAGE 5.1 VALIDATION CHECKPOINT: [ OVERALL FAIL ] - INVARIANTS VIOLATED!\n');
    end
    fprintf('========================================================================================\n\n');
end
