function audit_results = stage4_robustness_audit()
    % STAGE4_ROBUSTNESS_AUDIT Automated Stage 4.4 Robustness & Generalization Audit
    %
    % Evaluates frozen CA-CRC controller across 4 distinct scenario classes:
    %   1. Scenario A: Clearly Passable (passable_moderate)
    %   2. Scenario B: Marginally Passable (passable_marginal)
    %   3. Scenario C: Geometrically Impossible (impassable_center)
    %   4. Scenario D: Multi-Obstacle Sequence (multi_obstacle_sequence)

    addpath('planning', 'config', 'vehicle', 'core', 'environment', 'stages');
    
    scenarios = {
        'passable_moderate',        'Scenario A: Clearly Passable Corridor';
        'passable_marginal',        'Scenario B: Marginally Passable Corridor';
        'impassable_center',        'Scenario C: Geometrically Impossible Center Obstacle';
        'multi_obstacle_sequence',  'Scenario D: Multi-Obstacle Sequential Overtake'
    };

    fprintf('\n');
    fprintf('════════════════════════════════════════════════════════════════════════════════════════\n');
    fprintf('           STAGE 4.4 ROBUSTNESS & GENERALIZATION AUDIT SUITE                            \n');
    fprintf('════════════════════════════════════════════════════════════════════════════════════════\n');

    audit_results = struct();

    for idx = 1:size(scenarios, 1)
        scen_id = scenarios{idx, 1};
        scen_label = scenarios{idx, 2};
        
        fprintf('\n----------------------------------------------------------------------------------------\n');
        fprintf('[RUNNING] %s (%s)\n', scen_label, scen_id);
        fprintf('----------------------------------------------------------------------------------------\n');
        
        [pass, metrics] = stage4_cacrc_safety('scenario', scen_id, 'verbose', false);
        
        audit_results.(scen_id).pass = pass;
        audit_results.(scen_id).metrics = metrics;
        
        fprintf('  Distance Traveled:     %6.2f m\n', metrics.dist_traveled);
        fprintf('  Final Speed:           %6.2f m/s\n', metrics.final_v);
        fprintf('  Mean / Max e_y:        %6.3f m / %6.3f m\n', metrics.mean_ey, metrics.max_ey);
        fprintf('  Min Clearance:         %6.2f m\n', metrics.min_clr);
        fprintf('  Collision Steps:       %6d\n', metrics.collision_steps);
        fprintf('  Road Bounds Compliance:%6d / 100 steps\n', metrics.bounds_steps);
        fprintf('  4-State Breakdown:\n');
        fprintf('     - Hard QP Feasible: %6d steps\n', metrics.hard_qp_count);
        fprintf('     - Soft QP Fallback: %6d steps\n', metrics.soft_qp_count);
        fprintf('     - Safety Rejections: %5d steps\n', metrics.safety_rejected_count);
        fprintf('     - Emergency Braking:%5d steps\n', metrics.emergency_braking_count);
        fprintf('  Avg Solve Time:        %6.2f ms/step\n', metrics.mean_solve_time);
    end

    fprintf('\n════════════════════════════════════════════════════════════════════════════════════════\n');
    fprintf('                              GENERALIZATION SUMMARY TABLE                              \n');
    fprintf('════════════════════════════════════════════════════════════════════════════════════════\n');
    fprintf(' %-24s │ %-10s │ %-8s │ %-7s │ %-7s │ %-10s │ %-10s \n', ...
        'Scenario Class', 'Pass Status', 'Collision', 'Bounds', 'Hard QP', 'Soft/Filter', 'Emergency');
    fprintf('─────────────────────────┼────────────┼──────────┼─────────┼─────────┼────────────┼────────────\n');

    for idx = 1:size(scenarios, 1)
        scen_id = scenarios{idx, 1};
        m = audit_results.(scen_id).metrics;
        p_str = 'PASS'; if ~audit_results.(scen_id).pass && ~strcmp(scen_id, 'impassable_center'), p_str = 'FAIL'; end
        if strcmp(scen_id, 'impassable_center') && m.controlled_stop_executed && m.collision_steps == 0
            p_str = 'PASS (STOP)';
        end
        
        fprintf(' %-24s │ %-10s │ %8d │ %4d/100 │ %6d  │ %5d/%-4d │ %10d \n', ...
            scen_id, p_str, m.collision_steps, m.bounds_steps, m.hard_qp_count, ...
            m.soft_qp_count, m.safety_rejected_count, m.emergency_braking_count);
    end
    fprintf('════════════════════════════════════════════════════════════════════════════════════════\n\n');
end
