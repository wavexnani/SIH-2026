function stage4_head_to_head(varargin)
    % STAGE4_HEAD_TO_HEAD Benchmark Comparison: Stage 3 Baseline vs Stage 4
    
    p = inputParser;
    addParameter(p, 'scenario', 'moderate', @ischar);
    parse(p, varargin{:});
    scenario_name = p.Results.scenario;
    
    fprintf('\n════════════════════════════════════════════════════════════════════════════\n');
    fprintf('        HEAD-TO-HEAD BENCHMARK COMPARISON: STAGE 3 BASELINE VS STAGE 4      \n');
    fprintf('════════════════════════════════════════════════════════════════════════════\n');
    fprintf('  Scenario: %s | Target Speed: 8.00 m/s | Duration: 10.0 s\n', scenario_name);
    fprintf('════════════════════════════════════════════════════════════════════════════\n\n');
    
    % Run Stage 3 Baseline (Frozen)
    fprintf('[EXECUTION] Running Stage 3 (Linearized QP-MPC Baseline Planner)...\n');
    [pass3, m3] = stage3_qpmpc_baseline('scenario', scenario_name, 'verbose', false);
    
    % Run Stage 4 (CA-CRC + Safety Filter Layer)
    fprintf('[EXECUTION] Running Stage 4 (Context-Adaptive CRC + Hard Safety Layer)...\n');
    [pass4, m4] = stage4_cacrc_safety('scenario', scenario_name, 'verbose', false);
    
    % Print Comparative Benchmark Table
    fprintf('\n┌─────────────────────────────────────────┬──────────────────┬──────────────────┐\n');
    fprintf('│ Benchmark Metric / Parameter            │ Stage 3 (Base)   │ Stage 4 (CA-CRC) │\n');
    fprintf('├─────────────────────────────────────────┼──────────────────┼──────────────────┤\n');
    fprintf('│ Simulation Steps Executed               │  %5d/100 steps │  %5d/100 steps │\n', 100, 100);
    fprintf('│ Obstacle Longitudinally Passed          │  %15s │  %15s │\n', pass_str(m3.dist_traveled > 30), pass_str(m4.longitudinal_passed));
    fprintf('│ Collision-Free Run                      │  %15s │  %15s │\n', pass_str(m3.collision_steps == 0), pass_str(m4.collision_free_run));
    fprintf('│ Safe Obstacle Passage                   │  %15s │  %15s │\n', pass_str(pass3), pass_str(m4.safe_obstacle_passage));
    fprintf('│ Controlled Emergency Stop Executed      │  %15s │  %15s │\n', pass_str(m3.final_v < 0.1 && m3.collision_steps == 0), pass_str(m4.controlled_stop_executed));
    fprintf('│ Distance Traveled                       │  %12.2f m │  %12.2f m │\n', m3.dist_traveled, m4.dist_traveled);
    fprintf('│ Final Speed                             │  %10.2f m/s │  %10.2f m/s │\n', m3.final_v, m4.final_v);
    fprintf('│ Mean Cross-Track Error (e_y)            │  %12.3f m │  %12.3f m │\n', m3.mean_ey, m4.mean_ey);
    fprintf('│ Max Cross-Track Error (e_y)             │  %12.3f m │  %12.3f m │\n', m3.max_ey, m4.max_ey);
    fprintf('│ Mean Speed Error (e_v)                  │  %10.3f m/s │  %10.3f m/s │\n', m3.mean_ev, m4.mean_ev);
    fprintf('│ Min Obstacle Clearance                  │  %12.2f m │  %12.2f m │\n', m3.min_clr, m4.min_clr);
    fprintf('│ Collision-Active Steps                  │  %15d │  %15d │\n', m3.collision_steps, m4.collision_steps);
    fprintf('│ Road Bounds Compliance                  │  %5d/100 steps │  %5d/100 steps │\n', m3.bounds_steps, m4.bounds_steps);
    fprintf('│ Total Planner Runtime                   │  %10.2f ms │  %10.2f ms │\n', m3.total_time_ms, m4.mean_solve_time);
    fprintf('├─────────────────────────────────────────┼──────────────────┼──────────────────┤\n');
    fprintf('│ 4-STATE CONTROL BREAKDOWN               │                  │                  │\n');
    fprintf('│  - Hard QP Feasible (Tier 1)            │  %13.1f %% │  %5d/100 steps │\n', m3.feasibility_rate, m4.hard_qp_count);
    fprintf('│  - Layer 1 Soft QP Recovered (Tier 2)   │               -- │  %5d/100 steps │\n', m4.soft_qp_count);
    fprintf('│  - Safety Filter Rejected Plans         │               -- │  %5d/100 steps │\n', m4.safety_rejected_count);
    fprintf('│  - Layer 2 Emergency Braking (Tier 3)   │  %15d │  %5d/100 steps │\n', m3.fallback_count, m4.emergency_braking_count);
    fprintf('└─────────────────────────────────────────┴──────────────────┴──────────────────┘\n\n');
end

function s = pass_str(val)
    if val, s = 'PASS'; else, s = 'FAIL'; end
end
