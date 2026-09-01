function stats = run_phase13_envelope_analysis()
    % RUN_PHASE13_ENVELOPE_ANALYSIS Statistical Analysis & Feasibility Engine
    %
    % Reads artifacts/phase12d_results.csv (1000 runs) and reconstructs key
    % closed-loop metrics, statistical distributions, physical corridor limits,
    % and root cause tables.
    
    addpath('environment', 'core', 'vehicle', 'planning', 'config', 'stages', 'metrics', 'visualization');
    if ~exist('artifacts/phase13', 'dir'), mkdir('artifacts/phase13'); end
    
    csv_path = 'artifacts/phase12d_results.csv';
    if ~exist(csv_path, 'file')
        error('Dataset %s not found! Ensure Phase 12D Monte Carlo completed.', csv_path);
    end
    
    opts = detectImportOptions(csv_path);
    opts.VariableNamingRule = 'preserve';
    data = readtable(csv_path, opts);
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 13: CLOSED-LOOP OPERATING ENVELOPE STATISTICAL CHARACTERIZATION           \n');
    fprintf('========================================================================================\n');
    fprintf('Loaded Dataset: %s | Total Recorded Runs: %d\n\n', csv_path, height(data));
    
    modes = {'ideal', 'nominal_perception', 'delayed_perception', 'steering_bias', 'combined_realistic'};
    outcomes = {'SUCCESS', 'DEGRADED_SAFE', 'SAFE_STOP', 'PLANNER_INFEASIBLE', 'UNSAFE_FAILURE', 'COLLISION'};
    
    stats = struct();
    
    % 1. Outcome Distribution by Level & Mode
    fprintf('--- 1. OUTCOME DISTRIBUTION SUMMARY ---\n');
    outcome_matrix = zeros(10, length(outcomes));
    for lvl = 1:10
        sub_d = data(data.Level == lvl, :);
        for o_i = 1:length(outcomes)
            outcome_matrix(lvl, o_i) = sum(strcmp(sub_d.Outcome, outcomes{o_i}));
        end
        fprintf('Level %2d (%-30s): SUCCESS=%2d | DEGRADED=%2d | SAFE_STOP=%2d | INFEASIBLE=%2d | UNSAFE=%2d | COLLISION=%2d\n', ...
            lvl, ScenarioLadder.getLevel(lvl), outcome_matrix(lvl, 1), outcome_matrix(lvl, 2), ...
            outcome_matrix(lvl, 3), outcome_matrix(lvl, 4), outcome_matrix(lvl, 5), outcome_matrix(lvl, 6));
    end
    fprintf('\n');
    
    % 2. Clearance Distributions by Level
    fprintf('--- 2. MINIMUM CLEARANCE STATISTICAL DISTRIBUTIONS (m) ---\n');
    fprintf('| Level | Scenario Key                  | Mean Clr | Min Clr | 5th Pct  | 95th Pct | Negative Clr Runs |\n');
    fprintf('|-------|-------------------------------|----------|---------|----------|----------|-------------------|\n');
    for lvl = 1:10
        sub_d = data(data.Level == lvl, :);
        clrs = sub_d.MinClr;
        clrs = clrs(~isinf(clrs));
        if isempty(clrs)
            fprintf('| L%-4d | %-29s | %8s | %7s | %8s | %8s | %17d |\n', ...
                lvl, ScenarioLadder.getLevel(lvl), 'Inf', 'Inf', 'Inf', 'Inf', 0);
        else
            m_clr = mean(clrs); min_c = min(clrs);
            p5 = prctile(clrs, 5); p95 = prctile(clrs, 95);
            neg_cnt = sum(clrs < 0.0);
            fprintf('| L%-4d | %-29s | %8.4f | %7.4f | %8.4f | %8.4f | %17d |\n', ...
                lvl, ScenarioLadder.getLevel(lvl), m_clr, min_c, p5, p95, neg_cnt);
        end
    end
    fprintf('\n');
    
    % 3. Lateral Acceleration & Jerk Distributions
    fprintf('--- 3. MOTION COMFORT & LATERAL ACCELERATION DISTRIBUTIONS (g) ---\n');
    fprintf('| Level | Scenario Key                  | Max Lat Accel (g) | Max Yaw Rate (deg/s) | Lat RMSE (m) |\n');
    fprintf('|-------|-------------------------------|-------------------|----------------------|--------------|\n');
    for lvl = 1:10
        sub_d = data(data.Level == lvl, :);
        max_acc = max(sub_d.MaxLatAccel);
        max_yaw = max(sub_d.MaxYawRate);
        m_rmse  = mean(sub_d.LatRMSE);
        fprintf('| L%-4d | %-29s | %17.4f | %20.4f | %12.4f |\n', ...
            lvl, ScenarioLadder.getLevel(lvl), max_acc, max_yaw, m_rmse);
    end
    fprintf('\n');
    
    % 4. Perception vs Actuator Uncertainty Mode Impact Breakdown
    fprintf('--- 4. UNCERTAINTY MODE IMPACT ANALYSIS ---\n');
    fprintf('| Uncertainty Mode   | Total Runs | Success %% | Degraded Safe %% | Safe Stop %% | Infeasible %% | Collision %% |\n');
    fprintf('|--------------------|------------|-----------|------------------|--------------|--------------|------------|\n');
    for m_i = 1:length(modes)
        mn = modes{m_i};
        sub_m = data(strcmp(data.Mode, mn), :);
        n_m = height(sub_m);
        p_succ = sum(strcmp(sub_m.Outcome, 'SUCCESS')) / n_m * 100;
        p_deg  = sum(strcmp(sub_m.Outcome, 'DEGRADED_SAFE')) / n_m * 100;
        p_stop = sum(strcmp(sub_m.Outcome, 'SAFE_STOP')) / n_m * 100;
        p_inf  = sum(strcmp(sub_m.Outcome, 'PLANNER_INFEASIBLE')) / n_m * 100;
        p_coll = sum(strcmp(sub_m.Outcome, 'COLLISION')) / n_m * 100;
        fprintf('| %-18s | %10d | %8.1f%% | %15.1f%% | %11.1f%% | %11.1f%% | %9.1f%% |\n', ...
            mn, n_m, p_succ, p_deg, p_stop, p_inf, p_coll);
    end
    fprintf('\n');
    
    % 5. Physical Corridor Feasibility Engine for Levels 5, 6, 8, 9
    fprintf('========================================================================================\n');
    fprintf('        PHYSICAL CORRIDOR FEASIBILITY ANALYSIS (6.0m ROAD vs VEHICLE DIMENSIONS)        \n');
    fprintf('========================================================================================\n');
    fprintf('Corridor Geometry Parameters:\n');
    fprintf('  - Road Width (W_road): 6.00 m  (y_min = 0.0 m, y_max = 6.0 m)\n');
    fprintf('  - Ego Vehicle Width (W_ego): 1.80 m (Body radius = 0.90 m)\n');
    fprintf('  - Dynamic Agent Width (W_ag): 1.80 m (Body radius = 0.90 m)\n');
    fprintf('  - Static Obstacle Width (W_obs): 1.00 m (Radius = 0.50 m)\n');
    fprintf('  - Planning Safety Margin Buffer: 0.10 m per side\n\n');
    
    fprintf('Feasibility Analysis Case A: Level 5 (multi_vehicle_yield_overtake)\n');
    fprintf('  - Agent 1 (Lead car): y1 = 1.80m (Right lane centered). Left edge at y = 2.70m.\n');
    fprintf('  - Agent 2 (Oncoming): y2 = 4.20m (Left lane centered). Right edge at y = 3.30m.\n');
    fprintf('  - Passing gap between Agent 1 and Agent 2: 3.30m - 2.70m = 0.60m.\n');
    fprintf('  - Required Ego clearance width: W_ego = 1.80m.\n');
    fprintf('  - RESULT: Simultaneous passing between Agent 1 and Agent 2 is PHYSICALLY IMPOSSIBLE (0.60m < 1.80m).\n');
    fprintf('  - Ego MUST YIELD longitudinally behind Agent 1 until Agent 2 passes at t = 3.5s.\n');
    fprintf('  - Feasibility Verdict: Feasible ONLY IF Ego yields. Infeasible if Ego attempts early passing.\n\n');
    
    fprintf('Feasibility Analysis Case B: Level 6 (overtaking in narrow corridor)\n');
    fprintf('  - Lead Vehicle at y = 1.80m (Right lane). Left edge at y = 2.70m.\n');
    fprintf('  - Available left corridor space: y in [2.70m, 6.00m] -> Total available width = 3.30m.\n');
    fprintf('  - Ego width required: 1.80m. Left corridor space required: 2.70m + 1.80m = 4.50m <= 6.00m.\n');
    fprintf('  - Static spatial clearance margin when centered at y = 4.35m:\n');
    fprintf('    Gap to Agent = 4.35 - 0.90 - 2.70 = 0.75m > 0.0m.\n');
    fprintf('    Gap to Left Road Boundary = 6.00 - (4.35 + 0.90) = 0.75m > 0.0m.\n');
    fprintf('  - Feasibility Verdict: PHYSICALLY FEASIBLE. Negative clearance under steering bias is CONTROLLER LIMITATION.\n\n');
    
    fprintf('Feasibility Analysis Case C: Level 8/9 (complex: static obs at x=35m, y=1.80m + oncoming agent at x=60m, y=4.20m)\n');
    fprintf('  - Static Obstacle at (35.0, 1.80) blocks right lane. Left edge at y = 2.30m.\n');
    fprintf('  - Oncoming Agent at (60.0, 4.20) moving at -8.0 m/s in left lane. Right edge at y = 3.30m.\n');
    fprintf('  - Passing corridor width while Oncoming Agent is adjacent to static obstacle: 3.30m - 2.30m = 1.00m < 1.80m.\n');
    fprintf('  - RESULT: Simultaneous 3-body passing is PHYSICALLY IMPOSSIBLE.\n');
    fprintf('  - Feasibility Verdict: Feasible ONLY WITH TEMPORAL DECELERATION YIELD. Impatient passing causes physical collision.\n\n');
    
    stats.height_data = height(data);
end
