function benchmark_results = run_proposal_ab_benchmark()
    % RUN_PROPOSAL_AB_BENCHMARK Comprehensive 3x3 Proposal Evaluation Matrix
    %
    % Evaluates 3 Configurations across 3 Scenarios:
    %   Configs:
    %     Config A: Baseline (Existing SIH Validated Pipeline)
    %     Config B: + Curvature-Aware Velocity Profiler
    %     Config C: + Curvature-Aware Velocity Profiler + Delay-Aware Steering Filter
    %
    %   Scenarios:
    %     1. Indian Unstructured Bottleneck (2.0m static corridor gap, FreeSpaceMap)
    %     2. Indian Unstructured Cattle Crossing (dynamic agent with expanding uncertainty)
    %     3. R=18.5m Chicane Stress Test (C2-continuous quintic S-curve lateral dynamics & chatter test)
    %
    % Features:
    %   1. Matched-Distance vs Full-Run RMSE Disaggregation (Fair comparison over common distance)
    %   2. Reference vs Actual Peak Lateral Acceleration (v_ref^2*kappa vs actual plant |v*omega|)
    %   3. Sub-system Latency Disaggregation (Planner QP, Controller logic, Safety Filter, Overhead)
    %   4. Actuator Reversal & Limit-Cycle Dynamics Audit
    
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir), projectRoot = pwd; else projectRoot = fileparts(current_dir); end
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'core'));
    addpath(fullfile(projectRoot, 'environment'));
    addpath(fullfile(projectRoot, 'vehicle'));
    addpath(fullfile(projectRoot, 'stages'));
    addpath(fullfile(projectRoot, 'tests'));
    
    fprintf('\n==================================================================================================================\n');
    fprintf('                   SIH 2026 PS26037 — FINAL PROPOSAL EVIDENCE BENCHMARK MATRIX                                    \n');
    fprintf('==================================================================================================================\n\n');
    
    cfg = SimulationConfig();
    
    scenarios = {'bottleneck', 'cattle', 'chicane'};
    configs = {
        'Config A (Baseline)',    false, false;
        'Config B (+Velocity)',   true,  false;
        'Config C (+Vel +Steer)', true,  true
    };
    
    n_scen = length(scenarios);
    n_conf = size(configs, 1);
    
    benchmark_results = struct();
    traces = struct();
    
    for s_idx = 1:n_scen
        scen_key = scenarios{s_idx};
        fprintf('==================================================================================================================\n');
        fprintf('  SCENARIO %d/%d: %s\n', s_idx, n_scen, upper(scen_key));
        fprintf('==================================================================================================================\n');
        
        for c_idx = 1:n_conf
            conf_name = configs{c_idx, 1};
            use_vel   = configs{c_idx, 2};
            use_steer = configs{c_idx, 3};
            
            fprintf('  Evaluating [%s] ...\n', conf_name);
            
            [res, trace] = run_single_simulation(scen_key, use_vel, use_steer, cfg);
            
            res_key = sprintf('%s_%s', scen_key, regexprep(conf_name, '[^a-zA-Z0-9]', ''));
            benchmark_results.(res_key) = res;
            traces.(res_key) = trace;
        end
        
        % Compute Fair Matched-Distance Tracking RMSE across all 3 configurations
        key_A = sprintf('%s_%s', scen_key, regexprep(configs{1, 1}, '[^a-zA-Z0-9]', ''));
        key_B = sprintf('%s_%s', scen_key, regexprep(configs{2, 1}, '[^a-zA-Z0-9]', ''));
        key_C = sprintf('%s_%s', scen_key, regexprep(configs{3, 1}, '[^a-zA-Z0-9]', ''));
        
        resA = benchmark_results.(key_A);
        resB = benchmark_results.(key_B);
        resC = benchmark_results.(key_C);
        
        trA = traces.(key_A);
        trB = traces.(key_B);
        trC = traces.(key_C);
        
        x_matched = min([resA.max_x, resB.max_x, resC.max_x]);
        
        idxA_m = trA.ego_x <= x_matched;
        idxB_m = trB.ego_x <= x_matched;
        idxC_m = trC.ego_x <= x_matched;
        
        resA.matched_distance_m = x_matched;
        resB.matched_distance_m = x_matched;
        resC.matched_distance_m = x_matched;
        
        resA.matched_lat_rmse = sqrt(mean(trA.lat_err(idxA_m).^2));
        resB.matched_lat_rmse = sqrt(mean(trB.lat_err(idxB_m).^2));
        resC.matched_lat_rmse = sqrt(mean(trC.lat_err(idxC_m).^2));
        
        resA.full_lat_rmse = resA.lat_rmse;
        resB.full_lat_rmse = resB.lat_rmse;
        resC.full_lat_rmse = resC.lat_rmse;
        
        benchmark_results.(key_A) = resA;
        benchmark_results.(key_B) = resB;
        benchmark_results.(key_C) = resC;
        
        for c_idx = 1:n_conf
            conf_name = configs{c_idx, 1};
            r_key = sprintf('%s_%s', scen_key, regexprep(conf_name, '[^a-zA-Z0-9]', ''));
            r = benchmark_results.(r_key);
            fprintf('    [%s] Completed: %s | Max X: %.1fm (Matched: %.1fm) | Matched RMSE: %.2fcm | Full RMSE: %.2fcm\n', ...
                conf_name, r.completed_str, r.max_x, r.matched_distance_m, r.matched_lat_rmse * 100, r.full_lat_rmse * 100);
            fprintf('         Lateral Accel: Ref Peak=%.2f m/s^2 | Actual Peak=%.2f m/s^2 | Reversals: %d\n', ...
                r.reference_peak_a_lat, r.actual_peak_a_lat, r.reversal_count);
            fprintf('         Latency: Total=%.2fms [Planner: %.2fms, Ctrl: %.2fms, Safety: %.2fms, Overhead: %.2fms] | Wall: %.2fs\n', ...
                r.mean_latency_ms, r.planner_latency_ms, r.controller_latency_ms, r.safety_filter_latency_ms, r.overhead_latency_ms, r.wall_clock_s);
        end
        fprintf('\n');
    end
    
    % Assemble CSV
    results_dir = fullfile(projectRoot, 'results');
    if ~exist(results_dir, 'dir'), mkdir(results_dir); end
    
    csv_rows = {};
    csv_rows{1} = 'Scenario,Configuration,Completed,TotalSteps,MaxX_m,MatchedDist_m,MatchedRMSE_cm,FullRMSE_cm,SteeringReversals,SteeringEffort_rad2,RefPeakLatAccel_mps2,ActualPeakLatAccel_mps2,MeanLatency_ms,PlannerLatency_ms,ControllerLatency_ms,SafetyFilterLatency_ms,OverheadLatency_ms,WallClock_s,SafetyInterventions';
    
    for s_idx = 1:n_scen
        scen_key = scenarios{s_idx};
        for c_idx = 1:n_conf
            conf_name = configs{c_idx, 1};
            r_key = sprintf('%s_%s', scen_key, regexprep(conf_name, '[^a-zA-Z0-9]', ''));
            res = benchmark_results.(r_key);
            csv_line = sprintf('%s,%s,%s,%d,%.2f,%.2f,%.2f,%.2f,%d,%.5f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d', ...
                scen_key, conf_name, res.completed_str, res.total_steps, res.max_x, ...
                res.matched_distance_m, res.matched_lat_rmse * 100, res.full_lat_rmse * 100, ...
                res.reversal_count, res.steering_effort, res.reference_peak_a_lat, res.actual_peak_a_lat, ...
                res.mean_latency_ms, res.planner_latency_ms, res.controller_latency_ms, res.safety_filter_latency_ms, ...
                res.overhead_latency_ms, res.wall_clock_s, res.safety_interventions);
            csv_rows{end+1} = csv_line; %#ok<AGROW>
        end
    end
    
    csv_path = fullfile(results_dir, 'proposal_benchmark.csv');
    fid = fopen(csv_path, 'w');
    for i = 1:length(csv_rows)
        fprintf(fid, '%s\n', csv_rows{i});
    end
    fclose(fid);
    fprintf('>>> Saved benchmark CSV to: %s\n', csv_path);
    
    % Save MAT file for plotting figures
    mat_path = fullfile(results_dir, 'proposal_benchmark_data.mat');
    save(mat_path, 'benchmark_results', 'traces', 'scenarios', 'configs');
    fprintf('>>> Saved telemetry MAT to: %s\n', mat_path);
    
    % Generate Markdown Summary Report
    generate_markdown_summary(fullfile(results_dir, 'proposal_benchmark_summary.md'), benchmark_results, scenarios, configs);
    fprintf('>>> Saved summary report to: %s\n', fullfile(results_dir, 'proposal_benchmark_summary.md'));
    
    % Print terminal comparative summary scorecard
    print_comparative_summary(benchmark_results, scenarios, configs);
end

function [res, trace] = run_single_simulation(scen_key, use_vel, use_steer, cfg)
    vehicle = BicycleModel(cfg);
    ctrl = Stage5CoordinationController(cfg);
    ctrl.reset();
    ctrl.use_curvature_velocity = use_vel;
    ctrl.use_steering_filter = use_steer;
    
    switch scen_key
        case 'bottleneck'
            world = ScenarioDefinitions('indian_unstructured_bottleneck', cfg);
            world.n_static_obs = 0;
            map_obj = FreeSpaceMap.createIndianUnstructuredMap();
            ctrl.cacrc_planner.bound_provider = FreeSpaceBoundProvider(map_obj);
            ctrl.setOvertakeEnabled(true);
            
            N_path = 700;
            ref_path = zeros(N_path, 5);
            ref_path(:, 1) = linspace(0, 150, N_path)';
            ref_path(:, 2) = 2.75;
            ref_path(:, 4) = 0.0;
            ref_path(:, 5) = 5.0;
            
            v_nominal = 5.0;
            N_steps = 250;
            target_completion_x = 75.0;
            
        case 'cattle'
            world = ScenarioDefinitions('indian_unstructured', cfg);
            map_obj = FreeSpaceMap.createIndianUnstructuredMap();
            ctrl.cacrc_planner.bound_provider = FreeSpaceBoundProvider(map_obj);
            ctrl.setOvertakeEnabled(true);
            
            N_path = 700;
            ref_path = zeros(N_path, 5);
            ref_path(:, 1) = linspace(0, 150, N_path)';
            ref_path(:, 2) = 2.75;
            ref_path(:, 4) = 0.0;
            ref_path(:, 5) = 5.0;
            
            v_nominal = 5.0;
            N_steps = 250;
            target_completion_x = 55.0;
            
        case 'chicane'
            world = WorldState(cfg);
            world = world.setEgoState(0.0, 3.0, 0.0, 8.0);
            
            N_path = 500;
            x_pts = linspace(0, 120, N_path)';
            y_pts = 3.0 * ones(N_path, 1);
            S = @(u) 10*u.^3 - 15*u.^4 + 6*u.^5;
            for i = 1:N_path
                xi = x_pts(i);
                if xi < 20.0
                    y_pts(i) = 3.0;
                elseif xi <= 31.32
                    u = (xi - 20.0) / 11.32;
                    y_pts(i) = 3.0 + 1.20 * S(u);
                elseif xi <= 40.0
                    y_pts(i) = 4.20;
                elseif xi <= 56.02
                    u = (xi - 40.0) / 16.02;
                    y_pts(i) = 4.20 - 2.40 * S(u);
                elseif xi <= 65.0
                    y_pts(i) = 1.80;
                elseif xi <= 76.32
                    u = (xi - 65.0) / 11.32;
                    y_pts(i) = 1.80 + 1.20 * S(u);
                else
                    y_pts(i) = 3.0;
                end
            end
            
            ref_path = zeros(N_path, 5);
            ref_path(:, 1) = x_pts;
            ref_path(:, 2) = y_pts;
            for i = 1:N_path
                i1 = max(1, i-1); i2 = min(N_path, i+1);
                ref_path(i, 3) = atan2(y_pts(i2) - y_pts(i1), x_pts(i2) - x_pts(i1));
            end
            ref_path(:, 4) = 0.0;
            ref_path(:, 5) = 10.0;
            
            v_nominal = 10.0;
            N_steps = 220;
            target_completion_x = 100.0;
    end
    
    dt = cfg.dt;
    
    % Analytical curvature of reference trajectory
    p_curv = CurvatureVelocityPlanner();
    [kappa_ref_all, ~, ~] = p_curv.computeCurvature(ref_path(:, 1), ref_path(:, 2));
    
    % If velocity profiler is active, compute the reference speed profile ahead of time
    if use_vel
        ref_path_prof = p_curv.planVelocityProfile(ref_path, v_nominal);
        v_ref_profile = ref_path_prof(:, 5);
    else
        v_ref_profile = ref_path(:, 5);
    end
    
    trace.t          = zeros(N_steps, 1);
    trace.ego_x      = zeros(N_steps, 1);
    trace.ego_y      = zeros(N_steps, 1);
    trace.ego_psi    = zeros(N_steps, 1);
    trace.ego_v      = zeros(N_steps, 1);
    trace.delta_cmd  = zeros(N_steps, 1);
    trace.delta_act  = zeros(N_steps, 1);
    trace.a_cmd      = zeros(N_steps, 1);
    trace.y_ref      = zeros(N_steps, 1);
    trace.lat_err    = zeros(N_steps, 1);
    trace.clr        = zeros(N_steps, 1);
    trace.step_ms    = zeros(N_steps, 1);
    trace.planner_ms = zeros(N_steps, 1);
    trace.ctrl_ms    = zeros(N_steps, 1);
    trace.safety_ms  = zeros(N_steps, 1);
    trace.overhead_ms= zeros(N_steps, 1);
    trace.a_lat_ref  = zeros(N_steps, 1);
    
    safety_interventions = 0;
    collisions = 0;
    
    t_wall_start = tic;
    
    for k = 1:N_steps
        t_now = (k - 1) * dt;
        
        t_step_start = tic;
        [u_cmd, pred_states, status, info] = ctrl.step(world, ref_path, v_nominal);
        step_latency_ms = toc(t_step_start) * 1000;
        
        t_overhead_start = tic;
        
        if isfield(info, 'filter_active') && info.filter_active
            safety_interventions = safety_interventions + 1;
        end
        
        [~, nearest_idx] = min(abs(ref_path(:, 1) - world.ego.x));
        y_ref_k = ref_path(nearest_idx, 2);
        lat_err_k = world.ego.y - y_ref_k;
        
        % Reference lateral acceleration at current longitudinal position: a_lat_ref = v_ref^2 * kappa_ref
        k_ref_k = abs(kappa_ref_all(nearest_idx));
        v_ref_k = v_ref_profile(nearest_idx);
        a_lat_ref_k = (v_ref_k^2) * k_ref_k;
        
        world.ego = vehicle.stepKinematic(world.ego, u_cmd(2), u_cmd(1), dt);
        
        clr_k = world.getMinClearance(cfg);
        if clr_k <= 0
            collisions = collisions + 1;
        end
        
        if isprop(world, 'agents') && world.n_agents > 0
            world = world.stepAgents(dt);
        end
        
        overhead_ms = toc(t_overhead_start) * 1000;
        
        trace.t(k)          = t_now;
        trace.ego_x(k)      = world.ego.x;
        trace.ego_y(k)      = world.ego.y;
        trace.ego_psi(k)    = world.ego.theta;
        trace.ego_v(k)      = world.ego.v;
        trace.delta_cmd(k)  = u_cmd(1);
        trace.delta_act(k)  = world.ego.delta;
        trace.a_cmd(k)      = u_cmd(2);
        trace.y_ref(k)      = y_ref_k;
        trace.lat_err(k)    = lat_err_k;
        trace.clr(k)        = clr_k;
        trace.step_ms(k)    = step_latency_ms;
        trace.a_lat_ref(k)  = a_lat_ref_k;
        
        if isfield(info, 'timing')
            trace.planner_ms(k)  = info.timing.planner_ms;
            trace.ctrl_ms(k)     = info.timing.controller_ms;
            trace.safety_ms(k)   = info.timing.safety_filter_ms;
        else
            trace.planner_ms(k)  = step_latency_ms;
            trace.ctrl_ms(k)     = 0;
            trace.safety_ms(k)   = 0;
        end
        trace.overhead_ms(k) = overhead_ms;
        
        if world.ego.x >= target_completion_x && k > 20
            trace.t          = trace.t(1:k);
            trace.ego_x      = trace.ego_x(1:k);
            trace.ego_y      = trace.ego_y(1:k);
            trace.ego_psi    = trace.ego_psi(1:k);
            trace.ego_v      = trace.ego_v(1:k);
            trace.delta_cmd  = trace.delta_cmd(1:k);
            trace.delta_act  = trace.delta_act(1:k);
            trace.a_cmd      = trace.a_cmd(1:k);
            trace.y_ref      = trace.y_ref(1:k);
            trace.lat_err    = trace.lat_err(1:k);
            trace.clr        = trace.clr(1:k);
            trace.step_ms    = trace.step_ms(1:k);
            trace.planner_ms = trace.planner_ms(1:k);
            trace.ctrl_ms    = trace.ctrl_ms(1:k);
            trace.safety_ms  = trace.safety_ms(1:k);
            trace.overhead_ms= trace.overhead_ms(1:k);
            trace.a_lat_ref  = trace.a_lat_ref(1:k);
            break;
        end
        
        % Emergency stop standstill detection
        if world.ego.v <= 0.05 && k > 30
            trace.t          = trace.t(1:k);
            trace.ego_x      = trace.ego_x(1:k);
            trace.ego_y      = trace.ego_y(1:k);
            trace.ego_psi    = trace.ego_psi(1:k);
            trace.ego_v      = trace.ego_v(1:k);
            trace.delta_cmd  = trace.delta_cmd(1:k);
            trace.delta_act  = trace.delta_act(1:k);
            trace.a_cmd      = trace.a_cmd(1:k);
            trace.y_ref      = trace.y_ref(1:k);
            trace.lat_err    = trace.lat_err(1:k);
            trace.clr        = trace.clr(1:k);
            trace.step_ms    = trace.step_ms(1:k);
            trace.planner_ms = trace.planner_ms(1:k);
            trace.ctrl_ms    = trace.ctrl_ms(1:k);
            trace.safety_ms  = trace.safety_ms(1:k);
            trace.overhead_ms= trace.overhead_ms(1:k);
            trace.a_lat_ref  = trace.a_lat_ref(1:k);
            break;
        end
    end
    
    wall_clock_duration = toc(t_wall_start);
    actual_steps = length(trace.t);
    
    res.total_steps = actual_steps;
    res.max_x = max(trace.ego_x);
    res.completed = (res.max_x >= target_completion_x) || (strcmp(scen_key, 'cattle') && actual_steps >= 200 && collisions == 0);
    if res.completed
        res.completed_str = 'YES';
    else
        res.completed_str = 'NO';
    end
    
    res.collisions = collisions;
    res.min_clearance = min(trace.clr);
    res.lat_rmse = sqrt(mean(trace.lat_err.^2));
    
    % Reference Lateral Acceleration Peak (planned target)
    res.reference_peak_a_lat = max(trace.a_lat_ref);
    
    % Actual Closed-Loop Physical Vehicle Lateral Acceleration: a_lat_actual = |v * d(psi)/dt|
    yaw_rates = [0; diff(trace.ego_psi) / dt];
    lat_accels = abs(trace.ego_v .* yaw_rates);
    res.actual_peak_a_lat = max(lat_accels);
    res.peak_lat_accel = res.actual_peak_a_lat; % Maintain backwards-compatibility
    
    % Exclude first step for mean latency to avoid JIT startup bias
    if actual_steps > 1
        res.mean_latency_ms          = mean(trace.step_ms(2:end));
        res.planner_latency_ms       = mean(trace.planner_ms(2:end));
        res.controller_latency_ms    = mean(trace.ctrl_ms(2:end));
        res.safety_filter_latency_ms = mean(trace.safety_ms(2:end));
        res.overhead_latency_ms      = mean(trace.overhead_ms(2:end));
    else
        res.mean_latency_ms          = trace.step_ms(1);
        res.planner_latency_ms       = trace.planner_ms(1);
        res.controller_latency_ms    = trace.ctrl_ms(1);
        res.safety_filter_latency_ms = trace.safety_ms(1);
        res.overhead_latency_ms      = trace.overhead_ms(1);
    end
    res.wall_clock_s = wall_clock_duration;
    res.safety_interventions = safety_interventions;
    
    delta_cmds = trace.delta_cmd;
    diff_cmd = diff(delta_cmds);
    res.steering_effort = sum(diff_cmd.^2);
    
    reversals = 0;
    rate_signs = zeros(length(diff_cmd), 1);
    for i = 1:length(diff_cmd)
        if diff_cmd(i) > 0.002
            rate_signs(i) = 1;
        elseif diff_cmd(i) < -0.002
            rate_signs(i) = -1;
        end
    end
    last_sign = 0;
    for i = 1:length(rate_signs)
        s = rate_signs(i);
        if s ~= 0
            if last_sign ~= 0 && s ~= last_sign
                reversals = reversals + 1;
            end
            last_sign = s;
        end
    end
    res.reversal_count = reversals;
end

function generate_markdown_summary(summary_path, results, scenarios, configs)
    fid = fopen(summary_path, 'w');
    fprintf(fid, '# SIH 2026 PS26037 — Proposal A/B/C Benchmark Summary\n\n');
    fprintf(fid, '**Protocol**: Multi-Scenario Evaluation Matrix comparing:\n');
    fprintf(fid, '- **Config A (Baseline)**: Frozen Stage 4/5 QP-MPC & CA-CRC Planner\n');
    fprintf(fid, '- **Config B (+ Velocity)**: Curvature-Aware Velocity Profiler Enabled\n');
    fprintf(fid, '- **Config C (+ Vel + Steer)**: Curvature Velocity + Delay-Aware Steering Filter Enabled\n\n');
    fprintf(fid, '> [!NOTE]\n');
    fprintf(fid, '> All metrics reflect actual numerical measurements. Per the scientific reporting constraint, engineering targets (>=60%% chatter reduction, <=3 cm RMSE degradation) are used as design targets rather than pass/fail filters.\n\n');
    fprintf(fid, '> [!IMPORTANT]\n');
    fprintf(fid, '> **Scientific Metric Definitions**:\n');
    fprintf(fid, '> - **Reference Peak $a_{\\text{lat}}$**: $\\max(v_{\\text{ref}}^2 \\cdot |\\kappa_{\\text{ref}}|)$ of planned speed profile.\n');
    fprintf(fid, '> - **Actual Peak $a_{\\text{lat}}$**: $\\max(|v_{\\text{ego}} \\cdot \\omega_{\\text{yaw}}|)$ measured on the simulated vehicle plant.\n');
    fprintf(fid, '> - **Matched-Distance RMSE**: Evaluated over identical common longitudinal distance $x \\le \\min(x_{\\text{end},A}, x_{\\text{end},B}, x_{\\text{end},C})$ for fair side-by-side comparison.\n');
    fprintf(fid, '> - **Full-Run RMSE**: Evaluated over full individual trajectory up to completion or safety stop.\n');
    fprintf(fid, '> - **Latency Disaggregation**: Separates pure algorithmic computation (Planner QP, Controller logic, Safety Filter) from benchmark simulation/logging overhead. Real-time embedded performance is NOT claimed from total wall-clock benchmark time.\n\n');
    
    for s = 1:length(scenarios)
        scen_key = scenarios{s};
        fprintf(fid, '## Scenario: `%s`\n\n', scen_key);
        fprintf(fid, '| Metric | Config A (Baseline) | Config B (+ Velocity) | Config C (+ Vel + Steer) | Delta (C vs A) |\n');
        fprintf(fid, '| :--- | :---: | :---: | :---: | :---: |\n');
        
        key_A = sprintf('%s_%s', scen_key, regexprep(configs{1, 1}, '[^a-zA-Z0-9]', ''));
        key_B = sprintf('%s_%s', scen_key, regexprep(configs{2, 1}, '[^a-zA-Z0-9]', ''));
        key_C = sprintf('%s_%s', scen_key, regexprep(configs{3, 1}, '[^a-zA-Z0-9]', ''));
        
        res_A = results.(key_A);
        res_B = results.(key_B);
        res_C = results.(key_C);
        
        fprintf(fid, '| **Completion Status** | %s | %s | %s | %s |\n', res_A.completed_str, res_B.completed_str, res_C.completed_str, '-');
        fprintf(fid, '| **Max Progress ($x_{\\max}$)** | %.2f m | %.2f m | %.2f m | %+.2f m |\n', res_A.max_x, res_B.max_x, res_C.max_x, res_C.max_x - res_A.max_x);
        fprintf(fid, '| **Matched Comparison Distance** | $x \\le %.2f$ m | $x \\le %.2f$ m | $x \\le %.2f$ m | - |\n', res_A.matched_distance_m, res_B.matched_distance_m, res_C.matched_distance_m);
        
        matched_diff_cm = (res_C.matched_lat_rmse - res_A.matched_lat_rmse) * 100;
        full_diff_cm = (res_C.full_lat_rmse - res_A.full_lat_rmse) * 100;
        fprintf(fid, '| **Matched-Distance Tracking RMSE** | **%.2f cm** | **%.2f cm** | **%.2f cm** | **%+.2f cm** |\n', res_A.matched_lat_rmse * 100, res_B.matched_lat_rmse * 100, res_C.matched_lat_rmse * 100, matched_diff_cm);
        fprintf(fid, '| **Full-Run Tracking RMSE** | %.2f cm | %.2f cm | %.2f cm | %+.2f cm |\n', res_A.full_lat_rmse * 100, res_B.full_lat_rmse * 100, res_C.full_lat_rmse * 100, full_diff_cm);
        
        fprintf(fid, '| **Reference Peak $a_{\\text{lat}}$ ($v_{\\text{ref}}^2 \\kappa$)** | %.3f m/s² | **%.3f m/s²** | **%.3f m/s²** | %+.3f m/s² |\n', res_A.reference_peak_a_lat, res_B.reference_peak_a_lat, res_C.reference_peak_a_lat, res_C.reference_peak_a_lat - res_A.reference_peak_a_lat);
        fprintf(fid, '| **Actual Peak $a_{\\text{lat}}$ ($|v \\omega_{\\text{yaw}}|$)** | %.3f m/s² | %.3f m/s² | %.3f m/s² | %+.3f m/s² |\n', res_A.actual_peak_a_lat, res_B.actual_peak_a_lat, res_C.actual_peak_a_lat, res_C.actual_peak_a_lat - res_A.actual_peak_a_lat);
        
        rev_pct = 0;
        if res_A.reversal_count > 0
            rev_pct = ((res_C.reversal_count - res_A.reversal_count) / res_A.reversal_count) * 100;
        end
        fprintf(fid, '| **Steering Reversals (Chatter)** | %d | %d | %d | **%+.1f%%** (%+d) |\n', res_A.reversal_count, res_B.reversal_count, res_C.reversal_count, rev_pct, res_C.reversal_count - res_A.reversal_count);
        fprintf(fid, '| **Steering Effort ($\\sum |\\Delta \\delta|^2$)** | %.5f | %.5f | %.5f | %+.5f |\n', res_A.steering_effort, res_B.steering_effort, res_C.steering_effort, res_C.steering_effort - res_A.steering_effort);
        fprintf(fid, '| **Min Clearance ($d_{\\min}$)** | %.4f m | %.4f m | %.4f m | %+.4f m |\n', res_A.min_clearance, res_B.min_clearance, res_C.min_clearance, res_C.min_clearance - res_A.min_clearance);
        
        fprintf(fid, '| **Total Step Latency** | %.2f ms | %.2f ms | %.2f ms | %+.2f ms |\n', res_A.mean_latency_ms, res_B.mean_latency_ms, res_C.mean_latency_ms, res_C.mean_latency_ms - res_A.mean_latency_ms);
        fprintf(fid, '| └─ *Planner QP Solve* | %.2f ms | %.2f ms | %.2f ms | %+.2f ms |\n', res_A.planner_latency_ms, res_B.planner_latency_ms, res_C.planner_latency_ms, res_C.planner_latency_ms - res_A.planner_latency_ms);
        fprintf(fid, '| └─ *Controller Logic / Filter* | %.2f ms | %.2f ms | %.2f ms | %+.2f ms |\n', res_A.controller_latency_ms, res_B.controller_latency_ms, res_C.controller_latency_ms, res_C.controller_latency_ms - res_A.controller_latency_ms);
        fprintf(fid, '| └─ *Safety Filter* | %.2f ms | %.2f ms | %.2f ms | %+.2f ms |\n', res_A.safety_filter_latency_ms, res_B.safety_filter_latency_ms, res_C.safety_filter_latency_ms, res_C.safety_filter_latency_ms - res_A.safety_filter_latency_ms);
        fprintf(fid, '| └─ *Simulation Overhead* | %.2f ms | %.2f ms | %.2f ms | %+.2f ms |\n', res_A.overhead_latency_ms, res_B.overhead_latency_ms, res_C.overhead_latency_ms, res_C.overhead_latency_ms - res_A.overhead_latency_ms);
        fprintf(fid, '| **Wall-Clock Duration** | %.2f s | %.2f s | %.2f s | %+.2f s |\n', res_A.wall_clock_s, res_B.wall_clock_s, res_C.wall_clock_s, res_C.wall_clock_s - res_A.wall_clock_s);
        fprintf(fid, '| **Safety Filter Overrides** | %d | %d | %d | %+d |\n\n', res_A.safety_interventions, res_B.safety_interventions, res_C.safety_interventions, res_C.safety_interventions - res_A.safety_interventions);
    end
    
    fclose(fid);
end

function print_comparative_summary(results, scenarios, configs)
    fprintf('\n====================================================================================================================================\n');
    fprintf('                                  FINAL BENCHMARK COMPARATIVE SCORECARD (MATCHED VS FULL)                                           \n');
    fprintf('====================================================================================================================================\n');
    fprintf('  %-10s | %-15s | %-6s | %-12s | %-11s | %-9s | %-8s | %-8s | %-8s | %-7s\n', ...
        'Scenario', 'Config', 'Status', 'Matched RMSE', 'Full RMSE', 'Reversals', 'Ref alat', 'Act alat', 'Plan ms', 'Wall (s)');
    fprintf('  -----------+-----------------+--------+--------------+-------------+-----------+----------+----------+----------+---------\n');
    
    for s = 1:length(scenarios)
        scen_key = scenarios{s};
        for c = 1:size(configs, 1)
            conf_name = configs{c, 1};
            res_key = sprintf('%s_%s', scen_key, regexprep(conf_name, '[^a-zA-Z0-9]', ''));
            r = results.(res_key);
            fprintf('  %-10s | %-15s | %-6s | %10.2fcm | %9.2fcm | %9d | %6.2fm/s² | %6.2fm/s² | %6.2fms | %6.2fs\n', ...
                scen_key, conf_name, r.completed_str, r.matched_lat_rmse * 100, r.full_lat_rmse * 100, ...
                r.reversal_count, r.reference_peak_a_lat, r.actual_peak_a_lat, r.planner_latency_ms, r.wall_clock_s);
        end
        if s < length(scenarios)
            fprintf('  -----------+-----------------+--------+--------------+-------------+-----------+----------+----------+----------+---------\n');
        end
    end
    fprintf('====================================================================================================================================\n\n');
end
