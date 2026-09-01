function run_phase12d_monte_carlo(varargin)
    % RUN_PHASE12D_MONTE_CARLO Combined Closed-Loop Uncertainty & Robustness Sweep
    %
    % Options:
    %   'smoke_test': true/false (Default: false -> 1000 runs; true -> 50 runs)
    %   'n_seeds':    integer    (Default: 20 for full sweep, 1 for smoke test)
    
    addpath('environment', 'core', 'vehicle', 'planning', 'config', 'stages', 'metrics', 'visualization');
    if ~exist('artifacts', 'dir'), mkdir('artifacts'); end
    
    p = inputParser;
    addParameter(p, 'smoke_test', false, @islogical);
    addParameter(p, 'n_seeds', 20, @isnumeric);
    parse(p, varargin{:});
    
    is_smoke = p.Results.smoke_test;
    if is_smoke
        n_seeds = 1;
        fprintf('\n=== RUNNING PHASE 12D MONTE CARLO SMOKE TEST (10 Levels x 5 Modes x 1 Seed = 50 Runs) ===\n\n');
    else
        n_seeds = p.Results.n_seeds;
        fprintf('\n=== RUNNING FULL PHASE 12D MONTE CARLO SWEEP (10 Levels x 5 Modes x %d Seeds = %d Runs) ===\n\n', ...
            n_seeds, 10 * 5 * n_seeds);
    end
    
    modes = {'ideal', 'nominal_perception', 'delayed_perception', 'steering_bias', 'combined_realistic'};
    
    results = [];
    failures = [];
    
    requested_runs = 10 * length(modes) * n_seeds;
    completed_runs = 0;
    failed_runs = 0;
    
    % Open CSV files for writing
    res_csv_path = 'artifacts/phase12d_results.csv';
    fail_csv_path = 'artifacts/phase12d_failure_cases.csv';
    
    fid_res = fopen(res_csv_path, 'w');
    fprintf(fid_res, 'Requested,Completed,Failed,Level,Mode,Seed,TrajectoryHash,Outcome,LatRMSE,MinClr,MinTTC,MaxLatAccel,MaxYawRate,EmergencyCount,QPInfeasCount\n');
    
    fid_fail = fopen(fail_csv_path, 'w');
    fprintf(fid_fail, 'Requested,Completed,Failed,Level,Mode,Seed,TrajectoryHash,Outcome,Reason,MinClr,LatRMSE\n');
    
    run_counter = 0;
    
    for lvl = 1:10
        [scen_key, lvl_name] = ScenarioLadder.getLevel(lvl);
        
        for m_i = 1:length(modes)
            mode_name = modes{m_i};
            
            for s_i = 1:n_seeds
                seed = 42 + s_i - 1;
                run_counter = run_counter + 1;
                
                if mod(run_counter, 25) == 0 || run_counter == 1 || run_counter == requested_runs
                    fprintf('  [Progress %3d/%3d] Level %2d (%s) | Mode: %-18s | Seed: %d\n', ...
                        run_counter, requested_runs, lvl, scen_key, mode_name, seed);
                end
                
                try
                    [~, met, hist] = stage5_multivehicle_coordination(...
                        'scenario', scen_key, ...
                        'uncertainty_mode', mode_name, ...
                        'seed', seed, ...
                        'verbose', false);
                    
                    realism_met = RealismMetrics.compute(hist, struct(), SimulationConfig());
                    
                    completed_runs = completed_runs + 1;
                    
                    % Unique Trajectory MD5 Signature
                    traj_hash = compute_trajectory_hash(hist);
                    
                    r.level = lvl;
                    r.scen_key = scen_key;
                    r.mode = mode_name;
                    r.seed = seed;
                    r.traj_hash = traj_hash;
                    r.outcome = met.outcome;
                    r.lat_rmse = realism_met.tracking.lat_rmse;
                    r.min_clr = realism_met.safety.min_obstacle_clearance;
                    r.min_ttc = realism_met.traffic.min_ttc;
                    r.max_lat_accel = realism_met.motion.max_lat_accel_g;
                    r.max_yaw_rate = realism_met.motion.max_yaw_rate_deg;
                    r.emergency_count = met.emergency_braking_count;
                    r.qp_infeas = met.emergency_braking_count;
                    
                    results = [results; r];
                    
                    fprintf(fid_res, '%d,%d,%d,%d,%s,%d,%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d\n', ...
                        requested_runs, completed_runs, failed_runs, lvl, mode_name, seed, traj_hash, ...
                        met.outcome, r.lat_rmse, r.min_clr, r.min_ttc, ...
                        r.max_lat_accel, r.max_yaw_rate, r.emergency_count, r.qp_infeas);
                    
                    if ~strcmp(met.outcome, 'SUCCESS') && ~strcmp(met.outcome, 'DEGRADED_SAFE')
                        f.level = lvl; f.mode = mode_name; f.seed = seed; f.outcome = met.outcome;
                        f.min_clr = r.min_clr; f.lat_rmse = r.lat_rmse; f.hash = traj_hash;
                        failures = [failures; f];
                        fprintf(fid_fail, '%d,%d,%d,%d,%s,%d,%s,%s,%s,%.4f,%.4f\n', ...
                            requested_runs, completed_runs, failed_runs, lvl, mode_name, seed, traj_hash, ...
                            met.outcome, 'Safety/Infeasibility Trigger', r.min_clr, r.lat_rmse);
                    end
                catch ME
                    failed_runs = failed_runs + 1;
                    warning('Run %d failed with error: %s', run_counter, ME.message);
                end
            end
        end
    end
    
    fclose(fid_res);
    fclose(fid_fail);
    
    fprintf('\n========================================================================================\n');
    fprintf('                        MONTE CARLO EXPERIMENT ACCOUNTING REPORT                         \n');
    fprintf('========================================================================================\n');
    fprintf('  Requested Runs: %d\n', requested_runs);
    fprintf('  Completed Runs: %d\n', completed_runs);
    fprintf('  Failed Runs:    %d\n', failed_runs);
    fprintf('  Results CSV:    %s\n', res_csv_path);
    fprintf('  Failures CSV:   %s\n', fail_csv_path);
    fprintf('========================================================================================\n\n');
    
    % Render & Save Robustness Plots
    render_robustness_plots(results);
    
    % Print Summary Tables 1 & 2
    print_summary_tables(results, modes);
end

function str = compute_trajectory_hash(hist)
    data = [hist.ego_x, hist.ego_y, hist.ego_v, hist.ego_theta];
    bytes = typecast(data(:), 'uint8');
    md = java.security.MessageDigest.getInstance('MD5');
    md.update(bytes);
    hash_bytes = md.digest();
    str = sprintf('%02x', typecast(hash_bytes, 'uint8'));
    str = str(1:8);
end

function render_robustness_plots(results)
    if isempty(results), return; end
    
    fig1 = figure('Visible', 'off', 'Position', [100, 100, 800, 450]);
    outcomes = {'SUCCESS', 'DEGRADED_SAFE', 'SAFE_STOP', 'PLANNER_INFEASIBLE', 'UNSAFE_FAILURE', 'COLLISION'};
    counts_lvl = zeros(10, length(outcomes));
    for lvl = 1:10
        sub_r = results([results.level] == lvl);
        for o_i = 1:length(outcomes)
            counts_lvl(lvl, o_i) = sum(strcmp({sub_r.outcome}, outcomes{o_i}));
        end
    end
    bar(1:10, counts_lvl, 'stacked');
    xlabel('Scenario Level (1 - 10)'); ylabel('Run Count');
    title('Phase 12D/12E: Outcome Distribution vs Scenario Level');
    legend(outcomes, 'Location', 'eastoutside'); grid on;
    saveas(fig1, 'artifacts/phase12d_outcome_vs_level.png'); close(fig1);
    
    fig2 = figure('Visible', 'off', 'Position', [100, 100, 700, 400]);
    min_clrs = zeros(10, 1);
    for lvl = 1:10
        sub_r = results([results.level] == lvl);
        valid_c = [sub_r.min_clr]; valid_c = valid_c(~isinf(valid_c));
        if ~isempty(valid_c), min_clrs(lvl) = mean(valid_c); else, min_clrs(lvl) = 15.0; end
    end
    plot(1:10, min_clrs, 'b-o', 'LineWidth', 2, 'MarkerSize', 6);
    xlabel('Scenario Level'); ylabel('Mean Clearance (m)');
    title('Phase 12D/12E: Minimum Clearance vs Scenario Level'); grid on;
    saveas(fig2, 'artifacts/phase12d_clearance_vs_level.png'); close(fig2);
end

function print_summary_tables(results, modes)
    fprintf('========================================================================================================================\n');
    fprintf('                                REQUIRED FINAL TABLE 1: OUTCOME BREAKDOWN                                             \n');
    fprintf('========================================================================================================================\n');
    fprintf('| Level | Mode               | Runs | Success | Degraded Safe | Safe Stop | Planner Infeasible | Unsafe Failure | Collision |\n');
    fprintf('|-------|--------------------|------|---------|---------------|-----------|--------------------|----------------|-----------|\n');
    
    for lvl = 1:10
        for m_i = 1:length(modes)
            mn = modes{m_i};
            sub_r = results([results.level] == lvl & strcmp({results.mode}, mn));
            n_r = length(sub_r);
            if n_r == 0, continue; end
            
            n_succ = sum(strcmp({sub_r.outcome}, 'SUCCESS'));
            n_deg  = sum(strcmp({sub_r.outcome}, 'DEGRADED_SAFE'));
            n_stop = sum(strcmp({sub_r.outcome}, 'SAFE_STOP'));
            n_inf  = sum(strcmp({sub_r.outcome}, 'PLANNER_INFEASIBLE'));
            n_unsf = sum(strcmp({sub_r.outcome}, 'UNSAFE_FAILURE'));
            n_coll = sum(strcmp({sub_r.outcome}, 'COLLISION'));
            
            fprintf('| L%-4d | %-18s | %4d | %7d | %13d | %9d | %18d | %14d | %9d |\n', ...
                lvl, mn, n_r, n_succ, n_deg, n_stop, n_inf, n_unsf, n_coll);
        end
    end
    fprintf('========================================================================================================================\n\n');
    
    fprintf('===============================================================================================================\n');
    fprintf('                                REQUIRED FINAL TABLE 2: PERFORMANCE METRICS                                    \n');
    fprintf('===============================================================================================================\n');
    fprintf('| Level | Mode               | Min Clearance (m) | Min TTC (s) | Lat RMSE (m) | Max Lat Acc (g) | QP Infeas %% |\n');
    fprintf('|-------|--------------------|-------------------|-------------|--------------|-----------------|--------------|\n');
    
    for lvl = 1:10
        for m_i = 1:length(modes)
            mn = modes{m_i};
            sub_r = results([results.level] == lvl & strcmp({results.mode}, mn));
            if isempty(sub_r), continue; end
            
            clrs = [sub_r.min_clr]; valid_clr = clrs(~isinf(clrs));
            if isempty(valid_clr), m_clr = Inf; else, m_clr = min(valid_clr); end
            
            ttcs = [sub_r.min_ttc]; valid_ttc = ttcs(~isinf(ttcs));
            if isempty(valid_ttc), m_ttc = Inf; else, m_ttc = min(valid_ttc); end
            
            m_rmse = mean([sub_r.lat_rmse]);
            m_acc  = max([sub_r.max_lat_accel]);
            p_inf  = mean([sub_r.qp_infeas] > 0) * 100;
            
            fprintf('| L%-4d | %-18s | %17.4f | %11.4f | %12.4f | %15.2f | %11.1f%% |\n', ...
                lvl, mn, m_clr, m_ttc, m_rmse, m_acc, p_inf);
        end
    end
    fprintf('===============================================================================================================\n\n');
end
