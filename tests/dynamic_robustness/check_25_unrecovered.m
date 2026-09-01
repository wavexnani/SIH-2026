function check_25_unrecovered()
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common', 'tests/dynamic_robustness');

mat_file = 'tests/dynamic_robustness/results/dynamic_robustness_benchmark_results.mat';
data = load(mat_file);
metrics_cell = data.all_metrics{2};
n_trials = 50;

fprintf('\n========================================================================================\n');
fprintf('                  DETAILED AUDIT OF ALL 50 TRIALS IN SCENARIO 02                       \n');
fprintf('========================================================================================\n');
fprintf('%-5s | %-10s | %-8s | %-8s | %-8s | %-8s | %-8s | %-8s | %-8s | %-8s | %-15s\n', ...
    'Trial', 'Seed', 'Herd Vy', 't_block', 't_stop', 'x_stop', 't_reopen', 't_resume', 'Final X', 'Collision', 'Classification');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');

for tr = 1:n_trials
    m = metrics_cell{tr};
    trial_seed = uint32(42 + 2 * 1000 + tr);
    rng(trial_seed);
    herd_x = 50.0 + (rand() - 0.5) * 4.0;
    herd_vy = 0.50 + 0.10 * (rand() - 0.5);
    
    tb_str = '--'; if ~isnan(m.detection_time_s), tb_str = sprintf('%.2f', m.detection_time_s); end
    ts_str = '--'; if ~isnan(m.emergency_stop_time_s), ts_str = sprintf('%.2f', m.emergency_stop_time_s); end
    xs_str = '--'; if ~isnan(m.stop_position_m), xs_str = sprintf('%.2f', m.stop_position_m); end
    tr_str = '--'; if ~isnan(m.corridor_reopen_time_s), tr_str = sprintf('%.2f', m.corridor_reopen_time_s); end
    t_res_str = '--'; if ~isnan(m.resume_time_s), t_res_str = sprintf('%.2f', m.resume_time_s); end
    
    cls = 'RECOVERED';
    if m.scenario_obstacle_collision
        cls = 'A (Collision)';
    elseif ~m.safe_stop
        cls = 'B (No Safe Stop)';
    elseif isnan(m.corridor_reopen_time_s)
        cls = 'C (No Clear < 25s)';
    elseif isnan(m.resume_time_s)
        cls = 'E (No Resume)';
    end
    
    fprintf('%-5d | %-10d | %-8.3f | %-8s | %-8s | %-8s | %-8s | %-8s | %-8.2f | %-8s | %-15s\n', ...
        tr, trial_seed, herd_vy, tb_str, ts_str, xs_str, tr_str, t_res_str, m.ego_x_vec(end), ...
        num2str(m.scenario_obstacle_collision), cls);
end
end
