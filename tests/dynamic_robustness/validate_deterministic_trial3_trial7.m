function validate_deterministic_trial3_trial7()
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', 'tests/dynamic_robustness/common', 'tests/dynamic_robustness');

fprintf('\n====================================================================================================================================\n');
fprintf('                             STEP 6: DETERMINISTIC VALIDATION TABLE (TRIALS 3 & 7)                                                 \n');
fprintf('====================================================================================================================================\n');

cfg = SimulationConfig();

[~, metrics_cell] = run_scenario_02_herd_clears(7);
m3 = metrics_cell{3};
m7 = metrics_cell{7};

fprintf('%-6s | %-7s | %-6s | %-18s | %-10s | %-15s | %-10s | %-6s | %-9s | %-7s\n', ...
    'Trial', 'Blocked', 'Stop', 'Min Herd Clearance', 'Herd Clear', 'Corridor Reopen', 'SF Release', 'Resume', 'Collision', 'Success');
fprintf('------------------------------------------------------------------------------------------------------------------------------------\n');

print_row(m3);
print_row(m7);

fprintf('====================================================================================================================================\n\n');
end

function print_row(m)
blocked_str = fmt_time(m.detection_time_s);
stop_str    = fmt_time(m.emergency_stop_time_s);
clear_str   = fmt_time(m.herd_clear_time_s);
reopen_str  = fmt_time(m.corridor_reopen_time_s);
sf_rel_str  = fmt_time(m.safety_filter_release_time_s);
resume_str  = fmt_time(m.resume_time_s);

coll_str = 'NO';
if m.scenario_obstacle_collision, coll_str = 'YES'; end

succ_str = 'FAIL';
if m.success, succ_str = 'PASS'; end

fprintf('%-6d | %-7s | %-6s | %18.2f m | %-10s | %-15s | %-10s | %-6s | %-9s | %-7s\n', ...
    m.trial_id, blocked_str, stop_str, m.minimum_herd_clearance_m, clear_str, reopen_str, sf_rel_str, resume_str, coll_str, succ_str);
end

function s = fmt_time(val)
if isnan(val)
    s = '--';
else
    s = sprintf('%.2f s', val);
end
end
