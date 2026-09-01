classdef MetricsEvaluator
    % METRICSEVALUATOR Aggregates trial results and formats scientific performance tables (V3)
    
    methods (Static)
        function summary = summarizeScenario(scenario_name, metrics_cell)
            n_trials = length(metrics_cell);
            
            n_success = 0;
            n_scenario_collision = 0;
            n_env_collision = 0;
            n_global_collision = 0;
            n_recovery = 0;
            
            n_full_stop = 0;
            n_yield = 0;
            
            scen_clearances = zeros(n_trials, 1);
            env_clearances = zeros(n_trials, 1);
            glob_clearances = zeros(n_trials, 1);
            
            react_times = [];
            stop_positions = [];
            full_stop_latencies = [];
            yield_latencies = [];
            
            planner_prevented_sum = 0;
            sf_saved_sum = 0;
            
            for i = 1:n_trials
                m = metrics_cell{i};
                if m.success, n_success = n_success + 1; end
                if m.scenario_obstacle_collision, n_scenario_collision = n_scenario_collision + 1; end
                if m.unrelated_agent_collision, n_env_collision = n_env_collision + 1; end
                if m.global_collision, n_global_collision = n_global_collision + 1; end
                if m.recovery_success, n_recovery = n_recovery + 1; end
                
                if strcmp(m.recovery_mode, 'FULL_STOP') && m.recovery_success
                    n_full_stop = n_full_stop + 1;
                elseif strcmp(m.recovery_mode, 'DYNAMIC_YIELD') && m.recovery_success
                    n_yield = n_yield + 1;
                end
                
                scen_clearances(i) = m.minimum_herd_clearance_m;
                env_clearances(i)  = m.min_environment_clearance_m;
                glob_clearances(i) = m.min_global_clearance_m;
                
                if ~isnan(m.detection_time_s) && ~isnan(m.emergency_stop_time_s)
                    react_times = [react_times; m.emergency_stop_time_s - m.detection_time_s];
                end
                if ~isnan(m.stop_position_m), stop_positions = [stop_positions; m.stop_position_m]; end
                
                if ~isnan(m.full_stop_recovery_latency_s)
                    full_stop_latencies = [full_stop_latencies; m.full_stop_recovery_latency_s];
                end
                if ~isnan(m.yield_recovery_latency_s)
                    yield_latencies = [yield_latencies; m.yield_recovery_latency_s];
                end
                
                planner_prevented_sum = planner_prevented_sum + m.planner_prevented_count;
                sf_saved_sum = sf_saved_sum + m.safety_filter_saved_count;
            end
            
            summary = struct();
            summary.scenario_name = scenario_name;
            summary.n_trials = n_trials;
            summary.success_rate = (n_success / n_trials) * 100.0;
            summary.scenario_collision_free_rate = ((n_trials - n_scenario_collision) / n_trials) * 100.0;
            summary.global_collision_free_rate = ((n_trials - n_global_collision) / n_trials) * 100.0;
            summary.recovery_rate = (n_recovery / n_trials) * 100.0;
            
            summary.n_full_stop = n_full_stop;
            summary.n_yield = n_yield;
            summary.full_stop_recovery_rate = (n_full_stop / n_trials) * 100.0;
            summary.yield_recovery_rate = (n_yield / n_trials) * 100.0;
            
            summary.min_scenario_clearance = min(scen_clearances);
            summary.mean_scenario_clearance = mean(scen_clearances(isfinite(scen_clearances)));
            summary.min_global_clearance = min(glob_clearances);
            
            summary.mean_reaction_time = mean_or_nan(react_times);
            summary.mean_stop_position = mean_or_nan(stop_positions);
            
            % Full-stop latency stats
            summary.full_stop_lat_stats = calc_stats(full_stop_latencies);
            % Yield latency stats
            summary.yield_lat_stats = calc_stats(yield_latencies);
            
            summary.planner_prevention_pct = (planner_prevented_sum / max(1, planner_prevented_sum + sf_saved_sum)) * 100.0;
            summary.sf_saved_pct = (sf_saved_sum / max(1, planner_prevented_sum + sf_saved_sum)) * 100.0;
        end
        
        function printSummaryTable(summaries)
            fprintf('\n==================================================================================================================\n');
            fprintf('                               CA-CRC DYNAMIC ROBUSTNESS BENCHMARK SUMMARY (V3)                                   \n');
            fprintf('==================================================================================================================\n');
            fprintf('%-24s | %-6s | %-8s | %-12s | %-12s | %-12s | %-10s | %-12s\n', ...
                'Scenario', 'Trials', 'Success', 'Scen-Free', 'Glob-Free', 'RecoveryRate', 'MinScenClr', 'PlannerPrev%');
            fprintf('------------------------------------------------------------------------------------------------------------------\n');
            
            for i = 1:length(summaries)
                s = summaries{i};
                fprintf('%-24s | %6d | %7.1f%% | %11.1f%% | %11.1f%% | %11.1f%% | %10.2f m | %11.1f%%\n', ...
                    s.scenario_name, s.n_trials, s.success_rate, s.scenario_collision_free_rate, ...
                    s.global_collision_free_rate, s.recovery_rate, s.min_scenario_clearance, s.planner_prevention_pct);
            end
            fprintf('==================================================================================================================\n\n');
        end
    end
end

function st = calc_stats(arr)
st = struct('count', 0, 'mean', NaN, 'median', NaN, 'min', NaN, 'max', NaN, 'std', NaN);
if ~isempty(arr)
    st.count = length(arr);
    st.mean = mean(arr);
    st.median = median(arr);
    st.min = min(arr);
    st.max = max(arr);
    if length(arr) > 1
        st.std = std(arr);
    else
        st.std = 0.0;
    end
end
end

function val = mean_or_nan(arr)
if isempty(arr)
    val = NaN;
else
    val = mean(arr);
end
end
