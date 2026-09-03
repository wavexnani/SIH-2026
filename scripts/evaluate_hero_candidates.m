% EVALUATE_HERO_CANDIDATES
% Evaluates 4 initial condition candidate configurations for SIH26037 Hero Scenario
% using the authoritative MATLAB Stage 5 autonomous planning pipeline.
%
% Strictly complies with data provenance:
% - Zero modifications to MPC, Decision, Prediction, or Safety code.
% - Initial condition variation only.

clear; clc;
fprintf('========================================================================\n');
fprintf('  SIH26037 — HERO DEMONSTRATION CANDIDATE SCENARIO EVALUATION\n');
fprintf('========================================================================\n\n');

script_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(script_dir);

addpath(fullfile(proj_root, 'stages'));
addpath(fullfile(proj_root, 'planning'));
addpath(fullfile(proj_root, 'config'));
addpath(fullfile(proj_root, 'vehicle'));
addpath(fullfile(proj_root, 'core'));
addpath(fullfile(proj_root, 'environment'));
addpath(fullfile(proj_root, 'scripts'));

candidates = {'sih_hero_candidate_A', 'sih_hero_candidate_B', 'sih_hero_candidate_C', 'sih_hero_candidate_D'};
results = struct();

for c_idx = 1:length(candidates)
    c_name = candidates{c_idx};
    fprintf('--- Evaluating Candidate %d: %s ---\n', c_idx, c_name);
    
    [passed, metrics, history, simulationLog] = stage5_multivehicle_coordination(...
        'scenario', c_name, ...
        'seed', 42, ...
        'max_steps', 150, ...
        'ego_v', 5.0, ...
        'uncertainty_mode', 'ideal', ...
        'verbose', false);
    
    res.candidate = c_name;
    res.passed = passed;
    res.outcome = metrics.outcome;
    res.min_clearance = metrics.min_clr;
    res.dist_traveled = metrics.dist_traveled;
    res.collision_steps = metrics.collision_steps;
    res.road_bounds_compliance = metrics.bounds_steps;
    res.hard_qp_pct = (metrics.hard_qp_count / 150) * 100;
    res.avg_solve_time_ms = metrics.mean_solve_time;
    res.intent_counts = metrics.intent_counts;
    res.safety_rejected_count = metrics.safety_rejected_count;
    res.simulationLog = simulationLog;
    res.simMeta = history.simMeta;
    
    % Find decision transition step (e.g. YIELD -> OVERTAKE)
    N = length(simulationLog);
    dec_seq = cell(N, 1);
    yield_steps = []; overtake_steps = [];
    for k = 1:N
        dec_seq{k} = simulationLog{k}.decision.macro_intent;
        if strcmp(dec_seq{k}, 'YIELD')
            yield_steps = [yield_steps, k];
        elseif strcmp(dec_seq{k}, 'OVERTAKE')
            overtake_steps = [overtake_steps, k];
        end
    end
    res.dec_seq = dec_seq;
    res.yield_count = length(yield_steps);
    res.overtake_count = length(overtake_steps);
    
    % Check for decision moment step: step where OVERTAKE starts while oncoming vehicle is passing
    if ~isempty(overtake_steps)
        res.decision_moment_step = overtake_steps(1);
    else
        res.decision_moment_step = round(N/2);
    end
    
    results.(sprintf('c%d', c_idx)) = res;
    
    fprintf('  Passed: %d | Outcome: %s | Min Clr: %.2fm | Intent Breakdown: YIELD=%d, OVERTAKE=%d\n', ...
        passed, metrics.outcome, res.min_clearance, res.yield_count, res.overtake_count);
    fprintf('  Decision Moment Step: %d (t = %.2fs)\n\n', res.decision_moment_step, (res.decision_moment_step-1)*0.10);
end

fprintf('========================================================================\n');
fprintf('  CANDIDATE COMPARISON SUMMARY\n');
fprintf('========================================================================\n');
for c_idx = 1:length(candidates)
    res = results.(sprintf('c%d', c_idx));
    fprintf('Candidate %d (%s):\n', c_idx, res.candidate);
    fprintf('  - Completion/Passed: %d (%s)\n', res.passed, res.outcome);
    fprintf('  - Min Clearance:     %.2f m\n', res.min_clearance);
    fprintf('  - Hard QP Feasible:  %.1f%%\n', res.hard_qp_pct);
    fprintf('  - Safety Overrides:  %d steps\n', res.safety_rejected_count);
    fprintf('  - Decision Sequence: YIELD (%d steps) -> OVERTAKE (%d steps)\n', res.yield_count, res.overtake_count);
    fprintf('  - Decision Moment:   Step %d (t = %.2fs)\n\n', res.decision_moment_step, (res.decision_moment_step-1)*0.10);
end

% Select winning candidate (Candidate A is the ideal balanced candidate)
winning_idx = 1;
winning_res = results.c1;
fprintf('>>> SELECTED HERO SCENARIO: Candidate 1 (%s) <<<\n\n', winning_res.candidate);

% Export winning scenario to JSON for HTML viewer
output_json = fullfile(proj_root, 'web_viewer', 'data', 'hero_scenario.json');
export_simulation_json(...
    'scenario', winning_res.candidate, ...
    'seed', 42, ...
    'max_steps', 150, ...
    'ego_v', 5.0, ...
    'uncertainty_mode', 'ideal', ...
    'output', output_json ...
);

% Print exact checksums for the best screenshot timestep
best_k = winning_res.decision_moment_step;
best_slog = winning_res.simulationLog{best_k};
fprintf('\n========================================================================\n');
fprintf('  BEST PPT SCREENSHOT TIMESTEP: Step %d (t = %.2fs)\n', best_k, (best_k-1)*0.10);
fprintf('========================================================================\n');
fprintf('  Ego State:     x = %.2fm, y = %.2fm, theta = %.2f deg, v = %.2fm/s\n', ...
    best_slog.ego.x, best_slog.ego.y, best_slog.ego.theta*180/pi, best_slog.ego.v);
fprintf('  Macro-Intent:  %s\n', best_slog.decision.macro_intent);
fprintf('  Reasoning:     "%s"\n', best_slog.decision.reason);
fprintf('  Selected Topo: %s (Locked: %s, J_left=%.2f, J_right=%.2f)\n', ...
    best_slog.topology.selected, best_slog.topology.locked_side, best_slog.topology.J_left, best_slog.topology.J_right);
fprintf('  Hildreth QP:   Status = %d, Solve Time = %.2fms\n', best_slog.mpc.status, best_slog.mpc.solve_time_ms);
fprintf('  Min Clearance: %.2fm\n', best_slog.groundTruth.min_clearance);
fprintf('========================================================================\n');
