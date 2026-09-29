% RUN_STANDARD_SCENARIO_EXPORT
% Runs Stage 5 closed-loop simulation on the Government-Compliant Autonomous Pipeline
% Benchmark with zero hardcoded values, pure perception-driven coordination,
% and dynamic corridor/VRU defense.
% Exports telemetry to web_viewer/data/handdrawn_village_standard.json

proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

output_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_standard.json');

fprintf('Exporting government-compliant standard scenario to %s ...\n', output_path);
export_simulation_json( ...
    'scenario', 'handdrawn_village_standard', ...
    'max_steps', 450, ...
    'seed', 42, ...
    'ego_v', 4.5, ...
    'uncertainty_mode', 'ideal', ...
    'output', output_path);

fprintf('Export complete: %s\n', output_path);
