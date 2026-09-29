% RUN_ADVANCED_SCENARIO_EXPORT
% Runs Stage 5 closed-loop simulation on the Advanced Benchmark:
% Severe 90° Blind Bend & Apex Conflict with oncoming heavy tractor and apex debris.
% Exports telemetry to web_viewer/data/handdrawn_village_advanced.json

proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

output_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_advanced.json');

fprintf('Exporting advanced 90-deg blind bend scenario to %s ...\n', output_path);
export_simulation_json( ...
    'scenario', 'handdrawn_village_advanced', ...
    'max_steps', 550, ...
    'seed', 42, ...
    'ego_v', 4.5, ...
    'uncertainty_mode', 'ideal', ...
    'output', output_path);

fprintf('Export complete: %s\n', output_path);
