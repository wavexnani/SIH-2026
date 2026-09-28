% RUN_CORRECTED_SCENARIO_EXPORT
% Runs Stage 5 closed-loop simulation on the calibrated hand-drawn village scene
% with all inconsistencies, collisions, and tracking bugs resolved.
% Exports the complete telemetry to web_viewer/data/handdrawn_village_corrected.json

proj_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(proj_root, 'setup_paths.m'));

output_path = fullfile(proj_root, 'web_viewer', 'data', 'handdrawn_village_corrected.json');

fprintf('Exporting corrected hand-drawn village scenario to %s ...\n', output_path);
export_simulation_json( ...
    'scenario', 'handdrawn_village_corrected', ...
    'max_steps', 380, ...
    'seed', 42, ...
    'ego_v', 4.5, ...
    'uncertainty_mode', 'ideal', ...
    'output', output_path);

fprintf('Export complete: %s\n', output_path);
