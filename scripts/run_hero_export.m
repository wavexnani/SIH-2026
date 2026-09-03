% RUN_HERO_EXPORT Run and export Hero Demonstration Scenario to JSON for HTML Viewer
%
% Scenario: multi_vehicle_yield_overtake
% Seed: 42
% Description: Authoritative Stage 5 pipeline execution export for SIH26037.

clear; clc;
fprintf('====================================================\n');
fprintf('  SIH26037 — HERO DEMONSTRATION PIPELINE EXPORT\n');
fprintf('====================================================\n');

script_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(script_dir);

addpath(fullfile(proj_root, 'stages'));
addpath(fullfile(proj_root, 'planning'));
addpath(fullfile(proj_root, 'config'));
addpath(fullfile(proj_root, 'vehicle'));
addpath(fullfile(proj_root, 'core'));
addpath(fullfile(proj_root, 'environment'));
addpath(fullfile(proj_root, 'scripts'));

output_json = fullfile(proj_root, 'web_viewer', 'data', 'hero_scenario.json');

export_simulation_json(...
    'scenario', 'multi_vehicle_yield_overtake', ...
    'seed', 42, ...
    'max_steps', 150, ...
    'ego_v', 5.0, ...
    'uncertainty_mode', 'ideal', ...
    'output', output_json ...
);

fprintf('\nDone! The hero scenario JSON is exported to:\n  %s\n', output_json);
