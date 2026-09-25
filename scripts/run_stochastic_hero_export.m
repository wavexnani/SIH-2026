% RUN_STOCHASTIC_HERO_EXPORT Run and export Stochastic Village Mixed Scenario to JSON for HTML Viewer
%
% Scenario: stochastic_village_mixed
% Seed: 42
% Description: Stochastic Unstructured Village Traffic Environment with heterogeneous
% road users (cars, bikes, autos, pedestrians, cattle), curves, grades, and potholes.

clear; clc;
fprintf('====================================================\n');
fprintf('  SIH26037 — STOCHASTIC VILLAGE TRAFFIC EXPORT     \n');
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

output_json = fullfile(proj_root, 'web_viewer', 'data', 'stochastic_scenario.json');

export_simulation_json(...
    'scenario', 'stochastic_village_mixed', ...
    'seed', 42, ...
    'max_steps', 150, ...
    'ego_v', 5.0, ...
    'uncertainty_mode', 'ideal', ...
    'output', output_json ...
);

fprintf('\nDone! The stochastic scenario JSON is exported to:\n  %s\n', output_json);
