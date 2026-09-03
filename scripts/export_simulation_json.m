function export_simulation_json(varargin)
    % EXPORT_SIMULATION_JSON Run Stage 5 pipeline and export simulationLog to JSON
    %
    % Usage:
    %   export_simulation_json()                          % Default hero scenario
    %   export_simulation_json('scenario', 'multi_vehicle_following')
    %   export_simulation_json('seed', 99, 'max_steps', 200)
    %   export_simulation_json('output', 'web_viewer/data/custom.json')
    %
    % The exported JSON is the ONLY data source for the HTML viewer.
    % The HTML viewer must NOT simulate, calculate, or invent any values.
    
    p = inputParser;
    addParameter(p, 'scenario', 'multi_vehicle_yield_overtake', @ischar);
    addParameter(p, 'seed', 42, @isnumeric);
    addParameter(p, 'max_steps', 150, @isnumeric);
    addParameter(p, 'ego_v', 5.0, @isnumeric);
    addParameter(p, 'uncertainty_mode', 'ideal', @ischar);
    addParameter(p, 'output', '', @ischar);
    parse(p, varargin{:});
    
    scenario = p.Results.scenario;
    seed = p.Results.seed;
    max_steps = p.Results.max_steps;
    ego_v = p.Results.ego_v;
    unc_mode = p.Results.uncertainty_mode;
    output_path = p.Results.output;
    
    if isempty(output_path)
        output_path = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'web_viewer', 'data', 'hero_scenario.json');
    end
    
    % Ensure output directory exists
    [out_dir, ~, ~] = fileparts(output_path);
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    
    fprintf('\n=== PIPELINE JSON EXPORT ===\n');
    fprintf('Scenario:    %s\n', scenario);
    fprintf('Seed:        %d\n', seed);
    fprintf('Max Steps:   %d\n', max_steps);
    fprintf('Uncertainty: %s\n', unc_mode);
    fprintf('Output:      %s\n\n', output_path);
    
    % Run the actual MATLAB pipeline
    addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'stages'));
    addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'planning'));
    addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'config'));
    addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'vehicle'));
    addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'core'));
    addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'environment'));
    
    [passed, metrics, history, simulationLog] = stage5_multivehicle_coordination( ...
        'scenario', scenario, ...
        'seed', seed, ...
        'max_steps', max_steps, ...
        'ego_v', ego_v, ...
        'uncertainty_mode', unc_mode, ...
        'verbose', true);
    
    % Build export structure
    exportData.metadata = history.simMeta;
    exportData.metadata.export_timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
    exportData.metadata.matlab_version = version;
    exportData.metadata.outcome = metrics.outcome;
    exportData.metadata.passed = passed;
    
    % Convert simulationLog to cell array for clean JSON
    N = length(simulationLog);
    exportData.timesteps = cell(N, 1);
    for k = 1:N
        exportData.timesteps{k} = simulationLog{k};
    end
    
    % Encode and write JSON
    json_str = jsonencode(exportData);
    
    fid = fopen(output_path, 'w');
    if fid == -1
        error('Failed to open output file: %s', output_path);
    end
    fwrite(fid, json_str, 'char');
    fclose(fid);
    
    fprintf('\n=== EXPORT COMPLETE ===\n');
    fprintf('File:        %s\n', output_path);
    fprintf('Size:        %.1f KB\n', length(json_str) / 1024);
    fprintf('Timesteps:   %d\n', N);
    fprintf('Outcome:     %s\n', metrics.outcome);
    fprintf('Passed:      %d\n', passed);
    
    % Validation checksums
    fprintf('\n--- Validation Checksums (for HTML verification) ---\n');
    fprintf('Step 1:   ego_x=%.4f  ego_y=%.4f  intent=%s\n', ...
        simulationLog{1}.ego.x, simulationLog{1}.ego.y, simulationLog{1}.decision.macro_intent);
    mid = round(N/2);
    fprintf('Step %d: ego_x=%.4f  ego_y=%.4f  intent=%s\n', mid, ...
        simulationLog{mid}.ego.x, simulationLog{mid}.ego.y, simulationLog{mid}.decision.macro_intent);
    fprintf('Step %d: ego_x=%.4f  ego_y=%.4f  intent=%s\n', N, ...
        simulationLog{N}.ego.x, simulationLog{N}.ego.y, simulationLog{N}.decision.macro_intent);
    fprintf('=== END ===\n\n');
end
