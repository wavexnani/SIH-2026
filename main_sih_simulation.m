% ===================================================================
% SIH INTERNAL HACKATHON - MAIN ORCHESTRATION SCRIPT
% Stages 0-5: Complete closed-loop simulation with CA-CRC planner
% Target: September 1, 2026
% ===================================================================

%% QUICK START GUIDE
% 
% This script runs the complete pipeline:
% 1. Setup simulation infrastructure (Stage 0)
% 2. Validate closed-loop control (Stage 1)
% 3. Run baseline simple planner (Stage 2)
% 4. Run QP-MPC baseline (Stage 3)
% 5. Run CA-CRC proposed planner (Stage 4)
% 6. Generate comprehensive metrics & comparison (Stage 5)
%
% Usage:
%   >> main_sih_simulation
%
% Configuration: Edit SimulationConfig.m for parameters
%
% Expected Results:
%   - 3 trajectory plots (one per planner)
%   - Metric comparison table
%   - Performance visualization dashboard
%   - CSV export of all metrics
%

%% MAIN ORCHESTRATION FUNCTION

function main_sih_simulation(varargin)
    
    % === SET UP EXPLICIT MATLAB PATHS ===
    projectRoot = fileparts(mfilename('fullpath'));
    if isempty(projectRoot)
        projectRoot = pwd;
    end
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'core'));
    addpath(fullfile(projectRoot, 'environment'));
    addpath(fullfile(projectRoot, 'vehicle'));
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'metrics'));
    addpath(fullfile(projectRoot, 'stages'));
    
    % === PARSE INPUTS ===
    p = inputParser();
    addParameter(p, 'stage', 'all', @(x) ismember(x, {'0', '1', '2', '3', '4', '5', 'all'}));
    addParameter(p, 'scenario', 'moderate', @ischar);
    addParameter(p, 'use_roadrunner', [], @(x) islogical(x) || isempty(x));  % Auto-detect if empty
    addParameter(p, 'verbose', true, @islogical);
    addParameter(p, 'save_results', true, @islogical);
    addParameter(p, 'export_csv', true, @islogical);
    
    parse(p, varargin{:});
    
    STAGE = p.Results.stage;
    SCENARIO = p.Results.scenario;
    VERBOSE = p.Results.verbose;
    SAVE_RESULTS = p.Results.save_results;
    EXPORT_CSV = p.Results.export_csv;
    
    % Auto-detect RoadRunner
    use_rr = p.Results.use_roadrunner;
    if isempty(use_rr)
        use_rr = check_roadrunner_available();
    end
    
    if VERBOSE
        fprintf('\n');
        fprintf(repmat('=', 1, 80) + "\n");
        fprintf('    SIH INTERNAL HACKATHON - STAGES 0-5 SIMULATION\n');
        fprintf('    Target Date: September 1, 2026\n');
        fprintf(repmat('=', 1, 80) + "\n");
        fprintf('Configuration:\n');
        fprintf('  - Stage: %s\n', STAGE);
        fprintf('  - Scenario: %s\n', SCENARIO);
        fprintf('  - RoadRunner: %s\n', string(use_rr));
        fprintf('  - Verbose: %s\n', string(VERBOSE));
        fprintf('\n');
    end
    
    % === STAGE 0: AUTHORITATIVE INFRASTRUCTURE VALIDATION ===
    % Stage 0 is implemented by stages/stage0_setup.m.  Keeping this
    % dispatch here ensures the master entry point uses the same
    % WorldState/EgoState model as the standalone validation script.
    if strcmp(STAGE, '0')
        if VERBOSE
            fprintf('STAGE 0: Validating simulation infrastructure...\n');
        end
        stage0_setup('scenario', SCENARIO, 'verbose', VERBOSE);
        return;
    end

    % === STAGE 1: CLOSED-LOOP VEHICLE CONTROL ===
    if strcmp(STAGE, '1')
        if VERBOSE
            fprintf('STAGE 1: Executing closed-loop vehicle control...\n');
        end
        scen_s1 = SCENARIO;
        if ~any(strcmpi(varargin, 'scenario'))
            scen_s1 = 'clear';
        end
        stage1_closed_loop('scenario', scen_s1, 'verbose', VERBOSE);
        return;
    end

    % === STAGE 2: BASELINE CANDIDATE PLANNER ===
    if strcmp(STAGE, '2')
        if VERBOSE
            fprintf('STAGE 2: Executing baseline candidate planner...\n');
        end
        stage2_baseline('scenario', SCENARIO, 'verbose', VERBOSE);
        return;
    end

    % === STAGE 3: LINEARIZED QP-MPC BASELINE PLANNER ===
    if strcmp(STAGE, '3')
        if VERBOSE
            fprintf('STAGE 3: Executing linearized QP-MPC baseline planner...\n');
        end
        stage3_qpmpc_baseline('scenario', SCENARIO, 'verbose', VERBOSE);
        return;
    end

    % === STAGE 4: CONTEXT-ADAPTIVE CRC + HARD SAFETY LAYER ===
    if strcmp(STAGE, '4')
        if VERBOSE
            fprintf('STAGE 4: Executing Context-Adaptive CRC + Hard Safety Layer...\n');
        end
        stage4_cacrc_safety('scenario', SCENARIO, 'verbose', VERBOSE);
        return;
    end

    % === INITIALIZE ===
    config = initialize_configuration();
    scenario_def = define_scenario(SCENARIO);
    
    % === STAGE 0: SIMULATION INFRASTRUCTURE ===
    if ismember(STAGE, {'0', 'all'})
        if VERBOSE, fprintf('STAGE 0: Initializing simulation infrastructure...\n'); end
        
        if use_rr
            [world_model, roadrunner_scenario] = initialize_with_roadrunner(config, scenario_def);
        else
            [world_model, roadrunner_scenario] = initialize_without_roadrunner(config, scenario_def);
        end
        
        if VERBOSE, fprintf('✓ Stage 0 complete: World model initialized\n\n'); end
        
        if strcmp(STAGE, '0')
            return;
        end
    end
    
    % === STAGE 1: CLOSED-LOOP VEHICLE ===
    if ismember(STAGE, {'1', 'all'})
        if VERBOSE, fprintf('STAGE 1: Validating closed-loop vehicle control...\n'); end
        
        [results_stage1, log_stage1] = run_stage1_closed_loop(...
            config, world_model, scenario_def, use_rr);
        
        if VERBOSE
            fprintf('✓ Stage 1 complete:\n');
            fprintf('  Distance traveled: %.2f m\n', ...
                sqrt((log_stage1.x(end)-log_stage1.x(1))^2 + ...
                     (log_stage1.y(end)-log_stage1.y(1))^2));
            fprintf('  Collisions: %d\n', sum(log_stage1.collision));
            fprintf('  Min clearance: %.2f m\n\n', min(log_stage1.clearance));
        end
        
        if strcmp(STAGE, '1')
            return;
        end
    end
    
    % === STAGE 2: BASELINE SIMPLE PLANNER ===
    if ismember(STAGE, {'2', 'all'})
        if VERBOSE, fprintf('STAGE 2: Running baseline simple planner...\n'); end
        
        [results_baseline, log_baseline] = run_stage2_baseline_planning(...
            config, world_model, scenario_def, use_rr);
        
        if VERBOSE
            fprintf('✓ Stage 2 complete:\n');
            fprintf('  Collision rate: %.2f%%\n', results_baseline.collision_rate * 100);
            fprintf('  Min clearance: %.2f m\n', results_baseline.min_clearance);
            fprintf('  Completion rate: %.2f%%\n\n', results_baseline.completion_rate * 100);
        end
        
        if strcmp(STAGE, '2')
            return;
        end
    end
    
    % === STAGE 3: QP-MPC BASELINE ===
    if ismember(STAGE, {'3', 'all'})
        if VERBOSE, fprintf('STAGE 3: Running QP-MPC baseline...\n'); end
        
        [results_qpmpc, log_qpmpc] = run_stage3_qpmpc_baseline(...
            config, world_model, scenario_def, use_rr);
        
        if VERBOSE
            fprintf('✓ Stage 3 complete:\n');
            fprintf('  Collision rate: %.2f%%\n', results_qpmpc.collision_rate * 100);
            fprintf('  Avg replan time: %.3f ms\n', results_qpmpc.avg_replan_time * 1000);
            fprintf('  Max replan time: %.3f ms\n\n', results_qpmpc.max_replan_time * 1000);
        end
        
        if strcmp(STAGE, '3')
            return;
        end
    end
    
    % === STAGE 4: CA-CRC PLANNER ===
    if ismember(STAGE, {'4', 'all'})
        if VERBOSE, fprintf('STAGE 4: Running CA-CRC proposed planner...\n'); end
        
        [results_carc, log_carc] = run_stage4_carc_planner(...
            config, world_model, scenario_def, use_rr);
        
        if VERBOSE
            fprintf('✓ Stage 4 complete:\n');
            fprintf('  Collision rate: %.2f%%\n', results_carc.collision_rate * 100);
            fprintf('  Avg replan time: %.3f ms\n', results_carc.avg_replan_time * 1000);
            fprintf('  Safety filter activations: %d\n\n', results_carc.safety_filter_activations);
        end
        
        if strcmp(STAGE, '4')
            return;
        end
    end
    
    % === STAGE 5: METRICS & COMPARISON ===
    if ismember(STAGE, {'5', 'all'})
        if VERBOSE, fprintf('STAGE 5: Generating comprehensive metrics and comparison...\n'); end
        
        % Ensure all results are available
        if ~exist('results_baseline', 'var')
            [results_baseline, log_baseline] = run_stage2_baseline_planning(...
                config, world_model, scenario_def, use_rr);
        end
        if ~exist('results_qpmpc', 'var')
            [results_qpmpc, log_qpmpc] = run_stage3_qpmpc_baseline(...
                config, world_model, scenario_def, use_rr);
        end
        if ~exist('results_carc', 'var')
            [results_carc, log_carc] = run_stage4_carc_planner(...
                config, world_model, scenario_def, use_rr);
        end
        
        % Run Stage 5
        results_comparison = run_stage5_metrics_comparison(...
            config, log_baseline, log_qpmpc, log_carc);
        
        if VERBOSE
            fprintf('✓ Stage 5 complete: Metrics computed and visualized\n\n');
        end
        
        % === SAVE RESULTS ===
        if SAVE_RESULTS
            save_all_results(config, results_baseline, results_qpmpc, results_carc, ...
                           log_baseline, log_qpmpc, log_carc, results_comparison);
            if VERBOSE
                fprintf('✓ Results saved to workspace\n');
            end
        end
        
        % === EXPORT TO CSV ===
        if EXPORT_CSV
            export_metrics_to_csv(results_comparison);
            if VERBOSE
                fprintf('✓ Metrics exported to CSV files\n');
            end
        end
    end
    
    if VERBOSE
        fprintf('\n');
        fprintf(repmat('=', 1, 80) + "\n");
        fprintf('    SIMULATION COMPLETE\n');
        fprintf(repmat('=', 1, 80) + "\n\n");
    end
    
end

%% ===================================================================
% UTILITY FUNCTIONS
% ===================================================================

% Check if RoadRunner is available
function available = check_roadrunner_available()
    try
        % Try to list RoadRunner apps
        rr_apps = driving.roadrunner.RoadRunnerApp.getOpenInstances();
        available = ~isempty(rr_apps);
        
        if available
            fprintf('✓ RoadRunner detected\n');
        else
            fprintf('⚠ RoadRunner not currently open (will use fallback)\n');
        end
    catch
        fprintf('⚠ RoadRunner not available (will use MATLAB-only simulation)\n');
        available = false;
    end
end

% Initialize configuration with all parameters
function config = initialize_configuration()
    config.dt = 0.1;
    config.T_horizon = 10;
    config.T_replan = 0.5;
    config.frames_per_step = config.T_replan / config.dt;
    
    % Road
    config.road.width = 6.0;
    config.road.length = 100.0;
    config.road.boundary_type = "unclear";
    
    % Vehicle
    config.vehicle.L = 4.7;
    config.vehicle.W = 1.8;
    config.vehicle.max_speed = 20.0;
    config.vehicle.max_accel = 3.0;
    config.vehicle.max_decel = -6.0;
    config.vehicle.wheelbase = 2.7;
    config.vehicle.max_steering = deg2rad(35);
    
    % Agents
    config.max_agents = 10;
end

% Define scenario
function scenario = define_scenario(scenario_name)
    switch scenario_name
        case 'unmarked_village'
            scenario.name = 'Unmarked Village Road';
            scenario.description = 'Simple village road with parked vehicles and pedestrian crossing';
            scenario.duration = 10;
            
            % Ego vehicle
            scenario.ego.x0 = 10;
            scenario.ego.y0 = 0;
            scenario.ego.theta0 = 0;
            scenario.ego.v0 = 0;
            
            % Road
            scenario.road.left = [0, -3; 100, -3];
            scenario.road.right = [0, 3; 100, 3];
            
            % Dynamic agents
            scenario.agents(1).type = 'pedestrian';
            scenario.agents(1).x0 = 65;
            scenario.agents(1).y0 = -2;
            scenario.agents(1).theta0 = deg2rad(45);
            scenario.agents(1).v0 = 0.5;
            scenario.agents(1).length = 0.5;
            scenario.agents(1).width = 0.4;
            
            scenario.agents(2).type = 'bicycle';
            scenario.agents(2).x0 = 55;
            scenario.agents(2).y0 = 1.5;
            scenario.agents(2).theta0 = 0;
            scenario.agents(2).v0 = 2.0;
            scenario.agents(2).length = 1.8;
            scenario.agents(2).width = 0.6;
            
            scenario.agents(3).type = 'auto';
            scenario.agents(3).x0 = 75;
            scenario.agents(3).y0 = -1.5;
            scenario.agents(3).theta0 = deg2rad(-30);
            scenario.agents(3).v0 = 1.0;
            scenario.agents(3).length = 3.5;
            scenario.agents(3).width = 1.5;
            
            % Static obstacles
            scenario.static(1).type = 'car';
            scenario.static(1).x = 50;
            scenario.static(1).y = 1.5;
            scenario.static(1).length = 4.7;
            scenario.static(1).width = 1.8;
            
            scenario.static(2).type = 'car';
            scenario.static(2).x = 85;
            scenario.static(2).y = -2.0;
            scenario.static(2).length = 4.7;
            scenario.static(2).width = 1.8;
            
            scenario.static(3).type = 'debris';
            scenario.static(3).x = 40;
            scenario.static(3).y = 0.5;
            scenario.static(3).length = 0.8;
            scenario.static(3).width = 0.5;
            
            scenario.static(4).type = 'cart';
            scenario.static(4).x = 60;
            scenario.static(4).y = -2.5;
            scenario.static(4).length = 2.0;
            scenario.static(4).width = 1.2;
            
            % Road anomalies
            scenario.anomalies(1).type = 'pothole';
            scenario.anomalies(1).x = 80;
            scenario.anomalies(1).y = 0.5;
            scenario.anomalies(1).severity = 0.7;
            scenario.anomalies(1).depth = 0.15;
            
        otherwise
            error('Unknown scenario: %s', scenario_name);
    end
end

% Initialize with RoadRunner
function [world_model, scenario] = initialize_with_roadrunner(config, scenario_def)
    % Create roadrunner scenario programmatically
    scenario = roadrunnerScenario;
    scenario.SampleTime = config.dt;
    scenario.StopTime = config.T_horizon;
    
    % Add actors based on scenario definition
    % (RoadRunner-specific initialization)
    
    % For now, create dummy world model
    world_model = initialize_world_model_base(config, scenario_def);
end

% Initialize without RoadRunner (MATLAB-only)
function [world_model, scenario] = initialize_without_roadrunner(config, scenario_def)
    scenario = struct();
    scenario.type = 'matlab_simulation';
    scenario.step_func = @(t) step_agents(t, scenario_def);
    
    world_model = initialize_world_model_base(config, scenario_def);
end

% Base world model initialization
function world_model = initialize_world_model_base(config, scenario_def)
    world_model = struct();
    
    % Ego vehicle
    world_model.ego.x = scenario_def.ego.x0;
    world_model.ego.y = scenario_def.ego.y0;
    world_model.ego.theta = scenario_def.ego.theta0;
    world_model.ego.v = scenario_def.ego.v0;
    world_model.ego.delta = 0;
    world_model.ego.a = 0;
    world_model.ego.reference_path = build_reference_path();
    
    % Dynamic agents
    n_agents = length(scenario_def.agents);
    for i = 1:n_agents
        world_model.agents(i).type = scenario_def.agents(i).type;
        world_model.agents(i).x = scenario_def.agents(i).x0;
        world_model.agents(i).y = scenario_def.agents(i).y0;
        world_model.agents(i).theta = scenario_def.agents(i).theta0;
        world_model.agents(i).v = scenario_def.agents(i).v0;
        world_model.agents(i).length = scenario_def.agents(i).length;
        world_model.agents(i).width = scenario_def.agents(i).width;
    end
    
    % Static obstacles
    n_static = length(scenario_def.static);
    for i = 1:n_static
        world_model.static(i).type = scenario_def.static(i).type;
        world_model.static(i).x = scenario_def.static(i).x;
        world_model.static(i).y = scenario_def.static(i).y;
        world_model.static(i).length = scenario_def.static(i).length;
        world_model.static(i).width = scenario_def.static(i).width;
    end
    
    % Road anomalies
    n_anomalies = length(scenario_def.anomalies);
    for i = 1:n_anomalies
        world_model.anomalies(i).type = scenario_def.anomalies(i).type;
        world_model.anomalies(i).x = scenario_def.anomalies(i).x;
        world_model.anomalies(i).y = scenario_def.anomalies(i).y;
        world_model.anomalies(i).severity = scenario_def.anomalies(i).severity;
    end
    
    % Road
    world_model.road.left = scenario_def.road.left;
    world_model.road.right = scenario_def.road.right;
end

% Build reference path
function path = build_reference_path()
    x = linspace(10, 95, 100)';
    y = 0.5 * sin(x / 10);
    theta = atan2(diff(y), diff(x));
    theta = [theta; theta(end)];
    path = [x, y, theta];
end

% Step agents forward in time
function state = step_agents(t, scenario_def)
    % Update agent positions based on time
    % (Constant velocity for now)
end

% Placeholder Stage functions (will be filled in)
function [results, log] = run_stage1_closed_loop(config, world_model, scenario_def, use_rr)
    % Stage 1 implementation
    results = struct();
    log = struct();
end

function [results, log] = run_stage2_baseline_planning(config, world_model, scenario_def, use_rr)
    % Stage 2 implementation
    results = struct();
    log = struct();
end

function [results, log] = run_stage3_qpmpc_baseline(config, world_model, scenario_def, use_rr)
    % Stage 3 implementation
    results = struct();
    log = struct();
end

function [results, log] = run_stage4_carc_planner(config, world_model, scenario_def, use_rr)
    % Stage 4 Context-Adaptive Risk-Sensitive Control + Safety Layer
    [passed, metrics] = stage4_cacrc_safety('scenario', 'moderate', 'verbose', false);
    results = metrics;
    log = struct('passed', passed);
end

function results = run_stage5_metrics_comparison(config, log_baseline, log_qpmpc, log_carc)
    % Stage 5 implementation
    results = struct();
end

% Save all results
function save_all_results(config, res_base, res_qpmpc, res_carc, log_base, log_qpmpc, log_carc, comp)
    % Save workspace variables
    timestamp = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
    filename = sprintf('sih_results_%s.mat', timestamp);
    save(filename, 'config', 'res_base', 'res_qpmpc', 'res_carc', ...
         'log_base', 'log_qpmpc', 'log_carc', 'comp');
    fprintf('Results saved to: %s\n', filename);
end

% Export metrics to CSV
function export_metrics_to_csv(results)
    timestamp = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
    
    % Create metrics table
    metrics_table = table(...
        {'Baseline'; 'QP-MPC'; 'CA-CRC'}, ...
        [results.baseline.collision_rate * 100; ...
         results.qpmpc.collision_rate * 100; ...
         results.carc.collision_rate * 100], ...
        [results.baseline.min_clearance; ...
         results.qpmpc.min_clearance; ...
         results.carc.min_clearance], ...
        [results.baseline.completion_rate * 100; ...
         results.qpmpc.completion_rate * 100; ...
         results.carc.completion_rate * 100], ...
        'VariableNames', {'Planner', 'CollisionRate_pct', 'MinClearance_m', 'CompletionRate_pct'});
    
    filename = sprintf('sih_metrics_%s.csv', timestamp);
    writetable(metrics_table, filename);
    fprintf('Metrics exported to: %s\n', filename);
end
