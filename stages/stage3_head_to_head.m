function stage3_head_to_head(varargin)
    % STAGE3_HEAD_TO_HEAD - Benchmark Comparison: Stage 2 vs Stage 3
    %
    % Purpose:
    %   Executes Stage 2 (Candidate Baseline Planner) and Stage 3 (Linearized QP-MPC Planner)
    %   under identical conditions: initial state, scenario, speed target, duration,
    %   dt, reference path, road bounds, and static obstacle positions.
    
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir)
        projectRoot = pwd;
    else
        projectRoot = fileparts(current_dir);
    end
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'core'));
    addpath(fullfile(projectRoot, 'environment'));
    addpath(fullfile(projectRoot, 'vehicle'));
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'metrics'));
    addpath(fullfile(projectRoot, 'stages'));
    
    parser = inputParser;
    addParameter(parser, 'scenario', 'moderate', @ischar);
    addParameter(parser, 'duration', 10.0, @isnumeric);
    addParameter(parser, 'target_speed', 8.0, @isnumeric);
    
    parse(parser, varargin{:});
    
    scenario_name = parser.Results.scenario;
    T_sim = parser.Results.duration;
    v_target = parser.Results.target_speed;
    
    fprintf('\n');
    fprintf('════════════════════════════════════════════════════════════════════════════\n');
    fprintf('        HEAD-TO-HEAD BENCHMARK COMPARISON: STAGE 2 VS STAGE 3\n');
    fprintf('════════════════════════════════════════════════════════════════════════════\n');
    fprintf('  Scenario: %s | Target Speed: %.2f m/s | Duration: %.1f s\n', ...
        scenario_name, v_target, T_sim);
    fprintf('════════════════════════════════════════════════════════════════════════════\n\n');
    
    fprintf('[EXECUTION] Running Stage 2 (Candidate Baseline Planner)...\n');
    res2 = run_stage2_eval(scenario_name, T_sim, v_target);
    
    fprintf('[EXECUTION] Running Stage 3 (Linearized QP-MPC Baseline Planner)...\n');
    res3 = run_stage3_eval(scenario_name, T_sim, v_target);
    
    fprintf('\n');
    fprintf('┌─────────────────────────────────────────┬──────────────────┬──────────────────┐\n');
    fprintf('│ Benchmark Metric / Parameter            │ Stage 2 (Cand.)  │ Stage 3 (QP-MPC) │\n');
    fprintf('├─────────────────────────────────────────┼──────────────────┼──────────────────┤\n');
    fprintf('│ Simulation Steps Executed               │ %16s │ %16s │\n', res2.steps_str, res3.steps_str);
    fprintf('│ Task Success Status                     │ %16s │ %16s │\n', res2.task_success_str, res3.task_success_str);
    fprintf('│ Distance Traveled                       │ %14.2f m │ %14.2f m │\n', res2.dist, res3.dist);
    fprintf('│ Final Speed                             │ %12.2f m/s │ %12.2f m/s │\n', res2.v_final, res3.v_final);
    fprintf('│ Mean Cross-Track Error (e_y)            │ %14.3f m │ %14.3f m │\n', res2.mean_ey, res3.mean_ey);
    fprintf('│ Max Cross-Track Error (e_y)             │ %14.3f m │ %14.3f m │\n', res2.max_ey, res3.max_ey);
    fprintf('│ Mean Speed Error (e_v)                  │ %12.3f m/s │ %12.3f m/s │\n', res2.mean_ev, res3.mean_ev);
    fprintf('│ Min Obstacle Clearance                  │ %14.2f m │ %14.2f m │\n', res2.min_clr, res3.min_clr);
    fprintf('│ Collision-Active Steps                  │ %16d │ %16d │\n', res2.collision_steps, res3.collision_steps);
    fprintf('│ Road Bounds Compliance                  │ %16s │ %16s │\n', res2.bounds_str, res3.bounds_str);
    fprintf('│ Total Planner Runtime                   │ %13.2f ms │ %13.2f ms │\n', res2.avg_plan_ms, res3.avg_plan_ms);
    fprintf('│ Pure QP Solver Runtime                  │ %16s │ %13.2f ms │\n', 'N/A', res3.avg_qp_ms);
    fprintf('│ Solver Feasibility Rate                 │ %15.1f %% │ %15.1f %% │\n', res2.feasibility_rate, res3.feasibility_rate);
    fprintf('│ Fallback Control Usage Count            │ %16d │ %16d │\n', res2.fallback_count, res3.fallback_count);
    fprintf('└─────────────────────────────────────────┴──────────────────┴──────────────────┘\n\n');
end

function res = run_stage2_eval(scenario_name, T_sim, v_target)
    cfg = SimulationConfig();
    world = ScenarioDefinitions(scenario_name, cfg);
    planner = BaselinePlannerSimple(cfg);
    vehicle = BicycleModel(cfg);
    controller = VehicleController(cfg);
    
    world.ego.x = 10.0; world.ego.y = 3.0; world.ego.theta = 0.0; world.ego.v = 5.0;
    
    x_ref = (0:0.5:150)'; y_ref = 3.0 * ones(size(x_ref)); ptheta = zeros(size(x_ref));
    reference_path = [x_ref, y_ref, ptheta];
    
    dt = cfg.dt; N_steps = round(T_sim / dt);
    
    ey_list = zeros(N_steps, 1); ev_list = zeros(N_steps, 1);
    clr_list = zeros(N_steps, 1); coll_list = false(N_steps, 1); bounds_list = false(N_steps, 1);
    times = zeros(N_steps, 1);
    
    for k = 1:N_steps
        t_start = tic;
        [planned_path, ~, ~, ~] = planner.plan(world, reference_path, cfg);
        times(k) = toc(t_start) * 1000;
        
        a_cmd = controller.computeSpeedControl(world.ego.v, v_target, cfg.dt);
        [delta_cmd, ~, ~] = controller.computeStanleyControl(world.ego, planned_path);
        
        world.ego = vehicle.stepKinematic(world.ego, a_cmd, delta_cmd, dt);
        
        clr_list(k) = world.getMinClearance(cfg);
        coll_list(k) = clr_list(k) <= 0;
        bounds_list(k) = world.isEgoInBounds(cfg);
        ey_list(k) = world.ego.y - 3.0;
        ev_list(k) = world.ego.v - v_target;
    end
    
    res = struct();
    res.steps_str = sprintf('%d/%d steps', N_steps, N_steps);
    res.dist = world.ego.x - 10.0;
    res.v_final = world.ego.v;
    res.mean_ey = mean(abs(ey_list));
    res.max_ey = max(abs(ey_list));
    res.mean_ev = mean(abs(ev_list(20:end)));
    res.min_clr = min(clr_list);
    res.collision_steps = sum(coll_list);
    res.bounds_str = sprintf('%d/%d', sum(bounds_list), N_steps);
    res.avg_plan_ms = mean(times);
    res.avg_qp_ms = 0;
    res.feasibility_rate = 100.0;
    res.fallback_count = 0;
    
    task_ok = (res.collision_steps == 0) && (sum(bounds_list) == N_steps) && (res.mean_ey < 0.5);
    if task_ok, res.task_success_str = 'PASS'; else, res.task_success_str = 'FAIL'; end
end

function res = run_stage3_eval(scenario_name, T_sim, v_target)
    cfg = SimulationConfig();
    world = ScenarioDefinitions(scenario_name, cfg);
    planner = QPMPCPlanner(cfg);
    planner.reset();
    vehicle = BicycleModel(cfg);
    
    world.ego.x = 10.0; world.ego.y = 3.0; world.ego.theta = 0.0; world.ego.v = 5.0;
    
    x_ref = (0:0.5:150)'; y_ref = 3.0 * ones(size(x_ref)); theta_ref = zeros(size(x_ref));
    reference_path = [x_ref, y_ref, theta_ref];
    
    dt = cfg.dt; N_steps = round(T_sim / dt);
    
    ey_list = zeros(N_steps, 1); ev_list = zeros(N_steps, 1);
    clr_list = zeros(N_steps, 1); coll_list = false(N_steps, 1); bounds_list = false(N_steps, 1);
    qp_times = zeros(N_steps, 1); total_times = zeros(N_steps, 1); status_list = zeros(N_steps, 1);
    
    for k = 1:N_steps
        [u_opt, ~, status, info] = planner.plan(world, reference_path, v_target);
        qp_times(k) = info.solve_time_ms;
        total_times(k) = info.total_time_ms;
        status_list(k) = status;
        
        world.ego = vehicle.stepKinematic(world.ego, u_opt(2), u_opt(1), dt);
        
        clr_list(k) = world.getMinClearance(cfg);
        coll_list(k) = clr_list(k) <= 0;
        bounds_list(k) = world.isEgoInBounds(cfg);
        ey_list(k) = world.ego.y - 3.0;
        ev_list(k) = world.ego.v - v_target;
    end
    
    res = struct();
    res.steps_str = sprintf('%d/%d steps', N_steps, N_steps);
    res.dist = world.ego.x - 10.0;
    res.v_final = world.ego.v;
    res.mean_ey = mean(abs(ey_list));
    res.max_ey = max(abs(ey_list));
    res.mean_ev = mean(abs(ev_list(20:end)));
    res.min_clr = min(clr_list);
    res.collision_steps = sum(coll_list);
    res.bounds_str = sprintf('%d/%d', sum(bounds_list), N_steps);
    res.avg_plan_ms = mean(total_times);
    res.avg_qp_ms = mean(qp_times);
    res.feasibility_rate = mean(status_list) * 100;
    res.fallback_count = sum(status_list == 0);
    
    task_ok = (res.collision_steps == 0) && (sum(bounds_list) == N_steps) && (res.mean_ey < 0.5) && (res.feasibility_rate > 90);
    if task_ok, res.task_success_str = 'PASS'; else, res.task_success_str = 'FAIL'; end
end
