function run_cattle_coordination_experiment()
    % RUN_CATTLE_COORDINATION_EXPERIMENT Experiment B: Dynamic Irregular Agent (Cattle) Coordination Test
    %
    % Purpose:
    %   Evaluates Stage 5 Coordination Controller on dynamic cattle crossing scenario.
    %   Demonstrates that the system detects dynamic irregular road users (cattle)
    %   at long range and executes safe YIELD maneuvers without collision.

    addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'tests');
    
    fprintf('\n========================================================================================\n');
    fprintf('   EXPERIMENT B: DYNAMIC IRREGULAR AGENT (CATTLE) COORDINATION BENCHMARK              \n');
    fprintf('   Scenario: scenario_indian_unstructured (with Dynamic Cattle Agent at x=65m)         \n');
    fprintf('========================================================================================\n\n');
    
    cfg = SimulationConfig();
    map_obj = FreeSpaceMap.createIndianUnstructuredMap();
    
    ctrl = Stage5CoordinationController(cfg);
    ctrl.reset();
    ctrl.cacrc_planner.bound_provider = FreeSpaceBoundProvider(map_obj);
    ctrl.setOvertakeEnabled(true);
    
    gt_world = ScenarioDefinitions('indian_unstructured', cfg);
    vehicle = BicycleModel(cfg);
    obs_model = ObservationModel('ideal', 42);
    
    N_steps = 350; dt = cfg.dt;
    
    N_path = 700;
    ref_path = zeros(N_path, 5);
    ref_path(:, 1) = linspace(0, 150, N_path)';
    ref_path(:, 2) = 2.75;
    ref_path(:, 4) = 0.0; ref_path(:, 5) = 5.0;
    
    trace.ego_x = zeros(N_steps, 1);
    trace.ego_y = zeros(N_steps, 1);
    trace.ego_v = zeros(N_steps, 1);
    trace.intent = cell(N_steps, 1);
    
    clr_list = zeros(N_steps, 1);
    coll_list = false(N_steps, 1);
    yield_triggered = false;
    yield_step = -1;
    
    for k = 1:N_steps
        [obs_world, ~] = obs_model.observe(gt_world, dt);
        [u_cmd, pred_states, status, info] = ctrl.step(obs_world, ref_path, 5.0);
        
        gt_world.ego = vehicle.stepKinematic(gt_world.ego, u_cmd(2), u_cmd(1), dt);
        
        trace.ego_x(k) = gt_world.ego.x;
        trace.ego_y(k) = gt_world.ego.y;
        trace.ego_v(k) = gt_world.ego.v;
        trace.intent{k} = info.macro_intent;
        
        clr_list(k) = gt_world.getMinClearance(cfg);
        coll_list(k) = clr_list(k) <= 0;
        
        if strcmp(info.macro_intent, 'YIELD') && ~yield_triggered
            yield_triggered = true;
            yield_step = k;
        end
        
        gt_world = gt_world.stepAgents(dt);
    end
    
    max_x = max(trace.ego_x);
    min_clr = min(clr_list);
    coll_steps = sum(coll_list);
    
    fprintf('========================================================================================\n');
    fprintf('           EXPERIMENT B: DYNAMIC CATTLE COORDINATION BENCHMARK RESULTS                 \n');
    fprintf('========================================================================================\n');
    fprintf('  %-38s | %-18s\n', 'Metric', 'Evaluation Result');
    fprintf('  ----------------------------------------+--------------------\n');
    fprintf('  %-38s | %-18d\n', 'Total Steps Evaluated', N_steps);
    fprintf('  %-38s | %-18.2f m\n', 'Maximum x Position Reached', max_x);
    fprintf('  %-38s | %-18s\n', 'Yield Intent Triggered', char(string(yield_triggered)));
    if yield_triggered
        fprintf('  %-38s | k = %d (t = %.2f s)\n', 'First Yield Timestep', yield_step, (yield_step-1)*dt);
    end
    fprintf('  %-38s | %-18d\n', 'Collision Active Steps (OBB SAT)', coll_steps);
    fprintf('  %-38s | %-+18.4f m\n', 'Minimum Agent Clearance', min_clr);
    fprintf('========================================================================================\n\n');
    
    if coll_steps == 0 && yield_triggered
        fprintf('  => VERDICT: DYNAMIC AGENT COORDINATION VALIDATED!\n');
        fprintf('     Ego vehicle safely detects irregular cattle agent and yields without collision.\n');
    else
        fprintf('  => VERDICT: DYNAMIC AGENT COORDINATION FAILED!\n');
    end
    fprintf('========================================================================================\n\n');
end
