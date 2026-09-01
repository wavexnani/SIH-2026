function run_phase14_before_after_eval()
    % RUN_PHASE14_BEFORE_AFTER_EVAL Runs single-pass evaluation across 9 scenarios
    % comparing legacy metrics vs corrected Phase 14 metrics.
    
    addpath('planning', 'config', 'vehicle', 'core', 'environment', 'stages', 'metrics', 'tests');
    
    levels = [1, 2, 4, 5, 6, 7, 8, 9, 10];
    obs_modes = {'ideal', 'ideal', 'ideal', 'ideal', 'ideal', 'ideal', 'ideal', 'combined_realistic', 'ideal'};
    act_modes = {'ideal', 'ideal', 'ideal', 'ideal', 'ideal', 'ideal', 'ideal', 'combined', 'ideal'};
    
    fprintf('\n=========================================================================================================\n');
    fprintf('  PHASE 14 FORENSIC EVALUATION: BEFORE VS AFTER METRICS COMPARISON                                      \n');
    fprintf('=========================================================================================================\n\n');
    
    cfg = SimulationConfig();
    cfg.plot_enabled = false;
    cfg.verbose = false;
    
    results = struct();
    
    for idx = 1:length(levels)
        lvl = levels(idx);
        obs_mode = obs_modes{idx};
        act_mode = act_modes{idx};
        
        [scen_key, scen_name] = ScenarioLadder.getLevel(lvl);
        
        % Run simulation with live corrected pipeline
        [passed, metrics, history] = stage5_multivehicle_coordination('scenario', scen_key, 'uncertainty_mode', obs_mode, 'seed', 42, 'verbose', false);
        
        % Compute OLD radial clearance across history
        N = length(history.t);
        old_clr_hist = zeros(N, 1);
        r_ego = cfg.vehicle_width / 2; % 0.9m
        
        % Reconstruct world state at each step to get old radial clearance
        % or compute from stored agent/obs positions in history
        for k = 1:N
            ego_x = history.ego_x(k); ego_y = history.ego_y(k);
            min_r_clr = inf;
            % Re-read clearance if available or compute via SAT vs Radial
            % For old metric:
            % Static obs check
            % Dynamic agent check
            % In our stage history, min_clearance is NOW corrected OBB clearance.
            % We compute old radial clearance directly:
        end
        
        % Re-run metric extraction with old vs new formulas
        % Old lateral RMSE: relative to fixed 1.80m
        old_lat_rmse = sqrt(mean((history.ego_y - 1.80).^2));
        new_lat_rmse = metrics.lat_rmse; % relative to active reference path
        
        % Compute Old Radial Clearance minimum
        % Re-instantiate scenario world state step-by-step
        old_min_clr = compute_old_radial_min_clr(scen_key, obs_mode, act_mode, 42, cfg);
        new_min_clr = metrics.min_clr;
        
        % Determine Old Outcome using old metrics
        old_collision_steps = sum(history.is_collision_old);
        old_bounds_steps = metrics.bounds_steps;
        old_qp_infeas = sum(history.solver_status == 0);
        
        old_outcome = classify_old_outcome(old_collision_steps, old_bounds_steps, N, old_qp_infeas, metrics.final_v, old_min_clr, old_lat_rmse, metrics.emergency_braking_count, metrics.safety_rejected_count);
        new_outcome = metrics.outcome;
        
        % Store results
        results(idx).level = lvl;
        results(idx).name = scen_name;
        results(idx).old_outcome = old_outcome;
        results(idx).new_outcome = new_outcome;
        results(idx).old_min_clr = old_min_clr;
        results(idx).new_min_clr = new_min_clr;
        results(idx).old_lat_rmse = old_lat_rmse;
        results(idx).new_lat_rmse = new_lat_rmse;
        valid_ttc = history.min_ttc(~isinf(history.min_ttc));
        if ~isempty(valid_ttc), min_ttc_val = min(valid_ttc); else min_ttc_val = Inf; end
        results(idx).min_ttc = min_ttc_val;
        results(idx).collision_steps = metrics.collision_steps;
        results(idx).boundary_violations = N - metrics.bounds_steps;
        results(idx).emergency_interventions = metrics.emergency_braking_count;
        results(idx).qp_infeasibility = old_qp_infeas;
    end
    
    % Print detailed summary table
    fprintf('%-6s | %-16s | %-16s | %-7s | %-7s | %-8s | %-8s | %-7s | %-5s | %-5s | %-5s | %-5s\n', ...
        'Level', 'Old Outcome', 'Corrected Outcome', 'Old Clr', 'New Clr', 'Old RMSE', 'New RMSE', 'Min TTC', 'Coll', 'Bnd', 'Emg', 'QPInf');
    fprintf('%s\n', repmat('-', 1, 125));
    
    for idx = 1:length(levels)
        r = results(idx);
        fprintf('L%-5d | %-16s | %-16s | %7.2f | %7.2f | %8.3f | %8.3f | %7.2f | %5d | %5d | %5d | %5d\n', ...
            r.level, r.old_outcome, r.new_outcome, r.old_min_clr, r.new_min_clr, ...
            r.old_lat_rmse, r.new_lat_rmse, r.min_ttc, r.collision_steps, ...
            r.boundary_violations, r.emergency_interventions, r.qp_infeasibility);
    end
    fprintf('\n');
end

function old_min_clr = compute_old_radial_min_clr(scen_key, obs_mode, act_mode, seed, cfg)
    % Helper to reconstruct old radial clearance for historical comparison
    r_ego = cfg.vehicle_width / 2; % 0.9m
    
    % Run simulation and compute radial clearance at each step
    obs_model = ObservationModel(obs_mode, seed);
    act_model = ActuatorUncertaintyModel(act_mode);
    ctrl5 = Stage5CoordinationController(cfg);
    vehicle_plant = BicycleModel(cfg);
    
    world = ScenarioDefinitions(scen_key, cfg);
    v_des = 8.0;
    ref_path = zeros(500, 5);
    ref_path(:, 1) = linspace(0, 150, 500)';
    ref_path(:, 2) = world.ego.y;
    ref_path(:, 5) = v_des;
    dt = cfg.dt; N_steps = round(cfg.T_sim_max / dt);
    
    old_min_clr = inf;
    
    for k = 1:N_steps
        % Old radial clearance formula
        r_clr = inf;
        for i = 1:world.n_static_obs
            obs = world.static_obs(i, :);
            if obs(1) > -50 && obs(3) > 0
                d_c = norm([world.ego.x - obs(1), world.ego.y - obs(2)]);
                r_c = d_c - r_ego - (obs(4)/2);
                if r_c < r_clr, r_clr = r_c; end
            end
        end
        for i = 1:world.n_agents
            ag = world.agents(i);
            if ag.x > -50
                d_c = norm([world.ego.x - ag.x, world.ego.y - ag.y]);
                r_c = d_c - r_ego - (ag.width/2 + 0.1);
                if r_c < r_clr, r_clr = r_c; end
            end
        end
        if r_clr < old_min_clr, old_min_clr = r_clr; end
        
        [obs_world, ~] = obs_model.observe(world, dt);
        [u_cmd, ~, ~, ~] = ctrl5.step(obs_world, ref_path, v_des);
        [u_act, ~] = act_model.process(u_cmd, cfg);
        world.ego = vehicle_plant.stepKinematic(world.ego, u_act(2), u_act(1), dt);
        world = world.stepAgents(dt);
        if world.ego.x >= cfg.road_length - 5.0, break; end
    end
end

function old_outcome = classify_old_outcome(coll_steps, bnd_steps, N_steps, qp_inf, final_v, min_clr, lat_rmse, emg, rej)
    if coll_steps > 0
        old_outcome = 'COLLISION';
    elseif bnd_steps < N_steps
        old_outcome = 'UNSAFE_FAILURE';
    elseif qp_inf > 5 && final_v >= 0.10
        old_outcome = 'PLANNER_INFEASIBLE';
    elseif final_v < 0.10 && min_clr > 0.0
        old_outcome = 'SAFE_STOP';
    elseif lat_rmse > 0.50 || emg > 0 || rej > 0
        old_outcome = 'DEGRADED_SAFE';
    else
        old_outcome = 'SUCCESS';
    end
end
