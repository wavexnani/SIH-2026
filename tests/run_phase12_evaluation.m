function run_phase12_evaluation()
    % RUN_PHASE12_EVALUATION Master Closed-Loop Robustness & Scenario Ladder Evaluation
    %
    % Purpose:
    %   Evaluates closed-loop performance across the 10-level scenario progression ladder
    %   and switchable uncertainty modes, producing quantitative success, safe failure,
    %   unsafe failure, and motion comfort metrics.
    
    addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'tests', 'visualization');
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 12: MASTER CLOSED-LOOP REALISM & SCENARIO LADDER EVALUATION              \n');
    fprintf('========================================================================================\n\n');
    
    cfg = SimulationConfig();
    modes = {'ideal', 'combined_realistic'};
    n_levels = 10;
    
    results = struct();
    
    for m = 1:length(modes)
        mode_str = modes{m};
        fprintf('>>> Evaluating Uncertainty Mode: %s <<<\n', upper(mode_str));
        
        for lvl = 1:n_levels
            [world_base, map_obj, level_name, lvl_mode] = ScenarioLadder.getLevel(lvl, cfg);
            
            % Override uncertainty mode if level specifies or default mode
            active_mode = mode_str;
            if strcmp(lvl_mode, 'combined_realistic'), active_mode = 'combined_realistic'; end
            
            unc_model = UncertaintyModel(active_mode, 100 + lvl);
            logger = ScenarioEventLogger();
            
            vehicle = BicycleModel(cfg);
            ctrl = Stage5CoordinationController(cfg);
            ctrl.reset();
            ctrl.cacrc_planner.bound_provider = FreeSpaceBoundProvider(map_obj);
            if lvl >= 4, ctrl.setOvertakeEnabled(true); end
            
            % Construct Reference Path
            N_path = 500;
            ref_path = zeros(N_path, 5);
            ref_path(:, 1) = linspace(0, 150, N_path)';
            for r = 1:N_path
                [y_min_r, y_max_r] = map_obj.getRoadBoundsAt(ref_path(r, 1));
                ref_path(r, 2) = 0.5 * (y_min_r + y_max_r);
            end
            ref_path(:, 3) = 0.0; ref_path(:, 4) = 0.0; ref_path(:, 5) = 8.0;
            
            N_steps = 120;
            dt = cfg.dt;
            world = world_base;
            
            % Tracking lists
            drivable_list = true(N_steps, 1);
            coll_list = false(N_steps, 1);
            clr_list = zeros(N_steps, 1);
            status_list = zeros(N_steps, 1);
            ay_list = zeros(N_steps, 1);
            
            for k = 1:N_steps
                % 1. Feedback Observation with Uncertainty
                world_obs = unc_model.observeWorld(world);
                
                % 2. Controller Step
                [u_cmd, pred_states, status, info] = ctrl.step(world_obs, ref_path, 8.0);
                u_cmd = unc_model.perturbControl(u_cmd);
                
                status_list(k) = status;
                logger.update((k-1)*dt, world, info);
                
                % 3. Vehicle Kinematic Integration
                world.ego = vehicle.stepKinematic(world.ego, u_cmd(2), u_cmd(1), dt);
                
                % 4. Dynamic Agent Propagation (if present)
                if world.n_agents > 0
                    for a_idx = 1:world.n_agents
                        world.agents(a_idx) = world.agents(a_idx).constantVelocityUpdate(dt);
                    end
                end
                
                % 5. Metrics Collection
                half_W = cfg.vehicle_width / 2 + 0.15;
                bc_x = world.ego.x + 1.35 * cos(world.ego.theta);
                bc_y = world.ego.y + 1.35 * sin(world.ego.theta);
                drivable_list(k) = map_obj.isDrivable(bc_x, bc_y);
                clr_list(k) = world.getMinClearance(cfg);
                coll_list(k) = clr_list(k) <= 0;
                ay_list(k) = abs(world.ego.v * ((world.ego.v / 2.7) * tan(u_cmd(1))));
            end
            
            % Classify Outcome
            n_coll = sum(coll_list);
            n_bound_viol = sum(~drivable_list);
            n_infeas = sum(status_list == 0);
            min_clr = min(clr_list);
            max_ay_g = max(ay_list) / 9.81;
            
            if n_coll == 0 && n_bound_viol == 0 && n_infeas == 0
                outcome = 'SUCCESS';
            elseif n_coll == 0 && n_bound_viol == 0 && n_infeas > 0
                outcome = 'SAFE_STOP'; % Controlled emergency stop
            else
                outcome = 'UNSAFE_FAILURE';
            end
            
            fprintf('  Level %2d: %-45s | Outcome: %-14s | MinClr: %+5.2fm | Max Ay: %4.2fg | Infeas: %2d\n', ...
                lvl, level_name, outcome, min_clr, max_ay_g, n_infeas);
        end
        fprintf('\n');
    end
    fprintf('========================================================================================\n');
    fprintf('  PHASE 12 CLOSED-LOOP BENCHMARK EVALUATION COMPLETE!\n');
    fprintf('========================================================================================\n\n');
end
