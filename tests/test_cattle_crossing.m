function test_cattle_crossing()
    % TEST_CATTLE_CROSSING Execution of Test C (Cattle Crossing / Irregular Obstacle Scenario)
    %
    % Purpose:
    %   Evaluates Stage 5.1 with FreeSpaceBoundProvider when encountering a 
    %   slow-moving crossing livestock agent and irregular boundaries.
    %
    % Verifies:
    %   - 100% drivable space compliance
    %   - 0 collision steps
    %   - Positive minimum clearance
    %   - 0 emergency braking steps
    
    addpath('planning', 'config', 'core', 'environment', 'stages');
    
    fprintf('\n========================================================================================\n');
    fprintf('        TEST C: CATTLE CROSSING SCENARIO WITH FREESPACE BOUND PROVIDER                  \n');
    fprintf('========================================================================================\n\n');
    
    cfg = SimulationConfig();
    world = ScenarioDefinitions('cattle_crossing', cfg);
    vehicle = BicycleModel(cfg);
    
    % Initialize Stage 5 Controller with FreeSpaceBoundProvider
    ctrl = Stage5CoordinationController(cfg);
    ctrl.reset();
    unstructured_map = FreeSpaceMap.createUnstructuredMap();
    ctrl.cacrc_planner.bound_provider = FreeSpaceBoundProvider(unstructured_map);
    ctrl.setOvertakeEnabled(true);
    
    N_steps = 150;
    dt = cfg.dt;
    
    % Construct Reference Path following center of irregular road
    N_path = 500;
    ref_path = zeros(N_path, 5);
    ref_path(:, 1) = linspace(0, 150, N_path)';
    for r = 1:N_path
        [y_min_r, y_max_r] = unstructured_map.getRoadBoundsAt(ref_path(r, 1));
        ref_path(r, 2) = 0.5 * (y_min_r + y_max_r); % Road center
    end
    ref_path(:, 4) = 0.0;
    ref_path(:, 5) = 8.0;
    
    drivable_compliance = true(N_steps, 1);
    clr_list = zeros(N_steps, 1);
    coll_list = false(N_steps, 1);
    emergency_count = 0;
    
    for k = 1:N_steps
        [u_cmd, ~, status, info] = ctrl.step(world, ref_path, 8.0);
        if status == 0
            emergency_count = emergency_count + 1;
        end
        
        % Update Ego & Dynamic Cattle Agent
        world.ego = vehicle.stepKinematic(world.ego, u_cmd(2), u_cmd(1), dt);
        
        % Cattle agent moves slowly laterally across the road
        if world.n_agents >= 1 && world.agents(1).x > 0
            world.agents(1).y = world.agents(1).y + 0.03; % Slow crossing motion
        end
        
        % Verify 2D drivable space compliance
        drivable_compliance(k) = unstructured_map.isDrivable(world.ego.x, world.ego.y);
        clr_list(k) = world.getMinClearance(cfg);
        coll_list(k) = clr_list(k) <= 0;
    end
    
    bounds_passed = sum(drivable_compliance);
    coll_steps = sum(coll_list);
    min_clr = min(clr_list);
    
    fprintf('  [TEST C PERFORMANCE METRICS]\n');
    fprintf('    Drivable-Space Compliance (Steps) : %d / %d (%.1f%%)\n', ...
        bounds_passed, N_steps, (bounds_passed / N_steps) * 100);
    fprintf('    Collision Active Steps            : %d\n', coll_steps);
    fprintf('    Minimum Obstacle Clearance (m)     : +%.4f m\n', min_clr);
    fprintf('    Emergency Braking / Fallbacks     : %d steps\n', emergency_count);
    
    test_pass = (bounds_passed == N_steps) && (coll_steps == 0) && (min_clr > 0.0) && (emergency_count == 0);
    
    fprintf('========================================================================================\n');
    if test_pass
        fprintf('  OVERALL TEST C: PASS (Stage 5.1 handles dynamic cattle crossing cleanly!)\n');
    else
        fprintf('  OVERALL TEST C: FAIL (Cattle crossing safety constraints violated)\n');
        error('Test C failed: Cattle crossing safety requirements not met');
    end
    fprintf('========================================================================================\n\n');
end
