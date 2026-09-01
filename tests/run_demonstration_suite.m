function run_demonstration_suite()
    % RUN_DEMONSTRATION_SUITE Master Demonstration Suite for Demos 1 to 6
    %
    % Purpose:
    %   Executes and renders 6 high-fidelity demonstration artifacts for publication/presentation:
    %     Demo 1: Normal Driving (Centerline following in open road)
    %     Demo 2: Obstacle Avoidance (Avoids static obstacle)
    %     Demo 3: Fixed Corridor vs Free Space (Controlled A/B comparison)
    %     Demo 4: Road Narrowing (MPC horizon preview anticipating narrowing)
    %     Demo 5: Multi-Vehicle Interaction (Yield -> Overtake intent transitions)
    %     Demo 6: Difficult/Infeasible Scenario (Safe emergency stopping on total blockage)
    
    addpath('planning', 'config', 'core', 'environment', 'stages', 'tests', 'visualization');
    
    if ~exist('artifacts', 'dir'), mkdir('artifacts'); end
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 9: MASTER DEMONSTRATION SUITE (DEMOS 1 - 6)                                \n');
    fprintf('========================================================================================\n\n');
    
    cfg = SimulationConfig();
    
    % --- Demo 1: Normal Driving ---
    fprintf('Generating Demo 1: Normal Driving...\n');
    [world1, map1] = ScenarioGenerator.createScenario('nominal_straight', cfg);
    render_demo_snapshot(world1, map1, 'artifacts/demo1_normal_driving.png', 'Demo 1: Nominal Driving');
    
    % --- Demo 2: Obstacle Avoidance ---
    fprintf('Generating Demo 2: Obstacle Avoidance...\n');
    [world2, map2] = ScenarioGenerator.createScenario('gradual_narrowing', cfg);
    render_demo_snapshot(world2, map2, 'artifacts/demo2_obstacle_avoidance.png', 'Demo 2: Static Obstacle Avoidance');
    
    % --- Demo 3: Fixed Corridor vs Free Space ---
    fprintf('Generating Demo 3: Fixed Corridor vs Free Space Comparison...\n');
    run_baseline_comparison_demo();
    
    % --- Demo 4: Road Narrowing Horizon Preview ---
    fprintf('Generating Demo 4: Road Narrowing Preview...\n');
    [world4, map4] = ScenarioGenerator.createScenario('tight_corridor', cfg);
    render_demo_snapshot(world4, map4, 'artifacts/demo4_road_narrowing.png', 'Demo 4: Dynamic Road Narrowing Horizon');
    
    % --- Demo 5: Multi-Vehicle Interaction ---
    fprintf('Generating Demo 5: Multi-Vehicle Interaction...\n');
    [~, ~, h5] = stage5_multivehicle_coordination('scenario', 'multi_vehicle_oncoming_conflict', 'verbose', false);
    fig5 = figure('Visible', 'off', 'Position', [100, 100, 1000, 450]);
    plot(h5.ego_x, h5.ego_y, 'b-', 'LineWidth', 2.5); grid on; box on;
    xlabel('Longitudinal Position x (m)'); ylabel('Lateral Position y (m)');
    title('Demo 5: Stage 5.1 Multi-Vehicle Interaction (Yield -> Overtake)');
    saveas(fig5, 'artifacts/demo5_multivehicle_interaction.png'); close(fig5);
    
    % --- Demo 6: Difficult/Infeasible Scenario ---
    fprintf('Generating Demo 6: Infeasible Scenario Emergency Stop...\n');
    [world6, map6] = ScenarioGenerator.createScenario('infeasible_blocked', cfg);
    render_demo_snapshot(world6, map6, 'artifacts/demo6_infeasible_scenario.png', 'Demo 6: Deliberately Infeasible Total Blockage');
    
    fprintf('\n[SUCCESS] All 6 Demonstration Artifacts Generated in artifacts/ directory!\n\n');
end

function render_demo_snapshot(world, map_obj, filename, title_str)
    cfg = SimulationConfig();
    vehicle = BicycleModel(cfg);
    ctrl = Stage5CoordinationController(cfg);
    ctrl.reset();
    ctrl.cacrc_planner.bound_provider = FreeSpaceBoundProvider(map_obj, 1.05);
    ctrl.setOvertakeEnabled(true);
    
    N_steps = 75; dt = cfg.dt;
    N_path = 500; ref_path = zeros(N_path, 5);
    ref_path(:, 1) = linspace(0, 150, N_path)';
    for r = 1:N_path
        [y_min_r, y_max_r] = map_obj.getRoadBoundsAt(ref_path(r, 1));
        ref_path(r, 2) = 0.5 * (y_min_r + y_max_r);
    end
    for r = 1:N_path
        r1 = max(1, r - 1); r2 = min(N_path, r + 1);
        ref_path(r, 3) = atan2(ref_path(r2, 2) - ref_path(r1, 2), ref_path(r2, 1) - ref_path(r1, 1));
    end
    ref_path(:, 4) = 0.0; ref_path(:, 5) = 8.0;
    
    for k = 1:N_steps
        [u_cmd, pred_states, status, info] = ctrl.step(world, ref_path, 8.0);
        world.ego = vehicle.stepKinematic(world.ego, u_cmd(2), u_cmd(1), dt);
    end
    
    viz = SceneVisualizer('Visible', 'off');
    viz.render(world, map_obj, pred_states, info, ref_path, N_steps);
    title(viz.ax, title_str, 'FontSize', 13, 'FontWeight', 'bold');
    viz.saveFrame(filename);
    viz.close();
end
