function run_end_to_end_presentation()
    % RUN_END_TO_END_PRESENTATION Primary Presentation Master Scenarios (Demo A & Demo B)
    %
    % Purpose:
    %   Generates the two primary presentation demonstrations for Phase 12:
    %     1. Demo A: Complete End-to-End Success Flow (Nominal -> Narrowing -> Follow -> Overtake -> Recenter)
    %     2. Demo B: Controlled Emergency Safety Flow (Blockage -> Infeasible Topology -> Controlled Emergency Stop)
    
    addpath('planning', 'config', 'vehicle', 'core', 'environment', 'stages', 'tests', 'visualization');
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 12: PRIMARY PRESENTATION MASTER END-TO-END DEMONSTRATIONS                  \n');
    fprintf('========================================================================================\n\n');
    
    cfg = SimulationConfig();
    
    % -------------------------------------------------------------------------
    % DEMO A: COMPLETE END-TO-END SUCCESS FLOW
    % -------------------------------------------------------------------------
    fprintf('>>> Generating Demo A: Complete End-to-End Success Flow <<<\n');
    worldA = ScenarioDefinitions('cattle_crossing', cfg);
    mapA = FreeSpaceMap.createCattleScenarioMap();
    loggerA = ScenarioEventLogger();
    ctrlA = Stage5CoordinationController(cfg);
    ctrlA.reset();
    ctrlA.cacrc_planner.bound_provider = FreeSpaceBoundProvider(mapA);
    ctrlA.setOvertakeEnabled(true);
    vehicle = BicycleModel(cfg);
    
    N_path = 500;
    ref_pathA = zeros(N_path, 5);
    ref_pathA(:, 1) = linspace(0, 150, N_path)';
    for r = 1:N_path
        [y_min_r, y_max_r] = mapA.getRoadBoundsAt(ref_pathA(r, 1));
        ref_pathA(r, 2) = 0.5 * (y_min_r + y_max_r);
    end
    ref_pathA(:, 3) = 0.0; ref_pathA(:, 4) = 0.0; ref_pathA(:, 5) = 8.0;
    
    N_stepsA = 120;
    dt = cfg.dt;
    visA = SceneVisualizer('Visible', 'off');
    
    for k = 1:N_stepsA
        [u_cmd, pred_states, status, info] = ctrlA.step(worldA, ref_pathA, 8.0);
        info.event_banner = 'NOMINAL DRIVING';
        if worldA.ego.x > 35.0 && worldA.ego.x < 65.0
            info.event_banner = 'OVERTAKING SLOW VEHICLE & OBSTACLE';
        elseif worldA.ego.x >= 65.0
            info.event_banner = 'RECENTERING TO NOMINAL CENTERLINE';
        end
        
        loggerA.update((k-1)*dt, worldA, info);
        worldA.ego = vehicle.stepKinematic(worldA.ego, u_cmd(2), u_cmd(1), dt);
        
        if worldA.n_agents > 0
            worldA.agents(1) = worldA.agents(1).constantVelocityUpdate(dt);
        end
        
        visA.render(worldA, mapA, pred_states, info, ref_pathA, k);
    end
    visA.saveFrame('artifacts/demoA_presentation_success.png');
    loggerA.printTimeline();
    
    % -------------------------------------------------------------------------
    % DEMO B: CONTROLLED EMERGENCY SAFETY FLOW
    % -------------------------------------------------------------------------
    fprintf('>>> Generating Demo B: Controlled Emergency Safety Flow <<<\n');
    worldB = ScenarioDefinitions('impassable_center', cfg);
    mapB = FreeSpaceMap.createCorridorMap(0.0, 6.0);
    loggerB = ScenarioEventLogger();
    ctrlB = Stage5CoordinationController(cfg);
    ctrlB.reset();
    ctrlB.cacrc_planner.bound_provider = FreeSpaceBoundProvider(mapB);
    ctrlB.setOvertakeEnabled(true);
    
    ref_pathB = ref_pathA;
    N_stepsB = 80;
    visB = SceneVisualizer('Visible', 'off');
    
    for k = 1:N_stepsB
        [u_cmd, pred_states, status, info] = ctrlB.step(worldB, ref_pathB, 8.0);
        if status == 0
            info.event_banner = 'EMERGENCY STOP (No Safe Free-Space Topology)';
        else
            info.event_banner = 'APPROACHING TOTAL ROAD BLOCKAGE';
        end
        
        loggerB.update((k-1)*dt, worldB, info);
        worldB.ego = vehicle.stepKinematic(worldB.ego, u_cmd(2), u_cmd(1), dt);
        visB.render(worldB, mapB, pred_states, info, ref_pathB, k);
    end
    visB.saveFrame('artifacts/demoB_presentation_safetystop.png');
    loggerB.printTimeline();
    
    fprintf('========================================================================================\n');
    fprintf('  MASTER PRESENTATION DEMONSTRATIONS GENERATED IN artifacts/ DIRECTORY!\n');
    fprintf('========================================================================================\n\n');
end
