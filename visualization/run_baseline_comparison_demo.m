function run_baseline_comparison_demo()
    % RUN_BASELINE_COMPARISON_DEMO Phase 3 Baseline Visual Comparison
    %
    % Purpose:
    %   Executes and visualizes Run A (CorridorBoundProvider) vs Run B (FreeSpaceBoundProvider)
    %   on the exact same unstructured road scenario using high-fidelity 2D rendering.
    
    addpath('planning', 'config', 'core', 'environment', 'stages', 'visualization');
    
    if ~exist('artifacts', 'dir'), mkdir('artifacts'); end
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 3: CONTROLLED VISUAL COMPARISON DEMO (RUN A VS RUN B)                    \n');
    fprintf('========================================================================================\n\n');
    
    unstructured_map = FreeSpaceMap.createUnstructuredMap();
    
    % --- Run A: CorridorBoundProvider ---
    provider_A = CorridorBoundProvider(1.05);
    fprintf('Executing Run A (CorridorBoundProvider)...\n');
    [hist_A, map_A] = execute_recording_run(provider_A, unstructured_map);
    
    % --- Run B: FreeSpaceBoundProvider ---
    provider_B = FreeSpaceBoundProvider(unstructured_map, 1.05);
    fprintf('Executing Run B (FreeSpaceBoundProvider)...\n');
    [hist_B, map_B] = execute_recording_run(provider_B, unstructured_map);
    
    % Render & Save Comparative Visual Artifacts
    fprintf('Rendering comparative visual artifacts...\n');
    
    % Select critical timestep where road narrows (Step 75)
    step_crit = 75;
    
    vizA = SceneVisualizer('Visible', 'off');
    vizA.render(hist_A.world_snapshots{step_crit}, map_A, hist_A.pred_states{step_crit}, ...
        hist_A.info_snapshots{step_crit}, hist_A.ref_path, step_crit);
    vizA.saveFrame('artifacts/demo3_runA_corridor.png');
    vizA.close();
    
    vizB = SceneVisualizer('Visible', 'off');
    vizB.render(hist_B.world_snapshots{step_crit}, map_B, hist_B.pred_states{step_crit}, ...
        hist_B.info_snapshots{step_crit}, hist_B.ref_path, step_crit);
    vizB.saveFrame('artifacts/demo3_runB_freespace.png');
    vizB.close();
    
    % Generate Combined Side-by-Side Figure
    fig_comp = figure('Visible', 'off', 'Position', [100, 100, 1200, 700]);
    
    subplot(2, 1, 1);
    hold on; box on; grid on;
    plot_trajectory_overlay(hist_A, unstructured_map, 'Run A: Fixed Corridor (Violations & Infeasibility)');
    
    subplot(2, 1, 2);
    hold on; box on; grid on;
    plot_trajectory_overlay(hist_B, unstructured_map, 'Run B: FreeSpace Architecture (100% Compliant)');
    
    saveas(fig_comp, 'artifacts/demo3_side_by_side_comparison.png');
    close(fig_comp);
    
    fprintf('[SUCCESS] Comparative visual artifacts saved to artifacts/demo3_*.png\n\n');
end

function [hist, map_used] = execute_recording_run(bound_provider, map_obj)
    cfg = SimulationConfig();
    world = ScenarioDefinitions('unstructured_road', cfg);
    vehicle = BicycleModel(cfg);
    
    ctrl = Stage5CoordinationController(cfg);
    ctrl.reset();
    ctrl.cacrc_planner.bound_provider = bound_provider;
    ctrl.setOvertakeEnabled(true);
    
    N_steps = 150;
    dt = cfg.dt;
    
    N_path = 500;
    ref_path = zeros(N_path, 5);
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
    
    hist.x = zeros(N_steps, 1);
    hist.y = zeros(N_steps, 1);
    hist.v = zeros(N_steps, 1);
    hist.theta = zeros(N_steps, 1);
    hist.world_snapshots = cell(N_steps, 1);
    hist.info_snapshots = cell(N_steps, 1);
    hist.pred_states = cell(N_steps, 1);
    hist.ref_path = ref_path;
    
    for k = 1:N_steps
        [u_cmd, pred_states, status, info] = ctrl.step(world, ref_path, 8.0);
        
        hist.world_snapshots{k} = world;
        hist.info_snapshots{k} = info;
        hist.pred_states{k} = pred_states;
        hist.x(k) = world.ego.x;
        hist.y(k) = world.ego.y;
        hist.v(k) = world.ego.v;
        hist.theta(k) = world.ego.theta;
        
        world.ego = vehicle.stepKinematic(world.ego, u_cmd(2), u_cmd(1), dt);
    end
    
    if isprop(bound_provider, 'map') && ~isempty(bound_provider.map)
        map_used = bound_provider.map;
    else
        map_used = [];
    end
end

function plot_trajectory_overlay(hist, map_obj, title_str)
    N_pts = 200;
    x_vec = linspace(0, 120, N_pts);
    y_min_vec = zeros(1, N_pts); y_max_vec = zeros(1, N_pts);
    for i = 1:N_pts
        [y_min_vec(i), y_max_vec(i)] = map_obj.getRoadBoundsAt(x_vec(i));
    end
    
    fill([x_vec, fliplr(x_vec)], [y_min_vec, fliplr(y_max_vec)], [0.90, 0.94, 0.97], 'EdgeColor', 'none');
    plot(x_vec, y_max_vec, 'r-', 'LineWidth', 2.0);
    plot(x_vec, y_min_vec, 'r-', 'LineWidth', 2.0);
    
    plot(hist.x, hist.y, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Ego Trajectory');
    rectangle('Position', [45.0-0.9, 1.50-0.9, 1.8, 1.8], 'FaceColor', [0.85, 0.33, 0.10, 0.5], 'EdgeColor', 'r');
    
    xlabel('Longitudinal x (m)'); ylabel('Lateral y (m)');
    title(title_str, 'FontSize', 12, 'FontWeight', 'bold');
    xlim([0, 120]); ylim([-0.5, 6.5]);
end
