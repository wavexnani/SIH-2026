function run_phase13_gallery_generator()
    % RUN_PHASE13_GALLERY_GENERATOR Generate Failure/Success Case Visual Gallery
    %
    % Renders 8 representative multi-panel closed-loop visual cases into artifacts/phase13/:
    %   1. case1_nominal_success.png
    %   2. case2_perception_degraded_success.png
    %   3. case3_actuator_degraded_success.png
    %   4. case4_successful_safe_stop.png
    %   5. case5_tight_valid_overtake.png
    %   6. case6_corridor_squeeze_failed_overtake.png
    %   7. case7_oncoming_conflict_yield.png
    %   8. case8_combined_uncertainty_degradation.png
    
    addpath('environment', 'core', 'vehicle', 'planning', 'config', 'stages', 'metrics', 'visualization');
    if ~exist('artifacts/phase13', 'dir'), mkdir('artifacts/phase13'); end
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 13: GENERATING CLOSED-LOOP FAILURE/SUCCESS CASE VISUAL GALLERY            \n');
    fprintf('========================================================================================\n\n');
    
    gallery_configs = {
        struct('id', 1, 'name', 'case1_nominal_success', 'scen', 'clear', 'mode', 'ideal', 'title', 'Case 1: Nominal Success (Ideal Mode)'), ...
        struct('id', 2, 'name', 'case2_perception_degraded_success', 'scen', 'static', 'mode', 'nominal_perception', 'title', 'Case 2: Perception-Degraded Success (Noise)'), ...
        struct('id', 3, 'name', 'case3_actuator_degraded_success', 'scen', 'multi_obstacle_sequence', 'mode', 'steering_bias', 'title', 'Case 3: Actuator-Degraded Success (+0.8 deg Bias)'), ...
        struct('id', 4, 'name', 'case4_successful_safe_stop', 'scen', 'impassable_center', 'mode', 'ideal', 'title', 'Case 4: Successful Safe Stop (Total Road Blockage)'), ...
        struct('id', 5, 'name', 'case5_tight_valid_overtake', 'scen', 'multi_vehicle_following', 'mode', 'ideal', 'title', 'Case 5: Tight Valid Car-Following & Overtake'), ...
        struct('id', 6, 'name', 'case6_corridor_squeeze_failed_overtake', 'scen', 'multi_vehicle_yield_overtake', 'mode', 'combined_realistic', 'title', 'Case 6: Spatial Corridor Squeeze & Yield Interaction'), ...
        struct('id', 7, 'name', 'case7_oncoming_conflict_yield', 'scen', 'multi_vehicle_oncoming_conflict', 'mode', 'ideal', 'title', 'Case 7: Oncoming Conflict Yield Maneuver'), ...
        struct('id', 8, 'name', 'case8_combined_uncertainty_degradation', 'scen', 'complex', 'mode', 'combined_realistic', 'title', 'Case 8: Combined Uncertainty Disturbance') ...
    };

    for g_i = 1:length(gallery_configs)
        cfg_item = gallery_configs{g_i};
        fprintf('  Rendering [%d/8] %s (%s, Mode: %s)...\n', ...
            g_i, cfg_item.title, cfg_item.scen, cfg_item.mode);
        
        [~, met, hist] = stage5_multivehicle_coordination(...
            'scenario', cfg_item.scen, ...
            'uncertainty_mode', cfg_item.mode, ...
            'seed', 42, ...
            'verbose', false);
        
        realism_met = RealismMetrics.compute(hist, struct(), SimulationConfig());
        
        fig = figure('Visible', 'off', 'Position', [50, 50, 1100, 750]);
        
        % Panel 1: 2D Spatial Trajectory
        subplot(2, 2, 1);
        plot(hist.ego_x, hist.ego_y, 'b-', 'LineWidth', 2, 'DisplayName', 'Ego Trajectory'); hold on;
        plot(hist.ego_x(1), hist.ego_y(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'Start');
        plot(hist.ego_x(end), hist.ego_y(end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'End');
        yline(0, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        yline(6.0, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        xlabel('X Position (m)'); ylabel('Y Position (m)');
        title(sprintf('2D Trajectory | Outcome: %s', met.outcome), 'FontSize', 10, 'FontWeight', 'bold');
        grid on; legend('Location', 'best');
        
        % Panel 2: Speed & Acceleration Profiles
        subplot(2, 2, 2);
        plot(hist.t, hist.ego_v, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Speed (m/s)'); hold on;
        plot(hist.t, hist.a_actual_cmd, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Accel (m/s^2)');
        xlabel('Time (s)'); ylabel('Kinematics');
        title(sprintf('Speed & Accel Profile (Max Accel: %.2f g)', realism_met.motion.max_lat_accel_g), 'FontSize', 10);
        grid on; legend('Location', 'best');
        
        % Panel 3: Steering Angle & Control Profile
        subplot(2, 2, 3);
        plot(hist.t, rad2deg(hist.delta_actual_cmd), 'm-', 'LineWidth', 1.5, 'DisplayName', 'Actual Delta (deg)'); hold on;
        plot(hist.t, rad2deg(hist.delta_cmd), 'k:', 'LineWidth', 1.2, 'DisplayName', 'Cmd Delta (deg)');
        xlabel('Time (s)'); ylabel('Steering Delta (deg)');
        title(sprintf('Steering Control (Lat RMSE: %.3fm)', realism_met.tracking.lat_rmse), 'FontSize', 10);
        grid on; legend('Location', 'best');
        
        % Panel 4: Minimum Clearance & TTC Profile
        subplot(2, 2, 4);
        clrs = hist.min_clearance; clrs(isinf(clrs)) = 15.0;
        plot(hist.t, clrs, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Clearance (m)'); hold on;
        yline(0.0, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Collision Threshold (0m)');
        xlabel('Time (s)'); ylabel('Clearance (m)');
        title(sprintf('Body-to-Body Clearance (Min: %.3fm)', realism_met.safety.min_obstacle_clearance), 'FontSize', 10);
        grid on; legend('Location', 'best');
        
        sgtitle(sprintf('%s | Outcome: %s | Mode: %s', cfg_item.title, met.outcome, cfg_item.mode), 'FontSize', 12, 'FontWeight', 'bold');
        
        out_path = sprintf('artifacts/phase13/%s.png', cfg_item.name);
        saveas(fig, out_path);
        close(fig);
        fprintf('     Saved: %s\n', out_path);
    end
    fprintf('\n[SUCCESS] Phase 13 Failure/Success Visual Case Gallery Rendered!\n\n');
end
