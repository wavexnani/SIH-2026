function generate_proposal_figures()
    % GENERATE_PROPOSAL_FIGURES Generates publication-ready figures for SIH 2026 Proposal.
    %
    % Figures produced:
    %   1. artifacts/fig_curvature_velocity_profile.png
    %   2. artifacts/fig_steering_chatter_comparison.png
    %   3. artifacts/fig_proposal_trajectory_comparison.png
    
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir), projectRoot = pwd; else projectRoot = fileparts(current_dir); end
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'core'));
    addpath(fullfile(projectRoot, 'environment'));
    addpath(fullfile(projectRoot, 'vehicle'));
    
    data_path = fullfile(projectRoot, 'results', 'proposal_benchmark_data.mat');
    if ~exist(data_path, 'file')
        error('Benchmark data file not found: %s. Run run_proposal_ab_benchmark first!', data_path);
    end
    load(data_path, 'benchmark_results', 'traces');
    
    artifacts_dir = fullfile(projectRoot, 'artifacts');
    if ~exist(artifacts_dir, 'dir'), mkdir(artifacts_dir); end
    
    % Color Palette (Scientific Modern)
    c_blue   = [0.12, 0.47, 0.71]; % Config A Baseline
    c_green  = [0.17, 0.63, 0.17]; % Config B +Velocity
    c_purple = [0.58, 0.24, 0.70]; % Config C +Vel +Steer
    c_red    = [0.84, 0.15, 0.16]; % Threshold / Hazard
    c_gray   = [0.50, 0.50, 0.50]; % Reference / Bounds
    
    % =========================================================================
    % FIGURE 1: CURVATURE-AWARE VELOCITY PROFILING
    % =========================================================================
    fprintf('Generating Figure 1: Curvature Velocity Profile ...\n');
    fig1 = figure('Name', 'Curvature Velocity Profile', 'Color', 'w', 'Position', [100, 100, 1000, 700], 'Visible', 'off');
    
    % Generate smooth C2 quintic chicane path for crisp profile demonstration
    p = CurvatureVelocityPlanner();
    x_synth = linspace(0, 100, 500)';
    y_synth = 3.0 * ones(500, 1);
    S = @(u) 10*u.^3 - 15*u.^4 + 6*u.^5;
    for i = 1:500
        xi = x_synth(i);
        if xi < 20.0
            y_synth(i) = 3.0;
        elseif xi <= 31.32
            u = (xi - 20.0) / 11.32;
            y_synth(i) = 3.0 + 1.20 * S(u);
        elseif xi <= 40.0
            y_synth(i) = 4.20;
        elseif xi <= 56.02
            u = (xi - 40.0) / 16.02;
            y_synth(i) = 4.20 - 2.40 * S(u);
        elseif xi <= 65.0
            y_synth(i) = 1.80;
        elseif xi <= 76.32
            u = (xi - 65.0) / 11.32;
            y_synth(i) = 1.80 + 1.20 * S(u);
        else
            y_synth(i) = 3.0;
        end
    end
    
    [kappa_synth, s_synth, ~] = p.computeCurvature(x_synth, y_synth);
    ref_mat = [x_synth, y_synth, zeros(500, 2), 10.0 * ones(500, 1)];
    ref_prof = p.planVelocityProfile(ref_mat, 10.0);
    v_prof = ref_prof(:, 5);
    
    % Panel 1: Curvature Profile
    subplot(3, 1, 1);
    plot(s_synth, abs(kappa_synth), 'Color', [0.2, 0.2, 0.2], 'LineWidth', 2.0);
    grid on; box on;
    ylabel('Curvature |\kappa| (m^{-1})', 'FontSize', 11, 'FontWeight', 'bold');
    title('Anticipatory Curvature-Aware Speed Profiling along Chicane (R_{min}=18.5 m)', 'FontSize', 13, 'FontWeight', 'bold');
    xlim([0, 100]);
    yline(1/18.5, '--', 'R = 18.5 m Apex (|\kappa| \approx 0.054 m^{-1})', 'Color', c_red, 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
    
    % Panel 2: Velocity Profile
    subplot(3, 1, 2);
    plot(s_synth, 10.0 * ones(size(s_synth)), '--', 'Color', c_blue, 'LineWidth', 2.0, 'DisplayName', 'Config A: Flat Cruise (10.0 m/s)');
    hold on;
    plot(s_synth, v_prof, 'Color', c_green, 'LineWidth', 2.5, 'DisplayName', 'Config B/C: Curvature Velocity Planner');
    grid on; box on;
    ylabel('Speed v (m/s)', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'southwest', 'FontSize', 10);
    xlim([0, 100]); ylim([4.0, 11.0]);
    
    % Annotation callouts
    [min_v, min_v_idx] = min(v_prof);
    text(s_synth(min_v_idx), min_v - 0.4, sprintf('Apex Speed = %.2f m/s\n(Pre-Braked)', min_v), ...
        'FontSize', 9, 'FontWeight', 'bold', 'Color', c_green, 'HorizontalAlignment', 'center');
    
    % Panel 3: Resulting Lateral Acceleration a_lat = v^2 * kappa (Computed directly from data)
    subplot(3, 1, 3);
    a_lat_flat = (10.0^2) .* abs(kappa_synth);
    a_lat_prof = (v_prof.^2) .* abs(kappa_synth);
    peak_flat = max(a_lat_flat);
    peak_prof = max(a_lat_prof);
    
    plot(s_synth, a_lat_flat, '--', 'Color', c_blue, 'LineWidth', 1.8, ...
        'DisplayName', sprintf('Flat Cruise (Peak a_{lat} = %.2f m/s^2)', peak_flat));
    hold on;
    plot(s_synth, a_lat_prof, 'Color', c_green, 'LineWidth', 2.2, ...
        'DisplayName', sprintf('Curvature-Aware (Peak a_{lat} = %.2f m/s^2, Comfort Bound Preserved)', peak_prof));
    yline(2.50, 'r-', 'Passenger Comfort Limit (a_{lat,max} = 2.50 m/s^2)', 'LineWidth', 1.8, 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
    grid on; box on;
    xlabel('Path Arc Length s (m)', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Lateral Accel a_{lat} (m/s^2)', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 10);
    xlim([0, 100]); ylim([0, max(6.0, ceil(peak_flat))]);
    
    out_fig1 = fullfile(artifacts_dir, 'fig_curvature_velocity_profile.png');
    exportgraphics(fig1, out_fig1, 'Resolution', 300);
    close(fig1);
    fprintf('Saved: %s\n', out_fig1);
    
    % =========================================================================
    % FIGURE 2: STEERING CHATTER & STABILIZATION COMPARISON
    % =========================================================================
    fprintf('Generating Figure 2: Steering Chatter Comparison ...\n');
    fig2 = figure('Name', 'Steering Chatter Comparison', 'Color', 'w', 'Position', [150, 150, 1000, 650], 'Visible', 'off');
    
    % Extract Chicane traces
    trA = traces.chicane_ConfigABaseline;
    trC = traces.chicane_ConfigCVelSteer;
    
    % Align time horizons
    t_plot_max = min(trA.t(end), trC.t(end));
    idxA = trA.t <= t_plot_max;
    idxC = trC.t <= t_plot_max;
    
    rev_A = benchmark_results.chicane_ConfigABaseline.reversal_count;
    rev_C = benchmark_results.chicane_ConfigCVelSteer.reversal_count;
    
    % Panel 1: Commanded Steering Angle
    subplot(2, 1, 1);
    plot(trA.t(idxA), rad2deg(trA.delta_cmd(idxA)), 'Color', c_blue, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Config A (Baseline Raw MPC): %d Reversals', rev_A));
    hold on;
    plot(trC.t(idxC), rad2deg(trC.delta_cmd(idxC)), 'Color', c_purple, 'LineWidth', 2.2, ...
        'DisplayName', sprintf('Config C (+Steering Filter): %d Reversals', rev_C));
    grid on; box on;
    ylabel('Steering Command \delta_{cmd} (deg)', 'FontSize', 11, 'FontWeight', 'bold');
    title('Actuator-Aware Steering Command Stabilization (Jitter & Sign-Reversal Suppression)', 'FontSize', 13, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 10);
    xlim([0, t_plot_max]);
    
    % Panel 2: Commanded Steering Slew Rate (Delta delta / dt)
    subplot(2, 1, 2);
    dt = 0.10;
    rateA = [0; diff(trA.delta_cmd(idxA))] / dt;
    rateC = [0; diff(trC.delta_cmd(idxC))] / dt;
    plot(trA.t(idxA), rad2deg(rateA), 'Color', [c_blue, 0.6], 'LineWidth', 1.2, 'DisplayName', 'Config A Slew Rate (High Jitter)');
    hold on;
    plot(trC.t(idxC), rad2deg(rateC), 'Color', c_purple, 'LineWidth', 2.0, 'DisplayName', 'Config C Slew Rate (Rate-Limited & Filtered)');
    yline(rad2deg(0.50), 'r--', 'EPS Slew Limit (+28.6 deg/s)', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    yline(-rad2deg(0.50), 'r--', '-28.6 deg/s', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    grid on; box on;
    xlabel('Simulation Time t (s)', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Steering Rate d\delta/dt (deg/s)', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'southwest', 'FontSize', 10);
    xlim([0, t_plot_max]);
    
    out_fig2 = fullfile(artifacts_dir, 'fig_steering_chatter_comparison.png');
    exportgraphics(fig2, out_fig2, 'Resolution', 300);
    close(fig2);
    fprintf('Saved: %s\n', out_fig2);
    
    % =========================================================================
    % FIGURE 3: TOP-DOWN TRAJECTORY COMPARISON (BOTTLENECK & CHICANE)
    % =========================================================================
    fprintf('Generating Figure 3: Proposal Trajectory Comparison ...\n');
    fig3 = figure('Name', 'Proposal Trajectory Comparison', 'Color', 'w', 'Position', [200, 200, 1100, 750], 'Visible', 'off');
    
    % Panel 1: Bottleneck 2.0m Passage
    subplot(2, 1, 1);
    trA_bottleneck = traces.bottleneck_ConfigABaseline;
    trB_bottleneck = traces.bottleneck_ConfigBVelocity;
    trC_bottleneck = traces.bottleneck_ConfigCVelSteer;
    
    % Draw road boundaries
    x_fill = [0, 80, 80, 0];
    y_fill = [0, 0, 6, 6];
    patch(x_fill, y_fill, [0.95, 0.95, 0.95], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    hold on;
    yline(0, 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
    yline(6, 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
    
    % Draw static obstacles
    rectangle('Position', [38.0 - 1.1, 1.0 - 0.75, 2.2, 1.5], 'FaceColor', [0.85, 0.35, 0.35], 'EdgeColor', 'r', 'LineWidth', 1.5);
    text(38.0, 1.0, 'Rickshaw', 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'w');
    
    rectangle('Position', [42.0 - 2.25, 4.5 - 0.75, 4.5, 1.5], 'FaceColor', [0.85, 0.35, 0.35], 'EdgeColor', 'r', 'LineWidth', 1.5);
    text(42.0, 4.5, 'Stall Debris', 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'w');
    
    % Bottleneck passage callout
    plot([39, 41], [1.75, 3.75], 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    text(40.5, 2.75, '2.0m Gap', 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');
    
    % Trajectories
    plot(trA_bottleneck.ego_x, trA_bottleneck.ego_y, '--', 'Color', c_blue, 'LineWidth', 2.0, 'DisplayName', 'Config A (Baseline Free-Space)');
    plot(trB_bottleneck.ego_x, trB_bottleneck.ego_y, 'Color', c_green, 'LineWidth', 2.5, 'DisplayName', 'Config B (+Velocity Profile)');
    plot(trC_bottleneck.ego_x, trC_bottleneck.ego_y, ':', 'Color', c_purple, 'LineWidth', 2.2, 'DisplayName', 'Config C (+Vel +Steer)');
    
    grid on; box on; axis equal;
    xlim([10, 75]); ylim([-0.5, 6.5]);
    xlabel('Longitudinal Position X (m)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('Lateral Position Y (m)', 'FontSize', 10, 'FontWeight', 'bold');
    title('Indian Road Bottleneck Passage: 2.0m Clearance Corridor Navigation', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 9);
    
    % Panel 2: Chicane Stress Test Trajectories
    subplot(2, 1, 2);
    trA_chicane = traces.chicane_ConfigABaseline;
    trB_chicane = traces.chicane_ConfigBVelocity;
    trC_chicane = traces.chicane_ConfigCVelSteer;
    
    % Draw road corridor [0, 6]
    patch([0, 110, 110, 0], [0, 0, 6, 6], [0.95, 0.95, 0.95], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    hold on;
    yline(0, 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
    yline(6, 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
    
    peak_a_chicane = benchmark_results.chicane_ConfigABaseline.peak_lat_accel;
    peak_b_chicane = benchmark_results.chicane_ConfigBVelocity.peak_lat_accel;
    peak_c_chicane = benchmark_results.chicane_ConfigCVelSteer.peak_lat_accel;
    
    % Reference centerline
    plot(x_synth, y_synth, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Reference Centerline (R_{min}=18.5m)');
    plot(trA_chicane.ego_x, trA_chicane.ego_y, '--', 'Color', c_blue, 'LineWidth', 2.0, ...
        'DisplayName', sprintf('Config A: Baseline (Road Bound Stop at x=65.3m, Peak a_{lat}=%.2f m/s^2)', peak_a_chicane));
    plot(trB_chicane.ego_x, trB_chicane.ego_y, 'Color', c_green, 'LineWidth', 2.2, ...
        'DisplayName', sprintf('Config B: +Velocity (100%% Completed to 100.2m, Peak a_{lat}=%.2f m/s^2)', peak_b_chicane));
    plot(trC_chicane.ego_x, trC_chicane.ego_y, 'Color', c_purple, 'LineWidth', 2.2, ...
        'DisplayName', sprintf('Config C: +Vel +Steering (Safety Stop at x=65.5m, Peak a_{lat}=%.2f m/s^2)', peak_c_chicane));
    
    grid on; box on; axis equal;
    xlim([0, 105]); ylim([-0.5, 6.5]);
    xlabel('Longitudinal Position X (m)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('Lateral Position Y (m)', 'FontSize', 10, 'FontWeight', 'bold');
    title('R=18.5m Chicane Stress Test: Lateral Dynamics & Stability Comparison', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 9);
    
    out_fig3 = fullfile(artifacts_dir, 'fig_proposal_trajectory_comparison.png');
    exportgraphics(fig3, out_fig3, 'Resolution', 300);
    close(fig3);
    fprintf('Saved: %s\n', out_fig3);
    
    % Also copy to antigravity-ide artifact directory for embedding in conversation artifacts
    agy_artifact_dir = '/home/yeswanth/.gemini/antigravity-ide/brain/f2ad1a70-7254-460c-9ae9-6077321f4ae8';
    if exist(agy_artifact_dir, 'dir')
        copyfile(out_fig1, fullfile(agy_artifact_dir, 'fig_curvature_velocity_profile.png'));
        copyfile(out_fig2, fullfile(agy_artifact_dir, 'fig_steering_chatter_comparison.png'));
        copyfile(out_fig3, fullfile(agy_artifact_dir, 'fig_proposal_trajectory_comparison.png'));
        fprintf('Copied all 3 figures to Antigravity IDE artifact directory!\n');
    end
end
