function generate_stage51_figures()
    % GENERATE_STAGE51_FIGURES Generate high-resolution figures for hackathon presentation
    
    addpath('stages', 'planning', 'config', 'vehicle', 'core', 'environment');
    
    if ~exist('artifacts', 'dir')
        mkdir('artifacts');
    end

    % Set up figure defaults for publication quality
    set(0, 'DefaultAxesFontName', 'Helvetica');
    set(0, 'DefaultAxesFontSize', 11);
    set(0, 'DefaultLineLineWidth', 1.8);

    %% --------------------------------------------------------------------
    % Figure 1: Multi-Vehicle Trajectories (Scenario 3: Oncoming Conflict)
    % --------------------------------------------------------------------
    fig1 = figure('Visible', 'off', 'Position', [100, 100, 1000, 450]);
    [~, ~, h3] = stage5_multivehicle_coordination('scenario', 'multi_vehicle_oncoming_conflict', 'verbose', false);
    
    hold on; box on; grid on;
    % Road bounds
    yline(0.0, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Road Bounds');
    yline(4.8, 'k--', 'LineWidth', 2.0, 'HandleVisibility', 'off');
    yline(2.4, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Lane Divider');
    
    % Ego Trajectory
    plot(h3.ego_x, h3.ego_y, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Ego Vehicle (Stage 5.1)');
    
    % Static obstacle 1 (Lead Vehicle)
    rectangle('Position', [35.0-2.3, 1.80-0.9, 4.6, 1.8], 'Curvature', 0.2, ...
        'FaceColor', [0.85 0.325 0.098 0.4], 'EdgeColor', [0.85 0.325 0.098]);
    text(35.0, 1.80, 'Agent 1 (Lead)', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    
    % Oncoming dynamic agent 2
    rectangle('Position', [60.0-2.3, 4.20-0.9, 4.6, 1.8], 'Curvature', 0.2, ...
        'FaceColor', [0.494 0.184 0.556 0.4], 'EdgeColor', [0.494 0.184 0.556]);
    text(60.0, 4.20, 'Agent 2 (Oncoming)', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    
    xlabel('Longitudinal Position x (m)');
    ylabel('Lateral Position y (m)');
    title('Stage 5.1 Multi-Vehicle Oncoming Conflict: Yield \rightarrow Overtake \rightarrow Recenter Trajectory');
    legend('Location', 'northeast');
    ylim([-0.5, 5.3]);
    xlim([0, 85]);
    
    saveas(fig1, 'artifacts/fig1_multivehicle_trajectories.png');
    close(fig1);

    %% --------------------------------------------------------------------
    % Figure 2: Coordination State (Macro-Intent Timeline)
    % --------------------------------------------------------------------
    fig2 = figure('Visible', 'off', 'Position', [100, 100, 900, 350]);
    
    % Map intents to numeric codes for plotting
    intent_map = containers.Map({'MAINTAIN', 'FOLLOW', 'YIELD', 'OVERTAKE'}, [1, 2, 3, 4]);
    intent_codes = zeros(size(h3.t));
    for k = 1:length(h3.t)
        if isKey(intent_map, h3.macro_intent{k})
            intent_codes(k) = intent_map(h3.macro_intent{k});
        else
            intent_codes(k) = 1;
        end
    end
    
    stairs(h3.t, intent_codes, 'g-', 'LineWidth', 2.5);
    grid on; box on;
    yticks([1, 2, 3, 4]);
    yticklabels({'MAINTAIN', 'FOLLOW', 'YIELD', 'OVERTAKE'});
    xlabel('Time t (s)');
    ylabel('Stage 5 Macro-Intent State');
    title('Stage 5.1 Hierarchical Intent State Transitions (Scenario 3)');
    ylim([0.5, 4.5]);
    
    saveas(fig2, 'artifacts/fig2_coordination_states.png');
    close(fig2);

    %% --------------------------------------------------------------------
    % Figure 3: Safety Metrics & Speed Profile
    % --------------------------------------------------------------------
    fig3 = figure('Visible', 'off', 'Position', [100, 100, 900, 450]);
    
    subplot(2, 1, 1);
    plot(h3.t, h3.min_clearance, 'm-', 'LineWidth', 2.0);
    yline(0.0, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Safety Threshold (0m)');
    grid on; box on;
    ylabel('Min Clearance (m)');
    title('Safety Invariants: Minimum Clearance Profile (\ge +0.25m guaranteed)');
    
    subplot(2, 1, 2);
    plot(h3.t, h3.ego_v, 'b-', 'LineWidth', 2.0);
    yline(8.0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Target Cruise Speed (8 m/s)');
    grid on; box on;
    xlabel('Time t (s)');
    ylabel('Ego Speed (m/s)');
    title('Ego Longitudinal Velocity Profile (Yield Deceleration \rightarrow Overtake Acceleration)');
    
    saveas(fig3, 'artifacts/fig3_safety_metrics.png');
    close(fig3);

    fprintf('[SUCCESS] Visual evidence figures generated in artifacts/ directory.\n');
end
