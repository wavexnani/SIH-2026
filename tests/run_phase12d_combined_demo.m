function run_phase12d_combined_demo()
    % RUN_PHASE12D_COMBINED_DEMO Master Flagship Demonstration Artifact Generator
    %
    % Purpose:
    %   Executes the flagship 10-step driving story scenario ('complex' environment)
    %   across ideal, perception, actuator, and combined_realistic uncertainty modes
    %   and renders publication-grade comparative visual artifacts.
    
    addpath('planning', 'config', 'core', 'environment', 'stages', 'metrics', 'visualization');
    if ~exist('artifacts', 'dir'), mkdir('artifacts'); end
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 12D: FLAGSHIP COMBINED UNCERTAINTY DEMONSTRATION SUITE                    \n');
    fprintf('========================================================================================\n\n');
    
    demo_modes = {
        'ideal', 'Mode A: Ideal Ground-Truth Baseline';
        'nominal_perception', 'Mode B: Perception Noise & Delay Only';
        'steering_bias', 'Mode C: Actuator Steering Bias Only';
        'combined_realistic', 'Mode D: Combined Realistic Uncertainty'
    };
    
    for m_i = 1:size(demo_modes, 1)
        mode_key = demo_modes{m_i, 1};
        mode_label = demo_modes{m_i, 2};
        
        fprintf('Executing Flagship Demo: %s...\n', mode_label);
        
        [passed, metrics, history] = stage5_multivehicle_coordination(...
            'scenario', 'complex', ...
            'uncertainty_mode', mode_key, ...
            'seed', 42, ...
            'verbose', false);
        
        % Render Flagship Visual Artifact
        fig = figure('Visible', 'off', 'Position', [100, 100, 1000, 480]);
        plot(history.ego_x, history.ego_y, 'b-', 'LineWidth', 2.5); hold on; grid on; box on;
        yline(1.80, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference Center');
        
        xlabel('Longitudinal Position x (m)', 'FontSize', 11);
        ylabel('Lateral Position y (m)', 'FontSize', 11);
        title(sprintf('Phase 12D Flagship Demo — %s (Outcome: %s)', mode_label, metrics.outcome), ...
            'FontSize', 12, 'FontWeight', 'bold');
        
        text(history.ego_x(1) + 2, history.ego_y(1) + 0.3, 'Ego Start', 'FontWeight', 'bold', 'Color', 'blue');
        text(history.ego_x(end) - 10, history.ego_y(end) + 0.3, sprintf('Final: %.1fm', history.ego_x(end)), 'FontWeight', 'bold');
        
        filename = sprintf('artifacts/demo_phase12d_flagship_%s.png', mode_key);
        saveas(fig, filename); close(fig);
        
        fprintf('  Saved visual artifact: %s (Outcome: %s | Lat RMSE: %.3fm)\n', ...
            filename, metrics.outcome, metrics.lat_rmse);
    end
    
    fprintf('\n[SUCCESS] Phase 12D Flagship Visual Artifacts Rendered in artifacts/ directory!\n\n');
end
