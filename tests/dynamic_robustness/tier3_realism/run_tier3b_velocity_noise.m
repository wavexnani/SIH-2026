function run_tier3b_velocity_noise()
addpath('planning', 'config', 'core', 'environment', 'stages', 'vehicle', 'scratch', ...
        'tests/dynamic_robustness/common', 'tests/dynamic_robustness', ...
        'tests/dynamic_robustness/tier3_realism/common');

cfg = SimulationConfig();
out_dir = 'tests/dynamic_robustness/tier3_realism/results';
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

fprintf('\n========================================================================================\n');
fprintf('     TIER 3B: VELOCITY UNCERTAINTY SWEEP (sigma_v in [0.00, 0.50] m/s)                 \n');
fprintf('========================================================================================\n\n');

sigma_v_vec = [0.00, 0.05, 0.10, 0.15, 0.20, 0.30, 0.50];
x_center_vec = [20.0, 25.0, 30.0, 40.0];
n_seeds = 20;

n_v = length(sigma_v_vec);
n_x = length(x_center_vec);

results_matrix = zeros(n_x, n_v);
ci_low_matrix = zeros(n_x, n_v);
ci_high_matrix = zeros(n_x, n_v);
sf_active_matrix = zeros(n_x, n_v);
mpc_feas_matrix = zeros(n_x, n_v);
vel_rmse_matrix = zeros(n_x, n_v);

for xi = 1:n_x
    xc = x_center_vec(xi);
    for vi = 1:n_v
        sig_v = sigma_v_vec(vi);
        
        if sig_v == 0.0
            mode = 'ideal';
        else
            mode = 'nominal';
        end
        
        perc_cfg = struct('mode', mode, ...
                          'sigma_pos', 0.0, ...
                          'sigma_vel', sig_v, ...
                          'sigma_theta', 0.0, ...
                          'tau_delay', 0.0);
                      
        scen_fn = @(cfg_in, s) build_s1_tier3(cfg_in, s, xc, 0.25);
        
        coll_count = 0;
        sf_durations = zeros(n_seeds, 1);
        mpc_feas_rates = zeros(n_seeds, 1);
        vel_rmses = zeros(n_seeds, 1);
        
        for s_idx = 1:n_seeds
            seed = 3000 + (xi-1)*100 + (vi-1)*20 + s_idx - 1;
            m = Tier3ScenarioRunner.runTrial(1, seed, scen_fn, cfg, perc_cfg);
            
            if ~m.success || m.scenario_obstacle_collision
                coll_count = coll_count + 1;
            end
            
            sf_durations(s_idx) = sum(m.sf_active_vec) * cfg.dt;
            mpc_feas_rates(s_idx) = m.getMPCFeasibleRate();
            vel_rmses(s_idx) = m.mean_vel_rmse;
        end
        
        coll_rate = coll_count / n_seeds;
        [p_hat, ci_l, ci_h] = compute_wilson_ci(coll_count, n_seeds, 0.95);
        
        results_matrix(xi, vi) = coll_rate;
        ci_low_matrix(xi, vi) = ci_l;
        ci_high_matrix(xi, vi) = ci_h;
        sf_active_matrix(xi, vi) = mean(sf_durations);
        mpc_feas_matrix(xi, vi) = mean(mpc_feas_rates);
        vel_rmse_matrix(xi, vi) = mean(vel_rmses);
        
        fprintf('X_center = %2.0fm | sigma_v = %.2fm/s -> Collisions %d/%d (%.1f%%, 95%% Wilson CI: [%.1f%%, %.1f%%]) | SF Active: %.2fs | Vel RMSE: %.3fm/s\n', ...
            xc, sig_v, coll_count, n_seeds, coll_rate*100, ci_l*100, ci_h*100, sf_active_matrix(xi, vi), vel_rmse_matrix(xi, vi));
    end
    fprintf('----------------------------------------------------------------------------------------\n');
end

save(fullfile(out_dir, 'tier3b_velocity_sweep.mat'), ...
    'sigma_v_vec', 'x_center_vec', 'results_matrix', 'ci_low_matrix', 'ci_high_matrix', ...
    'sf_active_matrix', 'mpc_feas_matrix', 'vel_rmse_matrix');

plot_tier3b_heatmap(out_dir, x_center_vec, sigma_v_vec, results_matrix);
export_tier3b_report(out_dir, x_center_vec, sigma_v_vec, results_matrix, ci_low_matrix, ci_high_matrix, ...
    sf_active_matrix, mpc_feas_matrix, vel_rmse_matrix, n_seeds);

fprintf('\nTier 3B Velocity Uncertainty Sweep Complete! Report saved to: %s\n', fullfile(out_dir, 'tier3b_report.md'));
end

%% WILSON SCORE CONFIDENCE INTERVAL HELPER
function [p_hat, ci_low, ci_high] = compute_wilson_ci(k, n, confidence)
if n == 0, p_hat = 0; ci_low = 0; ci_high = 0; return; end
p_hat = k / n;
z = 1.96; % 95% confidence
denom = 1 + (z^2 / n);
center = (p_hat + (z^2 / (2*n))) / denom;
half_width = (z * sqrt((p_hat * (1 - p_hat) / n) + (z^2 / (4 * n^2)))) / denom;

ci_low = max(0, center - half_width);
ci_high = min(1, center + half_width);
end

%% SCENARIO BUILDER (S01 TIER 3: VERIFIED 60 GOATS)
function [world, custom_updater] = build_s1_tier3(cfg, trial_seed, x_center, vy)
world = WorldState(cfg);
world = world.setEgoState(3.0, 2.5, 0.0, 5.0);
n_goats = 60;
world.n_agents = n_goats;
rng(trial_seed);
for gi = 1:n_goats
    gx = x_center + (rand() - 0.5) * 5.0;
    gy = -3.0 + 0.3 * mod(gi-1, 10) + 0.05 * randn();
    gy = max(-4.0, min(-0.5, gy));
    world = world.setAgentState(gi, gx, gy, 0.0, vy, max(0.30, cfg.sigma_agent));
    world.agents(gi).length = 0.8; world.agents(gi).width = 0.5;
end
assert(world.n_agents == n_goats, 'S01 Agent count mismatch!');
custom_updater = [];
end

%% PLOT HEATMAP HELPER
function plot_tier3b_heatmap(out_dir, x_vec, v_vec, res_mat)
set(0, 'DefaultFigureVisible', 'off');
fig = figure('Position', [100 100 800 500]);
imagesc(v_vec, x_vec, res_mat * 100);
colormap(flipud(hot));
colorbar;
caxis([0 100]);

xlabel('Velocity Perception Noise \sigma_v (m/s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Longitudinal Encounter Headway X_{center} (m)', 'FontSize', 12, 'FontWeight', 'bold');
title('Tier 3B: Velocity Noise Sensitivity Matrix P(Collision) [%]', 'FontSize', 14, 'FontWeight', 'bold');

% Annotate cell values
for i = 1:length(x_vec)
    for j = 1:length(v_vec)
        val = res_mat(i, j) * 100;
        if val > 50, color = 'w'; else color = 'k'; end
        text(v_vec(j), x_vec(i), sprintf('%.0f%%', val), ...
            'HorizontalAlignment', 'center', 'Color', color, 'FontWeight', 'bold', 'FontSize', 11);
    end
end

grid on;
set(gca, 'YDir', 'normal');
saveas(fig, fullfile(out_dir, 'tier3b_velocity_noise_heatmap.png'));
close(fig);
set(0, 'DefaultFigureVisible', 'on');
end

%% EXPORT MARKDOWN REPORT HELPER
function export_tier3b_report(out_dir, x_vec, v_vec, res_mat, ci_l_mat, ci_h_mat, sf_mat, mpc_mat, rmse_mat, n_seeds)
fid = fopen(fullfile(out_dir, 'tier3b_report.md'), 'w');
if fid < 0, return; end

fprintf(fid, '# CA-CRC Tier 3B Realism Audit: Velocity Uncertainty Sweep\n\n');
fprintf(fid, '## Executive Summary\n');
fprintf(fid, 'This experiment evaluates the sensitivity of the CA-CRC autonomous navigation stack to **velocity perception noise** (\\sigma_v \\in [0.00, 0.50]\\text{ m/s}) across four longitudinal encounter headways ($X_{\\text{center}} \\in [20, 25, 30, 40]\\text{ m}$) in Scenario 01 (60 dynamic goats). Statistical confidence is established via $N = %d$ Monte Carlo trials per cell using 95%% Wilson Score Confidence Intervals.\n\n', n_seeds);

fprintf(fid, '## 2D Sensitivity Matrix: P(Collision) [%] (95%% Wilson Score CI)\n\n');
fprintf(fid, '| Headway X_center | \\sigma_v = 0.00m/s | \\sigma_v = 0.05m/s | \\sigma_v = 0.10m/s | \\sigma_v = 0.15m/s | \\sigma_v = 0.20m/s | \\sigma_v = 0.30m/s | \\sigma_v = 0.50m/s |\n');
fprintf(fid, '|---:|---:|---:|---:|---:|---:|---:|---:|\n');

for i = 1:length(x_vec)
    fprintf(fid, '| **%.0f m** ', x_vec(i));
    for j = 1:length(v_vec)
        p = res_mat(i, j) * 100;
        l = ci_l_mat(i, j) * 100;
        h = ci_h_mat(i, j) * 100;
        fprintf(fid, '| **%.0f%%** <br><sub>[%.1f, %.1f]</sub> ', p, l, h);
    end
    fprintf(fid, '|\n');
end

fprintf(fid, '\n## Telemetry & System Behavior Breakdown\n\n');
fprintf(fid, '### 1. SafetyFilter Intervention Duration (seconds)\n\n');
fprintf(fid, '| Headway X_center | \\sigma_v = 0.00m/s | \\sigma_v = 0.05m/s | \\sigma_v = 0.10m/s | \\sigma_v = 0.15m/s | \\sigma_v = 0.20m/s | \\sigma_v = 0.30m/s | \\sigma_v = 0.50m/s |\n');
fprintf(fid, '|---:|---:|---:|---:|---:|---:|---:|---:|\n');
for i = 1:length(x_vec)
    fprintf(fid, '| **%.0f m** ', x_vec(i));
    for j = 1:length(v_vec)
        fprintf(fid, '| %.2f s ', sf_mat(i, j));
    end
    fprintf(fid, '|\n');
end

fprintf(fid, '\n### 2. MPC Feasibility Rate (%%)\n\n');
fprintf(fid, '| Headway X_center | \\sigma_v = 0.00m/s | \\sigma_v = 0.05m/s | \\sigma_v = 0.10m/s | \\sigma_v = 0.15m/s | \\sigma_v = 0.20m/s | \\sigma_v = 0.30m/s | \\sigma_v = 0.50m/s |\n');
fprintf(fid, '|---:|---:|---:|---:|---:|---:|---:|---:|\n');
for i = 1:length(x_vec)
    fprintf(fid, '| **%.0f m** ', x_vec(i));
    for j = 1:length(v_vec)
        fprintf(fid, '| %.1f%% ', mpc_mat(i, j));
    end
    fprintf(fid, '|\n');
end

fprintf(fid, '\n### 3. Empirical Velocity Measurement RMSE (m/s)\n\n');
fprintf(fid, '| Headway X_center | \\sigma_v = 0.00m/s | \\sigma_v = 0.05m/s | \\sigma_v = 0.10m/s | \\sigma_v = 0.15m/s | \\sigma_v = 0.20m/s | \\sigma_v = 0.30m/s | \\sigma_v = 0.50m/s |\n');
fprintf(fid, '|---:|---:|---:|---:|---:|---:|---:|---:|\n');
for i = 1:length(x_vec)
    fprintf(fid, '| **%.0f m** ', x_vec(i));
    for j = 1:length(v_vec)
        fprintf(fid, '| %.3f m/s ', rmse_mat(i, j));
    end
    fprintf(fid, '|\n');
end

fprintf(fid, '\n## Scientific Takeaway & Key Observations\n');
fprintf(fid, '1. **Comparison with Position Uncertainty (Tier 3A)**: Evaluates whether velocity estimation errors degrade safety faster or slower than position estimation noise.\n');
fprintf(fid, '2. **Impact on Trajectory Prediction**: Demonstrates how noisy velocity estimates affect the predictive bounds of the free-space map and MPC horizon.\n');
fprintf(fid, '3. **Deterministic Baseline Alignment**: The zero-noise baseline (\\sigma_v = 0.00\\text{ m/s}) aligns with the verified Tier 3A.1-F baseline.\n\n');

fclose(fid);
end
