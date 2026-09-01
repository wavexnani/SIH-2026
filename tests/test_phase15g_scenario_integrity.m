function test_phase15g_scenario_integrity()
    % TEST_PHASE15G_SCENARIO_INTEGRITY Forensic Scenario Ladder Integrity Test
    %
    % Verifies Level 8 and Level 9 scenario mappings, physical world definitions,
    % benchmark harness scenario resolution, and closed-loop trajectory identity.
    
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir), projectRoot = pwd; else projectRoot = fileparts(current_dir); end
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'config'));
    addpath(fullfile(projectRoot, 'vehicle'));
    addpath(fullfile(projectRoot, 'core'));
    addpath(fullfile(projectRoot, 'environment'));
    addpath(fullfile(projectRoot, 'stages'));
    addpath(fullfile(projectRoot, 'metrics'));
    addpath(fullfile(projectRoot, 'artifacts'));
    addpath(fullfile(projectRoot, 'tests'));
    
    fprintf('\n========================================================================================\n');
    fprintf('  PHASE 15G — LEVEL 8 / LEVEL 9 SCENARIO INTEGRITY & BENCHMARK AUDIT                     \n');
    fprintf('========================================================================================\n\n');
    
    cfg = SimulationConfig();
    
    %% 1. Verify Level 8 Mapping
    [l8_key, l8_name] = ScenarioLadder.getLevel(8);
    fprintf('Level 8 Mapping: Name = "%s", Scenario Key = "%s"\n', l8_name, l8_key);
    assert(strcmp(l8_key, 'complex'), 'Level 8 key must be "complex"');
    
    %% 2. Verify Level 9 Mapping
    [l9_key, l9_name] = ScenarioLadder.getLevel(9);
    fprintf('Level 9 Mapping: Name = "%s", Scenario Key = "%s"\n', l9_name, l9_key);
    assert(strcmp(l9_key, 'complex'), 'Level 9 key must be "complex"');
    
    %% 3. Physical Scenario Definition Comparison
    w8 = ScenarioDefinitions(l8_key, cfg);
    w9 = ScenarioDefinitions(l9_key, cfg);
    
    fprintf('\n--- PHYSICAL SCENARIO DEFINITION COMPARISON ---\n');
    fprintf('  Ego Initial State (x, y, theta, v): w8=(%.2f, %.2f, %.2f, %.2f) vs w9=(%.2f, %.2f, %.2f, %.2f)\n', ...
        w8.ego.x, w8.ego.y, w8.ego.theta, w8.ego.v, w9.ego.x, w9.ego.y, w9.ego.theta, w9.ego.v);
    fprintf('  Number of Dynamic Agents        : w8=%d vs w9=%d\n', w8.n_agents, w9.n_agents);
    fprintf('  Number of Static Obstacles      : w8=%d vs w9=%d\n', w8.n_static_obs, w9.n_static_obs);
    
    % Check bit-exact physical equality
    ego_diff = norm([w8.ego.x - w9.ego.x, w8.ego.y - w9.ego.y, w8.ego.theta - w9.ego.theta, w8.ego.v - w9.ego.v]);
    assert(ego_diff == 0, 'Ego initial states must be bit-exact identical');
    
    agents_identical = true;
    for i = 1:w8.n_agents
        a8 = w8.agents(i); a9 = w9.agents(i);
        if a8.x ~= a9.x || a8.y ~= a9.y || a8.vx ~= a9.vx || a8.vy ~= a9.vy
            agents_identical = false;
        end
    end
    assert(agents_identical, 'Dynamic agents must be bit-exact identical');
    
    obs_identical = true;
    for j = 1:w8.n_static_obs
        o8 = w8.static_obs(j, :); o9 = w9.static_obs(j, :);
        if any(o8 ~= o9)
            obs_identical = false;
        end
    end
    assert(obs_identical, 'Static obstacles must be bit-exact identical');
    
    fprintf('  => PHYSICAL WORLD DEFINITION IDENTITY: YES (BIT-EXACT IDENTICAL)\n');
    
    %% 4. Verify Benchmark Harness Scenario Resolution
    fprintf('\n--- BENCHMARK HARNESS SCENARIO RESOLUTION CHECK ---\n');
    harness_l8_key = ScenarioLadder.SCENARIO_KEYS{8};
    harness_l9_key = ScenarioLadder.SCENARIO_KEYS{9};
    fprintf('  Benchmark Harness Level 8 Scenario Key: %s\n', harness_l8_key);
    fprintf('  Benchmark Harness Level 9 Scenario Key: %s\n', harness_l9_key);
    assert(strcmp(harness_l8_key, harness_l9_key), 'Harness resolves Level 8 and Level 9 to identical scenario key');
    
    %% 5. Compare Deterministic Trajectories
    fprintf('\n--- CLOSED-LOOP TRAJECTORY COMPARISON ---\n');
    
    % Test Mode: ideal, Seed 42
    [~, met8_ideal, hist8_ideal] = stage5_multivehicle_coordination('scenario', l8_key, 'uncertainty_mode', 'ideal', 'seed', 42, 'verbose', false);
    [~, met9_ideal, hist9_ideal] = stage5_multivehicle_coordination('scenario', l9_key, 'uncertainty_mode', 'ideal', 'seed', 42, 'verbose', false);
    
    traj_diff_ideal = max(abs(hist8_ideal.ego_x - hist9_ideal.ego_x)) + max(abs(hist8_ideal.ego_y - hist9_ideal.ego_y));
    fprintf('  Ideal Mode (Seed 42) Trajectory Max Diff            : %.12f m\n', traj_diff_ideal);
    assert(traj_diff_ideal < 1e-12, 'Ideal trajectories must be bit-exact identical');
    
    % Test Mode: combined_realistic, Seed 42
    [~, met8_comb, hist8_comb] = stage5_multivehicle_coordination('scenario', l8_key, 'uncertainty_mode', 'combined_realistic', 'seed', 42, 'verbose', false);
    [~, met9_comb, hist9_comb] = stage5_multivehicle_coordination('scenario', l9_key, 'uncertainty_mode', 'combined_realistic', 'seed', 42, 'verbose', false);
    
    traj_diff_comb = max(abs(hist8_comb.ego_x - hist9_comb.ego_x)) + max(abs(hist8_comb.ego_y - hist9_comb.ego_y));
    fprintf('  Combined Realistic Mode (Seed 42) Trajectory Max Diff: %.12f m\n', traj_diff_comb);
    assert(traj_diff_comb < 1e-12, 'Combined realistic trajectories must be bit-exact identical');
    
    fprintf('\n========================================================================================\n');
    fprintf('  INTEGRITY AUDIT CONCLUSION:                                                           \n');
    fprintf('  1. Level 8 and Level 9 are physically bit-exact identical scenario definitions.       \n');
    fprintf('  2. Intended semantics per ScenarioLadder.m: Level 9 is Level 8 under disturbance noise. \n');
    fprintf('  3. RESOLUTION: RESOLVED BY DOCUMENTATION (No physical environment modification).       \n');
    fprintf('  RESULT: PASS                                                                          \n');
    fprintf('========================================================================================\n\n');
end
