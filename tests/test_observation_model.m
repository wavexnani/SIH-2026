function test_observation_model()
    % TEST_OBSERVATION_MODEL Unit Test Suite for ObservationModel Class
    %
    % Verifies:
    %   Test A — Ideal identity (truth == observation)
    %   Test B — Deterministic noise reproducibility (same seed -> identical obs)
    %   Test C — Different seeds yield distinct noise realizations
    %   Test D — Position noise statistics (~0.15m sigma)
    %   Test E — Velocity noise statistics (~0.20m/s sigma)
    %   Test F — Heading noise & wrapping (~0.02rad sigma)
    %   Test G — Deterministic 1-step perception delay (obs(k) == truth(k-1))
    %   Test H — Ground truth preservation (truth object is untouched)
    
    addpath('environment', 'core', 'vehicle', 'planning', 'config');
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 12B UNIT TEST SUITE: OBSERVATION MODEL PERCEPTION PIPELINE               \n');
    fprintf('========================================================================================\n\n');
    
    all_passed = true;
    
    % Setup reference test world state
    cfg = SimulationConfig();
    world = WorldState(cfg);
    world = world.setAgentState(1, 30.0, 1.80, 5.0, 0.0, 0.20);
    dt = 0.1;
    
    % -------------------------------------------------------------------------
    % Test A: Ideal identity
    % -------------------------------------------------------------------------
    fprintf('Running Test A: Ideal identity mapping... ');
    obs_ideal = ObservationModel('ideal');
    [world_obs, structs] = obs_ideal.observe(world, dt);
    
    passA = (abs(world_obs.agents(1).x - world.agents(1).x) < 1e-12) && ...
            (abs(world_obs.agents(1).y - world.agents(1).y) < 1e-12) && ...
            (abs(world_obs.agents(1).vx - world.agents(1).vx) < 1e-12) && ...
            (structs(1).age == 0.0);
    if passA
        fprintf('PASS (Ground truth == Observation exactly)\n');
    else
        fprintf('FAIL (Ideal observation mismatch)\n');
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test B: Deterministic noise reproducibility (Same seed)
    % -------------------------------------------------------------------------
    fprintf('Running Test B: Deterministic noise reproducibility... ');
    obs_b1 = ObservationModel('nominal', 42);
    obs_b2 = ObservationModel('nominal', 42);
    
    [w_b1, s_b1] = obs_b1.observe(world, dt);
    [w_b2, s_b2] = obs_b2.observe(world, dt);
    
    diff_b = abs(s_b1(1).x - s_b2(1).x) + abs(s_b1(1).y - s_b2(1).y) + ...
             abs(s_b1(1).velocity - s_b2(1).velocity) + abs(s_b1(1).heading - s_b2(1).heading);
    passB = (diff_b < 1e-12);
    if passB
        fprintf('PASS (Identical seeds produced bit-exact noise, diff=%.2e)\n', diff_b);
    else
        fprintf('FAIL (Deterministic seed failure, diff=%.2e)\n', diff_b);
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test C: Different seeds yield distinct noise
    % -------------------------------------------------------------------------
    fprintf('Running Test C: Seed variation... ');
    obs_c1 = ObservationModel('nominal', 42);
    obs_c2 = ObservationModel('nominal', 999);
    
    [~, s_c1] = obs_c1.observe(world, dt);
    [~, s_c2] = obs_c2.observe(world, dt);
    
    diff_c = abs(s_c1(1).x - s_c2(1).x) + abs(s_c1(1).velocity - s_c2(1).velocity);
    passC = (diff_c > 1e-4);
    if passC
        fprintf('PASS (Distinct seeds produced distinct realizations, diff=%.4f)\n', diff_c);
    else
        fprintf('FAIL (Seed variation produced identical outputs)\n');
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test D, E, F: Noise statistics verification (Position, Velocity, Heading)
    % -------------------------------------------------------------------------
    fprintf('Running Test D, E, F: Noise statistics bounds... ');
    obs_stat = ObservationModel('nominal', 12345);
    N_samples = 1000;
    err_x = zeros(N_samples, 1);
    err_y = zeros(N_samples, 1);
    err_v = zeros(N_samples, 1);
    err_th = zeros(N_samples, 1);
    
    for k = 1:N_samples
        [~, s_k] = obs_stat.observe(world, dt);
        err_x(k) = s_k(1).x - world.agents(1).x;
        err_y(k) = s_k(1).y - world.agents(1).y;
        err_v(k) = s_k(1).velocity - 5.0;
        err_th(k) = atan2(sin(s_k(1).heading - 0.0), cos(s_k(1).heading - 0.0));
    end
    
    std_pos_x = std(err_x);
    std_pos_y = std(err_y);
    std_v = std(err_v);
    std_th = std(err_th);
    
    passD = abs(std_pos_x - 0.15) < 0.03 && abs(std_pos_y - 0.15) < 0.03;
    passE = abs(std_v - 0.20) < 0.04;
    passF = abs(std_th - 0.02) < 0.005;
    
    if passD && passE && passF
        fprintf('PASS (std_pos=%.3fm, std_vel=%.3fm/s, std_heading=%.4frad)\n', ...
            std_pos_x, std_v, std_th);
    else
        fprintf('FAIL (std_pos=%.3fm, std_vel=%.3fm/s, std_heading=%.4frad)\n', ...
            std_pos_x, std_v, std_th);
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test G: One-step perception delay
    % -------------------------------------------------------------------------
    fprintf('Running Test G: One-step perception delay... ');
    obs_del = ObservationModel('delayed', 42);
    obs_del.tau_delay = 0.10; % 1 step at dt=0.1
    
    world_k0 = world;
    world_k1 = world.setAgentState(1, 35.0, 1.80, 6.0, 0.0, 0.20); % Step k=1 state
    
    [w_obs0, s_obs0] = obs_del.observe(world_k0, dt);
    [w_obs1, s_obs1] = obs_del.observe(world_k1, dt);
    
    % At step 1, observation should match step 0 ground truth
    passG = (abs(w_obs1.agents(1).x - world_k0.agents(1).x) < 1e-12) && ...
            (abs(s_obs1(1).age - 0.10) < 1e-12);
    if passG
        fprintf('PASS (obs(k=1) == truth(k=0), age = %.2f s)\n', s_obs1(1).age);
    else
        fprintf('FAIL (Delay observation mismatch: obs_x=%.2f, true_k0_x=%.2f)\n', ...
            w_obs1.agents(1).x, world_k0.agents(1).x);
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test H: Ground truth preservation
    % -------------------------------------------------------------------------
    fprintf('Running Test H: Ground truth preservation... ');
    true_x_before = world.agents(1).x;
    obs_h = ObservationModel('nominal', 777);
    [~, ~] = obs_h.observe(world, dt);
    true_x_after = world.agents(1).x;
    
    passH = (true_x_before == true_x_after);
    if passH
        fprintf('PASS (True agent state untouched by observation model)\n');
    else
        fprintf('FAIL (Ground truth modified during observation call!)\n');
        all_passed = false;
    end
    
    fprintf('========================================================================================\n');
    if all_passed
        fprintf('  ALL OBSERVATION MODEL UNIT TESTS PASSED SUCCESSFULLY!\n');
    else
        fprintf('  SOME OBSERVATION MODEL UNIT TESTS FAILED!\n');
        error('test_observation_model failed.');
    end
    fprintf('========================================================================================\n\n');
end
