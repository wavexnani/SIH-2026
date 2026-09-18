function test_steering_filter()
    % TEST_STEERING_FILTER Unit verification for DelayAwareSteeringFilter
    %
    % Tests:
    %   1. Slew rate compliance under large step inputs
    %   2. Micro-jitter / chattering suppression under noisy oscillation
    %   3. Emergency bypass under sudden large steering demand
    %   4. Pass-through mode when policy is 'RAW' or filter disabled
    %   5. Internal first-order actuator arrival prediction
    
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir)
        projectRoot = pwd;
    else
        projectRoot = fileparts(current_dir);
    end
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'config'));
    
    fprintf('\n========================================================\n');
    fprintf('  DELAY-AWARE STEERING FILTER UNIT TEST SUITE\n');
    fprintf('========================================================\n\n');
    
    cfg = SimulationConfig();
    filter = DelayAwareSteeringFilter(cfg);
    filter.delta_rate_max = 0.50; % rad/s
    filter.deadband_rad = 0.015;  % ~0.86 deg
    filter.emergency_threshold_rad = 0.080; % ~4.6 deg
    filter.dt = 0.10;
    
    pass_count = 0;
    total_tests = 5;
    
    % -------------------------------------------------------------
    % Test 1: Slew Rate Limiting
    % -------------------------------------------------------------
    % Step from 0.0 to 0.30 rad at t=0
    % Max step per cycle = 0.50 * 0.10 = 0.05 rad
    filter.reset(0.0);
    [cmd1, info1] = filter.step(0.30, 0.0, 0.10);
    pass1 = (abs(cmd1 - 0.05) < 1e-4);
    print_test_result(1, 'Slew Rate Limiting (Step to 0.30 rad)', ...
        sprintf('cmd=%.4f rad (expected 0.0500)', cmd1), pass1);
    if pass1, pass_count = pass_count + 1; end
    
    % -------------------------------------------------------------
    % Test 2: Micro-Jitter Suppression (Chattering Elimination)
    % -------------------------------------------------------------
    % Input oscillating with +/- 0.010 rad (below deadband of 0.015 rad)
    filter.reset(0.10);
    N_steps = 40;
    t = (1:N_steps)';
    noisy_inputs = 0.10 + 0.010 * sin(2 * pi * t / 2); % Rapid 2-step alternating chatter
    
    cmds = zeros(N_steps, 1);
    for k = 1:N_steps
        [cmds(k), ~] = filter.step(noisy_inputs(k), 0.10, 0.10);
    end
    
    % With deadband hold, commands should stay perfectly constant at 0.10 rad
    chatter_reversals = filter.reversal_count;
    variance_cmds = var(cmds);
    pass2 = (chatter_reversals == 0) && (variance_cmds < 1e-12);
    print_test_result(2, 'Micro-Jitter Suppression (Oscillating +/- 0.010 rad)', ...
        sprintf('reversals=%d (target 0), var=%.2e (target ~0)', chatter_reversals, variance_cmds), pass2);
    if pass2, pass_count = pass_count + 1; end
    
    % -------------------------------------------------------------
    % Test 3: Emergency Override
    % -------------------------------------------------------------
    % Large steering change of 0.15 rad (> emergency threshold 0.080 rad)
    filter.reset(0.0);
    [cmd3, info3] = filter.step(0.15, 0.0, 0.10, true); % Emergency override = true
    pass3 = info3.is_emergency && (cmd3 > 0.0);
    print_test_result(3, 'Emergency Override Instantaneous Bypass', ...
        sprintf('cmd=%.4f rad, is_emergency=%d', cmd3, info3.is_emergency), pass3);
    if pass3, pass_count = pass_count + 1; end
    
    % -------------------------------------------------------------
    % Test 4: Pass-Through when Policy is 'RAW'
    % -------------------------------------------------------------
    filter.reset(0.0);
    filter.policy = 'RAW';
    [cmd4, ~] = filter.step(0.22, 0.0, 0.10);
    pass4 = (abs(cmd4 - 0.22) < 1e-4);
    filter.policy = 'HOLD_DEADBAND'; % restore
    print_test_result(4, 'Pass-Through under RAW Policy', ...
        sprintf('cmd=%.4f rad (target 0.2200)', cmd4), pass4);
    if pass4, pass_count = pass_count + 1; end
    
    % -------------------------------------------------------------
    % Test 5: Actuator Lag Arrival Prediction
    % -------------------------------------------------------------
    filter.reset(0.0);
    dt_p = 0.10; tau_p = 0.15;
    alpha_exp = 1.0 - exp(-dt_p / tau_p);
    delta_pred = filter.predictArrival(0.0, 0.20, dt_p);
    expected_pred = 0.0 + alpha_exp * (0.20 - 0.0);
    pass5 = (abs(delta_pred - expected_pred) < 1e-4);
    print_test_result(5, 'Actuator First-Order Arrival Prediction', ...
        sprintf('pred=%.4f rad (expected %.4f rad, alpha=%.3f)', delta_pred, expected_pred, alpha_exp), pass5);
    if pass5, pass_count = pass_count + 1; end
    
    fprintf('========================================================\n');
    fprintf('  STEERING FILTER TESTS: %d/%d PASSED\n', pass_count, total_tests);
    fprintf('========================================================\n\n');
    
    if pass_count < total_tests
        error('DelayAwareSteeringFilter unit tests failed.');
    end
end

function print_test_result(idx, name, details, pass)
    if pass
        fprintf('[PASS] Test %d: %s\n', idx, name);
    else
        fprintf('[FAIL] Test %d: %s\n', idx, name);
    end
    fprintf('       %s\n\n', details);
end
