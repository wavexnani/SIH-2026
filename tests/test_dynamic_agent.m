function test_dynamic_agent()
    % TEST_DYNAMIC_AGENT Comprehensive Unit Test Suite for DynamicAgent Class
    %
    % Verifies:
    %   Test A — Constant-speed behavior & convergence
    %   Test B — Acceleration bounds [-4.0, +3.0] m/s^2
    %   Test C — Following behavior & reaction delay response
    %   Test D — Minimum spacing preservation
    %   Test E — Steering stability & lateral error convergence
    %   Test F — Deterministic reproducibility across identical runs
    
    addpath('environment', 'core', 'vehicle', 'planning', 'config');
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 12A UNIT TEST SUITE: DYNAMIC AGENT CLOSED-LOOP BEHAVIOR                    \n');
    fprintf('========================================================================================\n\n');
    
    all_passed = true;
    
    % -------------------------------------------------------------------------
    % Test A: Constant-speed behavior
    % -------------------------------------------------------------------------
    fprintf('Running Test A: Constant-speed behavior... ');
    agentA = DynamicAgent(1, 0.0, 3.0, 2.0, 0.0, 8.0, 3.0); % v_init = 2 m/s, v_target = 8 m/s
    dt = 0.1;
    for k = 1:100
        agentA.step(dt);
    end
    passA = abs(agentA.v - 8.0) < 0.15;
    if passA
        fprintf('PASS (v_final = %.2f m/s, target = 8.0 m/s)\n', agentA.v);
    else
        fprintf('FAIL (v_final = %.2f m/s, target = 8.0 m/s)\n', agentA.v);
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test B: Acceleration bound enforcement
    % -------------------------------------------------------------------------
    fprintf('Running Test B: Acceleration bounds [-4.0, +3.0] m/s^2... ');
    agentB1 = DynamicAgent(1, 0.0, 3.0, 0.0, 0.0, 20.0, 3.0); % Full acceleration demand
    agentB2 = DynamicAgent(2, 0.0, 3.0, 15.0, 0.0, 0.0, 3.0);  % Full deceleration demand
    min_a = inf; max_a = -inf;
    for k = 1:50
        agentB1.step(dt);
        agentB2.step(dt, 10.0, 0.0); % Lead close ahead -> brake
        min_a = min([min_a, agentB1.a, agentB2.a]);
        max_a = max([max_a, agentB1.a, agentB2.a]);
    end
    passB = (min_a >= -4.0 - 1e-6) && (max_a <= 3.0 + 1e-6);
    if passB
        fprintf('PASS (min_a = %.2f m/s^2, max_a = %.2f m/s^2)\n', min_a, max_a);
    else
        fprintf('FAIL (min_a = %.2f m/s^2, max_a = %.2f m/s^2)\n', min_a, max_a);
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test C: Following behavior & reaction delay response
    % -------------------------------------------------------------------------
    fprintf('Running Test C: Following behavior & reaction delay... ');
    lead_x = 50.0; lead_v = 8.0;
    follower = DynamicAgent(1, 20.0, 3.0, 8.0, 0.0, 10.0, 3.0);
    follower.tau_react = 0.3; % 3 steps delay at dt=0.1
    
    % Lead suddenly brakes to 2 m/s at step 10
    a_history = zeros(40, 1);
    for k = 1:40
        if k >= 10, lead_v = 2.0; end
        lead_x = lead_x + lead_v * dt;
        follower.step(dt, lead_x, lead_v);
        a_history(k) = follower.a;
    end
    % Verify deceleration starts after step 10 + reaction delay steps (step 12-13)
    reaction_started = (a_history(11) > -0.1) && (a_history(14) < -0.5);
    passC = reaction_started && (follower.v < 8.0);
    if passC
        fprintf('PASS (Delayed deceleration confirmed at t=%.1fs)\n', 13*dt);
    else
        fprintf('FAIL (Reaction delay behavior inconsistent)\n');
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test D: Minimum spacing preservation
    % -------------------------------------------------------------------------
    fprintf('Running Test D: Minimum spacing preservation... ');
    lead_x = 35.0; lead_v = 0.0; % Lead vehicle stopped ahead
    follower = DynamicAgent(1, 10.0, 3.0, 10.0, 0.0, 10.0, 3.0);
    min_net_gap = inf;
    for k = 1:100
        lead_x = lead_x + lead_v * dt;
        follower.step(dt, lead_x, lead_v);
        net_gap = lead_x - follower.x - follower.length;
        min_net_gap = min(min_net_gap, net_gap);
    end
    passD = (min_net_gap > 0.5); % Positive gap maintained
    if passD
        fprintf('PASS (Min net gap = %.2f m > 0.5m)\n', min_net_gap);
    else
        fprintf('FAIL (Overlap detected: min net gap = %.2f m)\n', min_net_gap);
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test E: Steering stability & lateral convergence
    % -------------------------------------------------------------------------
    fprintf('Running Test E: Steering stability... ');
    agentE = DynamicAgent(1, 0.0, 4.5, 6.0, 0.0, 6.0, 3.0); % Offset y = 4.5m vs target y = 3.0m
    max_overshoot = 0.0;
    for k = 1:120
        agentE.step(dt);
        if agentE.y < 3.0
            max_overshoot = max(max_overshoot, 3.0 - agentE.y);
        end
    end
    passE = (abs(agentE.y - 3.0) < 0.10) && (max_overshoot < 0.30);
    if passE
        fprintf('PASS (Final y = %.3f m, max overshoot = %.3f m)\n', agentE.y, max_overshoot);
    else
        fprintf('FAIL (Final y = %.3f m, max overshoot = %.3f m)\n', agentE.y, max_overshoot);
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test F: Determinism check
    % -------------------------------------------------------------------------
    fprintf('Running Test F: Deterministic reproducibility... ');
    run1_agent = DynamicAgent(1, 5.0, 2.8, 7.0, 0.05, 8.0, 3.0);
    run2_agent = DynamicAgent(1, 5.0, 2.8, 7.0, 0.05, 8.0, 3.0);
    lead_x = 40.0; lead_v = 4.0;
    for k = 1:80
        lead_x = lead_x + lead_v * dt;
        run1_agent.step(dt, lead_x, lead_v);
        run2_agent.step(dt, lead_x, lead_v);
    end
    diff_vec = abs(run1_agent.getStateVec() - run2_agent.getStateVec());
    passF = max(diff_vec) < 1e-12;
    if passF
        fprintf('PASS (Max state diff = %.2e)\n', max(diff_vec));
    else
        fprintf('FAIL (Max state diff = %.2e)\n', max(diff_vec));
        all_passed = false;
    end
    
    fprintf('========================================================================================\n');
    if all_passed
        fprintf('  ALL DYNAMIC AGENT UNIT TESTS PASSED SUCCESSFULLY!\n');
    else
        fprintf('  SOME DYNAMIC AGENT UNIT TESTS FAILED!\n');
        error('test_dynamic_agent failed.');
    end
    fprintf('========================================================================================\n\n');
end
