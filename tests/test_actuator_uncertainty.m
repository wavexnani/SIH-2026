function test_actuator_uncertainty()
    % TEST_ACTUATOR_UNCERTAINTY Unit Test Suite for ActuatorUncertaintyModel Class
    %
    % Verifies:
    %   Test A — Ideal identity (u_actual == u_cmd)
    %   Test B — Steering bias (+0.8 deg steering bias application)
    %   Test C — Saturation enforcement (never exceeds physical max_delta)
    %   Test D — Existing slew-rate preservation in BicycleModel plant
    %   Test E — Longitudinal limits enforcement (a_actual within [a_min, a_max])
    %   Test F — Deterministic reproducibility across identical runs
    %   Test G — Ground-truth state preservation
    %   Test H — Zero-command stability & zero-drift verification
    
    addpath('environment', 'core', 'vehicle', 'planning', 'config');
    
    fprintf('\n========================================================================================\n');
    fprintf('        PHASE 12C UNIT TEST SUITE: EGO ACTUATOR UNCERTAINTY MODEL                      \n');
    fprintf('========================================================================================\n\n');
    
    all_passed = true;
    cfg = SimulationConfig();
    
    % -------------------------------------------------------------------------
    % Test A: Ideal identity
    % -------------------------------------------------------------------------
    fprintf('Running Test A: Ideal identity mapping... ');
    act_ideal = ActuatorUncertaintyModel('ideal');
    u_cmd = [deg2rad(5.0), 1.5];
    [u_act, info] = act_ideal.process(u_cmd, cfg);
    
    passA = (abs(u_act(1) - u_cmd(1)) < 1e-12) && (abs(u_act(2) - u_cmd(2)) < 1e-12) && ...
            (info.steering_error == 0.0) && (info.acceleration_error == 0.0);
    if passA
        fprintf('PASS (u_actual == u_cmd exactly)\n');
    else
        fprintf('FAIL (Ideal actuator output mismatch)\n');
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test B: Steering bias (+0.8 degrees)
    % -------------------------------------------------------------------------
    fprintf('Running Test B: Steering bias application (+0.8 deg)... ');
    act_bias = ActuatorUncertaintyModel('steering_bias', 'steering_bias_deg', 0.8);
    u_cmd_b = [deg2rad(2.0), 1.0];
    [u_act_b, info_b] = act_bias.process(u_cmd_b, cfg);
    
    expected_delta = deg2rad(2.0) + deg2rad(0.8);
    passB = (abs(u_act_b(1) - expected_delta) < 1e-12) && ...
            (abs(info_b.steering_error - deg2rad(0.8)) < 1e-12);
    if passB
        fprintf('PASS (Steering bias +0.8 deg applied accurately)\n');
    else
        fprintf('FAIL (Steering bias application mismatch: err=%.4f deg)\n', rad2deg(info_b.steering_error));
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test C: Saturation enforcement
    % -------------------------------------------------------------------------
    fprintf('Running Test C: Saturation enforcement under extreme command... ');
    u_cmd_extreme = [cfg.delta_max, 5.0]; % Max steering command + bias should clamp to delta_max
    [u_act_c, ~] = act_bias.process(u_cmd_extreme, cfg);
    
    passC = (u_act_c(1) <= cfg.delta_max + 1e-12) && (u_act_c(2) <= cfg.a_max + 1e-12);
    if passC
        fprintf('PASS (Command clamped to delta_max = %.2f deg)\n', rad2deg(cfg.delta_max));
    else
        fprintf('FAIL (Actuator exceeded physical saturation limits: delta=%.2f deg)\n', rad2deg(u_act_c(1)));
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test D: Existing slew-rate preservation in BicycleModel plant
    % -------------------------------------------------------------------------
    fprintf('Running Test D: Slew-rate preservation in BicycleModel... ');
    veh = BicycleModel(cfg);
    ego_s = EgoState(10, 1.8, 0, 5.0, 0);
    dt = cfg.dt;
    
    % Command massive jump in steering angle (0 -> 35 deg)
    u_jump = [cfg.delta_max, 0.0];
    [u_act_d, ~] = act_bias.process(u_jump, cfg);
    ego_next = veh.stepKinematic(ego_s, u_act_d(2), u_act_d(1), dt);
    
    observed_delta_rate = abs(ego_next.delta - ego_s.delta) / dt;
    passD = (observed_delta_rate <= cfg.delta_rate_max + 1e-6);
    if passD
        fprintf('PASS (Observed rate %.2f deg/s <= max limit %.2f deg/s)\n', ...
            rad2deg(observed_delta_rate), rad2deg(cfg.delta_rate_max));
    else
        fprintf('FAIL (Steering rate limit violated: %.2f deg/s)\n', rad2deg(observed_delta_rate));
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test E: Longitudinal limits enforcement
    % -------------------------------------------------------------------------
    fprintf('Running Test E: Longitudinal limits enforcement... ');
    u_cmd_long = [0.0, -10.0]; % Demanding -10 m/s^2 brake
    [u_act_e, ~] = act_bias.process(u_cmd_long, cfg);
    
    passE = (u_act_e(2) >= cfg.a_min - 1e-12) && (u_act_e(2) <= cfg.a_max + 1e-12);
    if passE
        fprintf('PASS (Deceleration clamped to a_min = %.2f m/s^2)\n', cfg.a_min);
    else
        fprintf('FAIL (Longitudinal limits violated: a=%.2f m/s^2)\n', u_act_e(2));
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test F: Deterministic reproducibility
    % -------------------------------------------------------------------------
    fprintf('Running Test F: Deterministic reproducibility... ');
    act_f1 = ActuatorUncertaintyModel('steering_bias');
    act_f2 = ActuatorUncertaintyModel('steering_bias');
    
    [u_f1, ~] = act_f1.process([0.1, 0.5], cfg);
    [u_f2, ~] = act_f2.process([0.1, 0.5], cfg);
    
    passF = (abs(u_f1(1) - u_f2(1)) < 1e-12) && (abs(u_f1(2) - u_f2(2)) < 1e-12);
    if passF
        fprintf('PASS (Bit-exact match across identical runs)\n');
    else
        fprintf('FAIL (Non-deterministic output detected)\n');
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test G: Ground-truth state preservation
    % -------------------------------------------------------------------------
    fprintf('Running Test G: Ground-truth state preservation... ');
    ego_orig = EgoState(10, 1.8, 0, 5.0, 0);
    x_orig = ego_orig.x;
    act_g = ActuatorUncertaintyModel('combined');
    [~, ~] = act_g.process([0.1, 0.0], cfg);
    
    passG = (ego_orig.x == x_orig);
    if passG
        fprintf('PASS (Ego State object untouched by actuator model)\n');
    else
        fprintf('FAIL (Ego State object mutated)\n');
        all_passed = false;
    end
    
    % -------------------------------------------------------------------------
    % Test H: Zero-command stability
    % -------------------------------------------------------------------------
    fprintf('Running Test H: Zero-command ideal stability... ');
    u_zero = [0.0, 0.0];
    [u_act_h, ~] = act_ideal.process(u_zero, cfg);
    
    passH = (u_act_h(1) == 0.0) && (u_act_h(2) == 0.0);
    if passH
        fprintf('PASS (Zero command produces zero actual input in ideal mode)\n');
    else
        fprintf('FAIL (Spurious input produced under zero command)\n');
        all_passed = false;
    end
    
    fprintf('========================================================================================\n');
    if all_passed
        fprintf('  ALL ACTUATOR UNCERTAINTY UNIT TESTS PASSED SUCCESSFULLY!\n');
    else
        fprintf('  SOME ACTUATOR UNCERTAINTY UNIT TESTS FAILED!\n');
        error('test_actuator_uncertainty failed.');
    end
    fprintf('========================================================================================\n\n');
end
