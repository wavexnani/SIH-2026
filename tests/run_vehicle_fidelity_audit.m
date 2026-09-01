function run_vehicle_fidelity_audit()
    % RUN_VEHICLE_FIDELITY_AUDIT Forensic Audit of Vehicle Dynamics & Motion Instability
    %
    % Purpose:
    %   Measures exact quantitative motion metrics for Demos 1, 2, 4, 5, and 6
    %   WITHOUT modifying any planner, controller, vehicle, or visualization code.
    
    addpath('planning', 'config', 'vehicle', 'core', 'environment', 'stages', 'tests', 'visualization');
    
    cfg = SimulationConfig();
    
    demos = {
        'Demo 1 — Nominal', 'nominal_straight';
        'Demo 2 — Static Obstacle', 'gradual_narrowing';
        'Demo 4 — Dynamic Road Narrowing', 'tight_corridor';
        'Demo 6 — Total Blockage', 'infeasible_blocked'
    };
    
    fprintf('\n========================================================================================\n');
    fprintf('        FORENSIC AUDIT: QUANTITATIVE VEHICLE MOTION INSTABILITY METRICS                \n');
    fprintf('========================================================================================\n\n');
    
    for d = 1:size(demos, 1)
        name_str = demos{d, 1};
        scen_id = demos{d, 2};
        
        [world, map_obj] = ScenarioGenerator.createScenario(scen_id, cfg);
        res = audit_scenario_motion(name_str, world, map_obj, cfg);
        print_audit_report(res);
    end
    
    % Also audit Demo 5 (Stage 5 Multi-Vehicle Oncoming Conflict)
    fprintf('----------------------------------------------------------------------------------------\n');
    fprintf('  Auditing: Demo 5 — Multi-Vehicle Oncoming Conflict\n');
    fprintf('----------------------------------------------------------------------------------------\n');
    [~, ~, h5] = stage5_multivehicle_coordination('scenario', 'multi_vehicle_oncoming_conflict', 'verbose', false);
    res5 = audit_trajectory_history('Demo 5 — Multi-Vehicle Oncoming Conflict', h5.t, h5.ego_x, h5.ego_y, h5.ego_v, h5.ego_theta, zeros(size(h5.ego_theta)));
    print_audit_report(res5);
end

function res = audit_scenario_motion(name_str, world, map_obj, cfg)
    vehicle = BicycleModel(cfg);
    ctrl = Stage5CoordinationController(cfg);
    ctrl.reset();
    ctrl.cacrc_planner.bound_provider = FreeSpaceBoundProvider(map_obj, 1.05);
    ctrl.setOvertakeEnabled(true);
    
    N_steps = 150; dt = cfg.dt;
    N_path = 500; ref_path = zeros(N_path, 5);
    ref_path(:, 1) = linspace(0, 150, N_path)';
    for r = 1:N_path
        [y_min_r, y_max_r] = map_obj.getRoadBoundsAt(ref_path(r, 1));
        ref_path(r, 2) = 0.5 * (y_min_r + y_max_r);
    end
    for r = 1:N_path
        r1 = max(1, r - 1); r2 = min(N_path, r + 1);
        ref_path(r, 3) = atan2(ref_path(r2, 2) - ref_path(r1, 2), ref_path(r2, 1) - ref_path(r1, 1));
    end
    ref_path(:, 4) = 0.0; ref_path(:, 5) = 8.0;
    
    t_hist = zeros(N_steps, 1);
    x_hist = zeros(N_steps, 1);
    y_hist = zeros(N_steps, 1);
    v_hist = zeros(N_steps, 1);
    theta_hist = zeros(N_steps, 1);
    delta_hist = zeros(N_steps, 1);
    a_cmd_hist = zeros(N_steps, 1);
    
    for k = 1:N_steps
        t_hist(k) = (k - 1) * dt;
        x_hist(k) = world.ego.x;
        y_hist(k) = world.ego.y;
        v_hist(k) = world.ego.v;
        theta_hist(k) = world.ego.theta;
        delta_hist(k) = world.ego.delta;
        
        [u_cmd, ~, ~, ~] = ctrl.step(world, ref_path, 8.0);
        a_cmd_hist(k) = u_cmd(2);
        world.ego = vehicle.stepKinematic(world.ego, u_cmd(2), u_cmd(1), dt);
    end
    
    res = audit_trajectory_history(name_str, t_hist, x_hist, y_hist, v_hist, theta_hist, delta_hist);
end

function res = audit_trajectory_history(name_str, t, x, y, v, theta, delta)
    dt = mean(diff(t));
    N = length(t);
    
    dtheta = diff(theta);
    % Unwrap heading diffs
    dtheta = atan2(sin(dtheta), cos(dtheta));
    yaw_rate = dtheta / dt;
    
    ddelta = diff(delta);
    delta_rate = ddelta / dt;
    
    dv = diff(v);
    accel = dv / dt;
    
    dy = diff(y);
    vy = dy / dt;
    
    lat_accel = v(1:N-1) .* yaw_rate;
    
    da = diff(accel);
    jerk = da / dt;
    
    res.name = name_str;
    res.max_yaw_rate_deg = max(abs(yaw_rate)) * (180 / pi);
    res.max_dtheta_deg_per_step = max(abs(dtheta)) * (180 / pi);
    res.max_delta_rate_deg = max(abs(delta_rate)) * (180 / pi);
    res.max_accel = max(accel);
    res.max_decel = min(accel);
    res.max_vy = max(abs(vy));
    res.max_lat_accel = max(abs(lat_accel));
    res.max_jerk = max(abs(jerk));
end

function print_audit_report(res)
    fprintf('Scenario: %s\n', res.name);
    fprintf('  - Max Heading Change per Step (|dTheta|):  %.2f deg/step (%.2f deg/s yaw rate)\n', ...
        res.max_dtheta_deg_per_step, res.max_yaw_rate_deg);
    fprintf('  - Max Steering Rate (|dDelta/dt|):        %.2f deg/s\n', res.max_delta_rate_deg);
    fprintf('  - Max Accel / Decel:                     %+6.2f m/s^2  /  %+6.2f m/s^2\n', ...
        res.max_accel, res.max_decel);
    fprintf('  - Max Lateral Velocity (|dy/dt|):          %.2f m/s\n', res.max_vy);
    fprintf('  - Max Lateral Acceleration (v * yaw_rate): %.2f m/s^2 (%.2f g)\n', ...
        res.max_lat_accel, res.max_lat_accel / 9.81);
    fprintf('  - Max Longitudinal Jerk (|da/dt|):         %.2f m/s^3\n\n', res.max_jerk);
end
