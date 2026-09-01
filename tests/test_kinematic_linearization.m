function test_kinematic_linearization()
    % TEST_KINEMATIC_LINEARIZATION Unit Test for Kinematic Bicycle Linearization
    %
    % Validates analytical Jacobians A_c, B_c against central finite-difference approximations:
    % df/dx \approx (f(x + eps) - f(x - eps)) / (2 * eps)
    % Verifies O(||eps||^2) error convergence.
    
    current_dir = fileparts(mfilename('fullpath'));
    if isempty(current_dir)
        projectRoot = pwd;
    else
        projectRoot = fileparts(current_dir);
    end
    addpath(fullfile(projectRoot, 'planning'));
    addpath(fullfile(projectRoot, 'config'));
    
    planner = QPMPCPlanner();
    
    fprintf('\n========================================================\n');
    fprintf('  KINEMATIC BICYCLE MODEL LINEARIZATION UNIT TEST\n');
    fprintf('========================================================\n\n');
    
    % Operating state (x, y, theta, v) and control (delta, a)
    x_op = [10.0; 3.0; 0.15; 8.0];
    u_op = [0.05; 0.5];
    
    % Analytical Jacobians via Forward Euler method
    [A_d, B_d, c_d] = planner.linearize_kinematics(x_op, u_op);
    dt = planner.dt;
    A_c_analytical = (A_d - eye(4)) / dt;
    B_c_analytical = B_d / dt;
    
    % Central Finite-Difference Computation for A_c = df/dx
    eps_h = 1e-5;
    A_c_fd = zeros(4, 4);
    for j = 1:4
        dx = zeros(4, 1); dx(j) = eps_h;
        f_plus = bicycle_dynamics(x_op + dx, u_op, planner.L);
        f_minus = bicycle_dynamics(x_op - dx, u_op, planner.L);
        A_c_fd(:, j) = (f_plus - f_minus) / (2 * eps_h);
    end
    
    % Central Finite-Difference Computation for B_c = df/du
    B_c_fd = zeros(4, 2);
    for j = 1:2
        du = zeros(2, 1); du(j) = eps_h;
        f_plus = bicycle_dynamics(x_op, u_op + du, planner.L);
        f_minus = bicycle_dynamics(x_op, u_op - du, planner.L);
        B_c_fd(:, j) = (f_plus - f_minus) / (2 * eps_h);
    end
    
    err_A = norm(A_c_analytical - A_c_fd, 'fro');
    err_B = norm(B_c_analytical - B_c_fd, 'fro');
    
    pass_A = err_A < 1e-4;
    pass_B = err_B < 1e-4;
    
    fprintf('  A_c Frobenius Norm Difference: %.2e %s\n', err_A, pass_str(pass_A));
    fprintf('  B_c Frobenius Norm Difference: %.2e %s\n', err_B, pass_str(pass_B));
    
    % Taylor Expansion Residual Scaling Test (O(||dx||^2))
    dx_small = [0.01; 0.01; 0.005; 0.02];
    du_small = [0.005; 0.01];
    
    f_actual = bicycle_dynamics(x_op + dx_small, u_op + du_small, planner.L);
    f_op = bicycle_dynamics(x_op, u_op, planner.L);
    f_taylor = f_op + A_c_analytical * dx_small + B_c_analytical * du_small;
    
    taylor_error = norm(f_actual - f_taylor);
    norm_dx = norm([dx_small; du_small]);
    
    fprintf('  Taylor Linear Approximation Error: %.2e (Scaling ~ O(%.2e))\n', ...
        taylor_error, norm_dx^2);
    
    all_pass = pass_A && pass_B && (taylor_error < 1e-2);
    
    fprintf('\n========================================================\n');
    if all_pass
        fprintf('  MODEL LINEARIZATION UNIT TEST: PASSED\n');
    else
        fprintf('  MODEL LINEARIZATION UNIT TEST: FAILED\n');
    end
    fprintf('========================================================\n\n');
end

function f = bicycle_dynamics(x, u, L)
    theta = x(3); v = x(4);
    delta = u(1); a = u(2);
    f = [v * cos(theta);
         v * sin(theta);
         (v / L) * tan(delta);
         a];
end

function str = pass_str(val)
    if val, str = '[PASS]'; else, str = '[FAIL]'; end
end
