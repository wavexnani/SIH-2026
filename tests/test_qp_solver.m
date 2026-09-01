function test_qp_solver()
    % TEST_QP_SOLVER Standalone Unit Verification for Hildreth Dual QP Solver
    %
    % Tests hildreth_dual_qp against 5 analytically known QP benchmarks.
    
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
    fprintf('  INDEPENDENT QP SOLVER UNIT TEST SUITE\n');
    fprintf('========================================================\n\n');
    
    pass_count = 0;
    
    % --- Test 1: Unconstrained 1D Quadratic ---
    % min 0.5 * x^2 + x  =>  x* = -1.0
    H1 = 1.0; f1 = 1.0;
    A1 = []; b1 = []; lb1 = -inf; ub1 = inf;
    [x1, ~, res1, ok1] = planner.hildreth_dual_qp(H1, f1, A1, b1, lb1, ub1);
    err1 = abs(x1 - (-1.0));
    pass1 = ok1 && (err1 < 1e-3);
    print_qp_test('Test 1: Unconstrained 1D Quadratic (x* = -1.0)', x1, -1.0, err1, res1, pass1);
    if pass1, pass_count = pass_count + 1; end
    
    % --- Test 2: Active Lower Bound ---
    % min 0.5 * x^2 s.t. x >= 1.0  =>  x* = 1.0
    H2 = 1.0; f2 = 0.0;
    A2 = []; b2 = []; lb2 = 1.0; ub2 = inf;
    [x2, ~, res2, ok2] = planner.hildreth_dual_qp(H2, f2, A2, b2, lb2, ub2);
    err2 = abs(x2 - 1.0);
    pass2 = ok2 && (err2 < 1e-3);
    print_qp_test('Test 2: Active Lower Bound (x* = 1.0)', x2, 1.0, err2, res2, pass2);
    if pass2, pass_count = pass_count + 1; end
    
    % --- Test 3: Active Upper Bound ---
    % min 0.5 * (x - 5)^2 s.t. x <= 2.0  =>  x* = 2.0
    H3 = 1.0; f3 = -5.0;
    A3 = []; b3 = []; lb3 = -inf; ub3 = 2.0;
    [x3, ~, res3, ok3] = planner.hildreth_dual_qp(H3, f3, A3, b3, lb3, ub3);
    err3 = abs(x3 - 2.0);
    pass3 = ok3 && (err3 < 1e-3);
    print_qp_test('Test 3: Active Upper Bound (x* = 2.0)', x3, 2.0, err3, res3, pass3);
    if pass3, pass_count = pass_count + 1; end
    
    % --- Test 4: 2D Constrained Linear Inequalities ---
    % min 0.5 * (x1^2 + x2^2) - 2*x1 - 2*x2  s.t. x1 + x2 <= 1, x1>=0, x2>=0
    % Unconstrained opt = (2,2). Constrained opt on x1+x2=1 => x* = (0.5, 0.5)
    H4 = eye(2); f4 = [-2.0; -2.0];
    A4 = [1.0, 1.0]; b4 = 1.0;
    lb4 = [0.0; 0.0]; ub4 = [inf; inf];
    [x4, ~, res4, ok4] = planner.hildreth_dual_qp(H4, f4, A4, b4, lb4, ub4);
    err4 = norm(x4 - [0.5; 0.5]);
    pass4 = ok4 && (err4 < 1e-3);
    print_qp_test('Test 4: 2D Linear Inequality (x* = [0.5, 0.5])', norm(x4), norm([0.5; 0.5]), err4, res4, pass4);
    if pass4, pass_count = pass_count + 1; end
    
    % --- Test 5: MPC-Sized 40-Variable System ---
    n5 = 40;
    H5 = eye(n5);
    f5 = -2.0 * ones(n5, 1); % Unconstrained min = 2.0 for all i
    A5 = eye(n5); b5 = 1.5 * ones(n5, 1); % Upper bound x_i <= 1.5
    lb5 = zeros(n5, 1); ub5 = 3.0 * ones(n5, 1);
    [x5, ~, res5, ok5] = planner.hildreth_dual_qp(H5, f5, A5, b5, lb5, ub5);
    err5 = norm(x5 - 1.5 * ones(n5, 1));
    
    % Check against MATLAB quadprog if available
    if exist('quadprog', 'file') == 2 || exist('quadprog', 'builtin') == 5
        opts = optimoptions('quadprog', 'Display', 'off');
        x_quad = quadprog(H5, f5, A5, b5, [], [], lb5, ub5, [], opts);
        diff_quad = max(abs(x5 - x_quad));
        fprintf('       [ORACLE] quadprog Available: ||x_Hildreth - x_quadprog||_inf = %.1e\n', diff_quad);
    else
        fprintf('       [ORACLE] quadprog Not Available (MATLAB-only independent solution checked)\n');
    end
    
    pass5 = ok5 && (err5 < 1e-3);
    print_qp_test('Test 5: MPC-Sized 40-Var System (x* = 1.5*ones)', norm(x5), norm(1.5*ones(n5,1)), err5, res5, pass5);
    if pass5, pass_count = pass_count + 1; end
    
    fprintf('\n========================================================\n');
    fprintf('  QP SOLVER UNIT TEST SUMMARY: %d/5 TESTS PASSED\n', pass_count);
    fprintf('========================================================\n\n');
end

function print_qp_test(title_str, val_found, val_target, err, res, is_pass)
    if is_pass
        fprintf('[PASS] %s\n', title_str);
    else
        fprintf('[FAIL] %s\n', title_str);
    end
    fprintf('       Found: %.4f | Target: %.4f | Error: %.1e\n', val_found, val_target, err);
    fprintf('       KKT Residuals: r_primal=%.1e | r_dual=%.1e | r_comp=%.1e | r_stat=%.1e\n\n', ...
        res.r_primal, res.r_dual, res.r_comp, res.r_stat);
end
