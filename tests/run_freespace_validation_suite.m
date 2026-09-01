function run_freespace_validation_suite()
    % RUN_FREESPACE_VALIDATION_SUITE Complete Validation Suite for FreeSpaceBoundProvider
    %
    % Executes:
    %   1. Regression Test A: 100% Corridor Parity Verification
    %   2. Test B: 2D Unstructured Road Scenario Verification
    %   3. Test C: Cattle Crossing Dynamic Obstacle Verification
    %   4. Stage 5.1 Multi-Vehicle Nominal Validation Suite
    
    addpath('planning', 'config', 'core', 'environment', 'stages', 'tests');
    
    fprintf('\n========================================================================================\n');
    fprintf('     COMPREHENSIVE VALIDATION SUITE: FLEXIBLE FREE-SPACE MAP INTEGRATION                \n');
    fprintf('========================================================================================\n');
    
    % Step 1: Regression Test A
    test_freespace_regression();
    
    % Step 2: Test B (Unstructured Road)
    test_unstructured_road();
    
    % Step 3: Test C (Cattle Crossing)
    test_cattle_crossing();
    
    % Step 4: Nominal Stage 5.1 Validation
    run_stage51_validation();
    
    fprintf('\n========================================================================================\n');
    fprintf('  ALL SUITE TESTS PASSED SUCCESSFULLY! FLEXIBLE FREE-SPACE MAP INTEGRATION VALIDATED. \n');
    fprintf('========================================================================================\n\n');
end
