classdef DynamicScenarioConfig
    % DYNAMICSCENARIOCONFIG Configuration for Dynamic Robustness Benchmark Suite
    
    properties
        n_trials            int32 = 50           % Monte Carlo trials per scenario
        dt                  double = 0.10        % Discretization timestep
        N_steps             int32 = 250          % Simulation steps per trial (25.0s)
        target_v            double = 5.0         % Nominal target velocity (m/s)
        W_req               double = 1.60        % Required minimum vehicle passage width (m)
        
        % Seed management
        base_seed           uint32 = 42
        
        % Output paths
        results_dir         char = 'tests/dynamic_robustness/results'
    end
end
