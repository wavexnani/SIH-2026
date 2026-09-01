classdef PredictorModule < handle
    % PREDICTORMODULE Obstacle Trajectory & Uncertainty Predictor (Stage 4)
    %
    % Purpose:
    %   Predicts future states and time-growing covariance matrices for dynamic
    %   and static obstacles over prediction horizon N_p.
    %
    % Formulation:
    %   p_x(k) = p_x(0) + v_x * k * dt
    %   p_y(k) = p_y(0) + v_y * k * dt
    %   sigma_y(k) = sigma_y0 + alpha_unc * k * dt
    
    properties
        dt          double = 0.10      % Sample time step (s)
        Np          int32 = 20         % Prediction horizon steps (2.0s lookahead)
        sigma_y0    double = 0.05      % Initial lateral uncertainty (m)
        alpha_unc   double = 0.02      % Uncertainty expansion rate (m/s)
    end
    
    methods
        function obj = PredictorModule(varargin)
            if nargin > 0 && isa(varargin{1}, 'SimulationConfig')
                cfg = varargin{1};
                obj.dt = cfg.dt;
            end
        end
        
        function predictions = predict(obj, world)
            % PREDICT Generates predicted trajectories and uncertainty for all obstacles.
            
            n_obs = world.n_static_obs;
            Np_double = double(obj.Np);
            
            predictions = struct('x', zeros(n_obs, Np_double), ...
                                 'y', zeros(n_obs, Np_double), ...
                                 'radius', zeros(n_obs, 1), ...
                                 'sigma_y', zeros(n_obs, Np_double));
            
            for i = 1:n_obs
                obs = world.static_obs(i, :);
                px0 = obs(1);
                py0 = obs(2);
                vx = 0.0; % static for baseline, dynamic velocity extendable
                vy = 0.0;
                r_obs = obs(4) / 2;
                
                predictions.radius(i) = r_obs;
                
                for k = 1:Np_double
                    t_f = k * obj.dt;
                    predictions.x(i, k) = px0 + vx * t_f;
                    predictions.y(i, k) = py0 + vy * t_f;
                    predictions.sigma_y(i, k) = obj.sigma_y0 + obj.alpha_unc * t_f;
                end
            end
        end
    end
end
