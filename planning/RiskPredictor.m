classdef RiskPredictor
    % RISKPREDICTOR Predictive risk and Time-To-Conflict (TTC) estimator
    %
    % Computes time-to-conflict metrics and predictive safety buffers
    % for all surrounding dynamic vehicles across the prediction horizon.
    
    methods (Static)
        function [ttc_vector, min_ttc] = predictTTC(detections, L_ego)
            % Compute Time-To-Conflict (TTC) for each detected agent
            %
            % Input:
            %   detections: Array of structs from MultiVehicleDetector
            %   L_ego:      Length of ego vehicle (m) [Default 4.5m]
            %
            % Output:
            %   ttc_vector: Array of TTC values (s)
            %   min_ttc:    Minimum TTC across all active ahead threats
            
            if nargin < 2, L_ego = 4.5; end
            
            if isempty(detections)
                ttc_vector = [];
                min_ttc = inf;
                return;
            end
            
            n_det = length(detections);
            ttc_vector = inf(1, n_det);
            min_ttc = inf;
            
            L_agent_default = 4.5; % Standard vehicle length
            
            for i = 1:n_det
                det = detections(i);
                
                % Focus TTC calculation on vehicles ahead or in potential conflict paths
                if det.dx > 0
                    clearance_dx = det.dx - (L_ego + L_agent_default)/2;
                    closing_rate = -det.dvx; % Positive when ego is closing in on agent
                    
                    if clearance_dx <= 0
                        ttc_vector(i) = 0.0;
                    elseif closing_rate > 0.05
                        ttc_vector(i) = clearance_dx / closing_rate;
                    else
                        ttc_vector(i) = inf;
                    end
                    
                    % Track minimum TTC for vehicles in same or overlapping lane
                    if abs(det.dy) <= 2.0
                        if ttc_vector(i) < min_ttc
                            min_ttc = ttc_vector(i);
                        end
                    end
                elseif det.is_oncoming && det.dx <= 75.0 && det.dx > -15.0
                    % Oncoming vehicle closing rate
                    clearance_dx = det.dx;
                    oncoming_closing_rate = det.v + abs(det.dvx);
                    if clearance_dx > 0 && oncoming_closing_rate > 0.1
                        ttc_vector(i) = clearance_dx / oncoming_closing_rate;
                        if abs(det.dy) <= 2.0 && ttc_vector(i) < min_ttc
                            min_ttc = ttc_vector(i);
                        end
                    end
                end
            end
        end
        
        function preds = predictTrajectories(detections, Np, dt)
            % Predict future trajectory coordinates for dynamic agents over horizon Np
            %
            % Input:
            %   detections: Array of structs from MultiVehicleDetector
            %   Np:         Prediction horizon length (steps, default 10)
            %   dt:         Timestep duration (seconds, default 0.10s)
            %
            % Output:
            %   preds: Array of structs containing predicted x_traj, y_traj, and spatial envelopes
            
            if nargin < 2, Np = 10; end
            if nargin < 3, dt = 0.10; end
            
            if isempty(detections)
                preds = [];
                return;
            end
            
            n_det = length(detections);
            preds = struct('id', {}, 'x_traj', {}, 'y_traj', {}, 'vx', {}, 'vy', {}, ...
                           'x_min', {}, 'x_max', {}, 'y_min', {}, 'y_max', {});
            
            for i = 1:n_det
                det = detections(i);
                
                pred_i.id = det.id;
                pred_i.vx = det.vx;
                pred_i.vy = det.vy;
                
                % Multi-step linear kinematic propagation
                k_vec = 1:Np;
                t_vec = k_vec * dt;
                
                pred_i.x_traj = det.x + det.vx * t_vec;
                pred_i.y_traj = det.y + det.vy * t_vec;
                
                pred_i.x_min = min([det.x, pred_i.x_traj]);
                pred_i.x_max = max([det.x, pred_i.x_traj]);
                pred_i.y_min = min([det.y, pred_i.y_traj]);
                pred_i.y_max = max([det.y, pred_i.y_traj]);
                
                preds(i) = pred_i;
            end
        end
    end
end
