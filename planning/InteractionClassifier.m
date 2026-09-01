classdef InteractionClassifier
    % INTERACTIONCLASSIFIER Categorizes multi-vehicle interaction dynamics
    %
    % Classifies interactions into 5 core classes:
    %   1. NO_INTERACTION
    %   2. SAME_DIRECTION_FOLLOWING
    %   3. CLOSING_VEHICLE
    %   4. ONCOMING_VEHICLE
    %   5. POTENTIAL_MERGE_CONFLICT
    %   6. OVERTAKING_CONFLICT
    
    methods (Static)
        function interactions = classify(detections, ego, ttc_matrix)
            % Classify interaction status for all detected vehicles
            %
            % Input:
            %   detections: Array of structs from MultiVehicleDetector
            %   ego:        EgoState object
            %   ttc_matrix: Array of computed TTC values corresponding to detections
            %
            % Output:
            %   interactions: Struct array with classification labels and risk indicators
            
            if isempty(detections)
                interactions = [];
                return;
            end
            
            n_det = length(detections);
            interactions = struct('id', {}, 'class_name', {}, 'primary_concern', {}, ...
                                  'risk_level', {}, 'ttc', {});
            
            for i = 1:n_det
                det = detections(i);
                ttc_val = ttc_matrix(i);
                
                res.id = det.id;
                res.ttc = ttc_val;
                
                % Distance cutoff for interaction relevance
                if det.d_rel > 75.0
                    res.class_name = 'NO_INTERACTION';
                    res.primary_concern = false;
                    res.risk_level = 0.0;
                elseif det.is_oncoming
                    res.class_name = 'ONCOMING_VEHICLE';
                    % Clear the threat once the oncoming vehicle has passed.
                    res.primary_concern = det.is_ahead && isfinite(ttc_val) && (ttc_val <= 6.0);
                    res.risk_level = max(0.0, 1.0 - det.d_rel / 75.0);
                elseif abs(det.dx) <= 4.5 && abs(det.dy) <= 3.6
                    res.class_name = 'OVERTAKING_CONFLICT';
                    res.primary_concern = true;
                    res.risk_level = 0.85;
                elseif det.is_adjacent_lane && abs(det.vy) > 0.3 && (det.dy * det.vy < 0)
                    res.class_name = 'POTENTIAL_MERGE_CONFLICT';
                    res.primary_concern = true;
                    res.risk_level = 0.75;
                elseif det.is_ahead && det.is_same_lane && (det.dvx < -2.0 || ttc_val < 4.0)
                    res.class_name = 'CLOSING_VEHICLE';
                    res.primary_concern = true;
                    res.risk_level = max(0.0, 1.0 - ttc_val / 5.0);
                elseif det.is_ahead && det.is_same_lane
                    res.class_name = 'SAME_DIRECTION_FOLLOWING';
                    res.primary_concern = (det.dx < 40.0);
                    res.risk_level = max(0.0, 1.0 - det.dx / 40.0);
                else
                    res.class_name = 'NO_INTERACTION';
                    res.primary_concern = false;
                    res.risk_level = 0.0;
                end
                
                interactions(i) = res;
            end
        end
    end
end
