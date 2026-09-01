classdef MultiVehicleDetector
    % MULTIVEHICLEDETECTOR Multi-Vehicle Detection and Relative Metric Extraction
    %
    % Extracts relative position, velocity, heading, and spatial placement
    % for all dynamic vehicles/agents surrounding the ego vehicle.
    
    methods (Static)
        function detections = detect(world, ego)
            % Extract relative perception metrics for dynamic agents
            %
            % Input:
            %   world: WorldState object
            %   ego:   EgoState object
            %
            % Output:
            %   detections: Array of structs containing relative agent metrics
            
            detections = struct('id', {}, 'x', {}, 'y', {}, 'vx', {}, 'vy', {}, ...
                                'v', {}, 'theta', {}, 'dx', {}, 'dy', {}, ...
                                'd_rel', {}, 'dvx', {}, 'dvy', {}, ...
                                'is_ahead', {}, 'is_behind', {}, ...
                                'is_same_lane', {}, 'is_adjacent_lane', {}, ...
                                'is_oncoming', {});
            
            n_agents = world.n_agents;
            ego_x = ego.x;
            ego_y = ego.y;
            ego_v = ego.v;
            ego_th = ego.theta;
            ego_vx = ego_v * cos(ego_th);
            ego_vy = ego_v * sin(ego_th);
            
            count = 0;
            for i = 1:n_agents
                agent = world.agents(i);
                if isempty(agent) || agent.id == 0, continue; end
                if agent.x < -10.0 || (agent.x == 0 && agent.y == 0 && agent.vx == 0 && agent.vy == 0), continue; end
                
                count = count + 1;
                det.id = agent.id;
                det.x = agent.x;
                det.y = agent.y;
                det.vx = agent.vx;
                det.vy = agent.vy;
                det.v = hypot(agent.vx, agent.vy);
                det.theta = atan2(agent.vy, agent.vx + 1e-6);
                
                % Relative quantities (ego-frame aligned)
                det.dx = agent.x - ego_x;
                det.dy = agent.y - ego_y;
                det.d_rel = hypot(det.dx, det.dy);
                det.dvx = agent.vx - ego_vx;
                det.dvy = agent.vy - ego_vy;
                
                % Spatial indicators
                det.is_ahead = (det.dx > 0);
                det.is_behind = (det.dx < 0);
                det.is_same_lane = (abs(det.dy) <= 1.2); % Within vehicle footprint collision path
                det.is_adjacent_lane = (abs(det.dy) > 1.2 && abs(det.dy) <= 3.6);
                
                % Heading orientation indicator
                det.is_oncoming = (cos(det.theta - ego_th) < -0.7);
                
                detections(count) = det;
            end
            
            % Process active static obstacles on the road as stationary detections
            if world.n_static_obs > 0
                for j = 1:world.n_static_obs
                    obs = world.static_obs(j, :);
                    if obs(1) <= 0.0 || obs(2) <= 0.0 || obs(1) > world.road_length, continue; end
                    
                    count = count + 1;
                    det.id = 1000 + j;
                    det.x = obs(1);
                    det.y = obs(2);
                    det.vx = 0.0;
                    det.vy = 0.0;
                    det.v = 0.0;
                    det.theta = 0.0;
                    
                    det.dx = obs(1) - ego_x;
                    det.dy = obs(2) - ego_y;
                    det.d_rel = hypot(det.dx, det.dy);
                    det.dvx = 0.0 - ego_vx;
                    det.dvy = 0.0 - ego_vy;
                    
                    det.is_ahead = (det.dx > 0);
                    det.is_behind = (det.dx < 0);
                    det.is_same_lane = (abs(det.dy) <= 1.2);
                    det.is_adjacent_lane = (abs(det.dy) > 1.2 && abs(det.dy) <= 3.6);
                    det.is_oncoming = false;
                    
                    detections(count) = det;
                end
            end
        end
    end
end
